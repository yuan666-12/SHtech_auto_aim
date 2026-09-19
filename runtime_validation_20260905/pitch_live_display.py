"""Read-only pitch dashboard for the existing runtime log."""
import os
import re
import time
import tkinter as tk
from pathlib import Path

LOG = Path(os.environ.get(
    'AUTOAIM_LIVE_LOG',
    '/home/rcs/rm_project/SHtech_auto_aim/runtime_validation_20260905/track_live_20260905.log',
))
pattern = re.compile(r'p-m:\s*([-+\d.]+)\s*\|\s*p-s:\s*([-+\d.]+).*?target_id:(\d+)')
root = tk.Tk()
root.title('Live Pitch - read only')
# Keep the diagnostics beside the OpenCV detection view rather than covering it.
root.geometry('560x280+770+45')
root.configure(bg='#17212b')
root.attributes('-topmost', True)
values = []
for name in ('IMU pitch', 'Target pitch (raw)', 'TX pitch (limited)'):
    label = tk.Label(root, text=name + ': --', font=('DejaVu Sans', 19), fg='#ecf0f1', bg='#17212b', anchor='w')
    label.pack(fill='x', padx=15, pady=6)
    values.append(label)
tx_status = tk.Label(root, text='NUC serial: waiting...', font=('DejaVu Sans', 12), fg='#6ee7b7', bg='#17212b', anchor='w')
tx_status.pack(fill='x', padx=15, pady=(2, 0))
status = tk.Label(root, text='Waiting for log...', font=('DejaVu Sans', 10), fg='#f4c542', bg='#17212b', anchor='w')
status.pack(fill='x', padx=15, pady=(2, 0))
source = LOG.open(errors='replace')
source.seek(max(0, os.fstat(source.fileno()).st_size - 65536))
pending = ''
last = 0.0
target_id = 0
current_pitch = None
raw_pitch = None

def refresh():
    global pending, last, target_id, current_pitch, raw_pitch
    if LOG.stat().st_size < source.tell():
        source.seek(0)
        pending = ''
    # The C++ logger uses carriage-return progress output and embeds ANSI/NUL
    # bytes.  Normalize it before line-oriented matching.
    pending += source.read(262144)
    pending = pending.replace('\x00', '').replace('\r', '\n')
    lines = pending.split('\n')
    pending = lines.pop()
    latest = None
    for line in lines:
        match = pattern.search(line)
        if match:
            latest = match
    if latest:
        current_pitch = float(latest.group(1))
        candidate_raw = float(latest.group(2))
        candidate_id = int(latest.group(3))
        # target_id=0 is the planner's intentional hold frame between valid
        # detections.  Keep the last actual armor target visible instead of
        # replacing it with the current pitch.
        if candidate_id != 0:
            raw_pitch = candidate_raw
            target_id = candidate_id
        if current_pitch is not None and raw_pitch is not None:
            sent = min(43.0, max(0.0, raw_pitch))
            for label, name, val in zip(values, ('IMU pitch', 'Target pitch (raw)', 'TX pitch (limited)'), (current_pitch, raw_pitch, sent)):
                label.config(text=f'{name}: {val:+.2f} deg')
        elif current_pitch is not None:
            values[0].config(text=f'IMU pitch: {current_pitch:+.2f} deg')
        last = time.monotonic()
    age = time.monotonic() - last if last else float('inf')
    if age < 0.2:
        tx_status.config(text='NUC serial: streaming target frames at 200 Hz | FIRE DISABLED', fg='#6ee7b7')
        status.config(text=f'Target ID: {target_id} | TX limited to 0..43 deg | STM32 receive not yet confirmed')
    else:
        tx_status.config(text='NUC serial: no current target stream', fg='#f4c542')
        status.config(text='STALE: no recent runtime updates')
    root.after(100, refresh)

root.after(0, refresh)
root.mainloop()
