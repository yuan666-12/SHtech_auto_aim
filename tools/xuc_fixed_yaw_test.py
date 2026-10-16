#!/usr/bin/env python3
"""One-shot MCU-native fixed-yaw test. No camera and no auto-aim process.
Default: absolute yaw 0 deg, hold starting pitch, fire=0, 200 Hz for 5 s.
Only sends after fresh CRC-valid RX reports fire_mode=1. No return motion.
"""
import argparse
import datetime
import fcntl
import math
import os
from pathlib import Path
import select
import signal
import struct
import termios
import time

DEVICE = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'

def crc16(data):
    crc = 0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc

def control_packet(yaw, pitch):
    packet = bytearray(31)
    packet[0] = 0xa5
    struct.pack_into('<5f', packet, 1, pitch, yaw, 0.0, 0.0, 0.4)
    # byte 21 fire=0; bytes 22..24 padding; bytes 25..28 v_y=0.
    struct.pack_into('<H', packet, 29, crc16(packet[:29]))
    return bytes(packet)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default=DEVICE)
    parser.add_argument('--yaw', type=float, default=0.0, help='absolute MCU-native yaw, degrees')
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--wait', type=float, default=120.0)
    parser.add_argument('--log', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.yaw) or not -180 <= args.yaw <= 180:
        parser.error('yaw must be finite and within [-180,180]')
    if not 0 < args.duration <= 30 or not 0 < args.wait <= 300:
        parser.error('duration must be (0,30] seconds; wait must be (0,300] seconds')
    project = Path(__file__).resolve().parent.parent
    log_path = args.log or project / 'log' / ('xuc-fixed-yaw-' + datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S-%f') + '.log')
    log_path.parent.mkdir(parents=True, exist_ok=True)
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    fd = None
    old = None
    tx_count = rx_count = crc_fail = 0
    latest = None
    last_rx = 0.0
    buffer = bytearray()
    with log_path.open('x', buffering=1) as log:
        def record(message, terminal=False):
            log.write(datetime.datetime.now().isoformat(timespec='milliseconds') + ' ' + message + '\n')
            if terminal:
                print(message, flush=True)
        print('LOG=' + str(log_path), flush=True)
        def receive():
            nonlocal latest, last_rx, rx_count, crc_fail
            while select.select([fd], [], [], 0)[0]:
                data = os.read(fd, 4096)
                if not data:
                    raise RuntimeError('serial disconnected')
                buffer.extend(data)
            while len(buffer) >= 28:
                if buffer[0] != 0x5a:
                    del buffer[0]
                    continue
                packet = bytes(buffer[:28])
                if crc16(packet[:26]) != int.from_bytes(packet[26:28], 'little'):
                    crc_fail += 1
                    del buffer[0]
                    continue
                del buffer[:28]
                roll, pitch, yaw = struct.unpack_from('<3f', packet, 2)
                if not all(math.isfinite(v) for v in (roll, pitch, yaw)):
                    raise RuntimeError('invalid RX attitude')
                latest = (packet[1], roll, pitch, yaw)
                last_rx = time.monotonic()
                rx_count += 1
                record('RX flags=%02X fire_mode=%d roll=%.6f pitch=%.6f yaw=%.6f raw=%s' % (packet[1], (packet[1] >> 7) & 1, roll, pitch, yaw, packet.hex(' ')))
        try:
            fd = os.open(args.device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            fcntl.ioctl(fd, termios.TIOCEXCL)
            old = termios.tcgetattr(fd)
            cfg = termios.tcgetattr(fd)
            cfg[0] = cfg[1] = cfg[3] = 0
            cfg[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            cfg[4] = cfg[5] = termios.B460800
            cfg[6][termios.VMIN] = cfg[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, cfg)
            termios.tcflush(fd, termios.TCIFLUSH)
            record('ARMED wait=%.1fs absolute_yaw=%.3f fire=0; no detection; waiting for fresh fire_mode=1' % (args.wait, args.yaw), True)
            deadline = time.monotonic() + args.wait
            while not stopped and time.monotonic() < deadline:
                receive()
                if latest and time.monotonic() - last_rx < 0.1 and latest[0] & 0x80:
                    break
                time.sleep(0.005)
            else:
                raise RuntimeError('cancelled or self-aim mode wait expired')
            pitch = latest[2]
            if not 0 <= pitch <= 43:
                raise RuntimeError('starting pitch outside [0,43], refusing control')
            packet = control_packet(args.yaw, pitch)
            record('START rx_yaw=%.3f target_yaw=%.3f fixed_pitch=%.3f duration=%.1fs' % (latest[3], args.yaw, pitch, args.duration), True)
            start = next_tick = time.monotonic()
            next_report = start
            while not stopped and time.monotonic() - start < args.duration:
                receive()
                now = time.monotonic()
                if now - last_rx > 0.1:
                    raise RuntimeError('RX stale >100ms; stopped sending')
                if not latest[0] & 0x80:
                    raise RuntimeError('fire_mode=0; stopped sending')
                written = os.write(fd, packet)
                if written != 31:
                    raise RuntimeError('short write %d/31; stopped sending' % written)
                tx_count += 1
                record('TX t=%.6f yaw=%.6f pitch=%.6f yaw_diff=0 pitch_diff=0 distance=0.4 fire=0 written=31 raw=%s' % (now - start, args.yaw, pitch, packet.hex(' ')))
                if now >= next_report:
                    record('STATUS t=%.3f target_yaw=%.3f actual_yaw=%.3f fixed_pitch=%.3f actual_pitch=%.3f TX=%d' % (now - start, args.yaw, latest[3], pitch, latest[2], tx_count), True)
                    next_report = now + 0.5
                next_tick += 0.005
                time.sleep(max(0, next_tick - time.monotonic()))
            record('DONE cancelled=%s; stopped sending, no return command' % stopped, True)
        except (OSError, RuntimeError) as exc:
            record('STOP ' + str(exc), True)
        finally:
            if fd is not None:
                try:
                    if old is not None:
                        termios.tcsetattr(fd, termios.TCSANOW, old)
                    fcntl.ioctl(fd, termios.TIOCNXCL)
                finally:
                    os.close(fd)
            record('CLOSED RX=%d TX=%d CRC_fail_candidates=%d' % (rx_count, tx_count, crc_fail), True)

if __name__ == '__main__':
    main()
