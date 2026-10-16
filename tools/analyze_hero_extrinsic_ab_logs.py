#!/usr/bin/env python3
"""Match stopped-frame RX captures with later DRY_RUN TX logs; no hardware control.
TX-RX is a requested correction, NOT an execution error or aiming accuracy measurement.
"""
import argparse
import csv
import datetime as dt
import json
import re
from pathlib import Path
import numpy as np

TIME=re.compile(r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}(?:\.\d{3})?) (.*)$")
PAIR=re.compile(r"(\w+)=(-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)")
def wrap(value):return (np.asarray(value)+180.)%360.-180.
def median_yaw(values):
    unwrapped=np.degrees(np.unwrap(np.radians(values)))
    return float(wrap(np.median(unwrapped)))

def analyze(directory):
    session=json.loads((directory/"session.json").read_text())
    timezone=dt.timezone(dt.timedelta(seconds=session["utc_offset_seconds"]))
    packets=[]
    for line in (directory/"xuc.log").read_text(errors="replace").splitlines():
        match=TIME.match(line)
        if not match or "[TX/DRY_RUN" not in match[2]:continue
        fields={key:float(value) for key,value in PAIR.findall(match[2])}
        if not all(key in fields for key in ["yaw","pitch","distance"]):continue
        timestamp=dt.datetime.fromisoformat(match[1]).replace(tzinfo=timezone).timestamp()
        packets.append((timestamp,fields))
    rows=[]
    metadata_paths=sorted(directory.glob("frame_*.json"),key=lambda p:int(p.stem.split("_")[1]))
    for path in metadata_paths:
        meta=json.loads(path.read_text())
        frame_time=meta["image_read_unix_ms"]/1000.
        # Static-pose association only, not an exposure/serial synchronization claim.
        selected=[fields for timestamp,fields in packets
                  if frame_time+0.05<=timestamp<=frame_time+0.75 and fields["distance"]>0]
        row={"run_directory":str(directory),"candidate":session["candidate"],
             "distance_label":session["distance_label"],"sample_id":int(path.stem.split("_")[1]),
             "image_read_unix_ms":meta["image_read_unix_ms"],"RX_yaw_deg":meta["yaw_deg"],
             "RX_pitch_deg":meta["pitch_deg"],"RX_roll_deg":meta["roll_deg"],
             "rx_age_ms":meta["rx_age_ms"],"valid_TX_count":len(selected),
             "median_TX_yaw_deg":"","median_TX_pitch_deg":"","median_target_distance_m":"",
             "requested_yaw_correction_deg":"","requested_pitch_correction_deg":"",
             "status":"no_valid_target_TX"}
        if selected:
            yaw=median_yaw([p["yaw"] for p in selected])
            pitch=float(np.median([p["pitch"] for p in selected]))
            row.update(median_TX_yaw_deg=yaw,median_TX_pitch_deg=pitch,
                       median_target_distance_m=float(np.median([p["distance"] for p in selected])),
                       requested_yaw_correction_deg=float(wrap(yaw-meta["yaw_deg"])),
                       requested_pitch_correction_deg=pitch-meta["pitch_deg"],status="associated")
        rows.append(row)
    valid=[row for row in rows if row["status"]=="associated"]
    summary={"run_directory":str(directory),"candidate":session["candidate"],
             "distance_label":session["distance_label"],"captured_frames":len(rows),
             "frames_with_valid_target_TX":len(valid)}
    if valid:
        yaw=np.degrees(np.unwrap(np.radians([row["median_TX_yaw_deg"] for row in valid])))
        pitch=np.array([row["median_TX_pitch_deg"] for row in valid])
        summary.update(TX_yaw_range_deg=float(np.ptp(yaw)),TX_pitch_range_deg=float(np.ptp(pitch)),
                       median_TX_yaw_deg=median_yaw(yaw),median_TX_pitch_deg=float(np.median(pitch)))
    return rows,summary

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("directories",nargs="+")
    ap.add_argument("--output-dir",required=True)
    args=ap.parse_args()
    all_rows=[];summaries=[]
    for name in args.directories:
        rows,summary=analyze(Path(name).resolve());all_rows.extend(rows);summaries.append(summary)
    output=Path(args.output_dir).resolve();output.mkdir(parents=True,exist_ok=False)
    if all_rows:
        with (output/"frame_TX_RX.csv").open("w",newline="") as file:
            writer=csv.DictWriter(file,fieldnames=list(all_rows[0]))
            writer.writeheader();writer.writerows(all_rows)
    report={"runs":summaries,
      "limitations":["Stop before P and hold after P; association uses 0.05-0.75 s after frame read.",
       "TX is DRY_RUN packet content, not an executed command.",
       "Requested TX-RX correction is not aiming error unless RX is an independently established aim reference.",
       "Absolute accuracy comparison requires the target to remain fixed between A/B and a physical aiming reference."]}
    (output/"summary.json").write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False,indent=2))
    print("OUTPUT_DIRECTORY",output)

if __name__=="__main__":main()
