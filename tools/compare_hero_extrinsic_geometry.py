#!/usr/bin/env python3
"""Compare two extrinsics on identical fixed-board samples, using a common pivot origin.
Reproduces the static angle formula only; it does not run inference, prediction or UART.
The ideal world-bearing calculation is a diagnostic, not measured aiming ground truth.
"""
import argparse
import csv
import json
from pathlib import Path
import cv2
import numpy as np
from solve_hero_extrinsic import load, BOARD, SQUARE_M

def wrap(value):
    return (np.asarray(value) + 180.) % 360. - 180.

def stats(yaw, pitch):
    yaw = np.degrees(np.unwrap(np.radians(yaw)))
    pitch = np.asarray(pitch)
    residual = np.column_stack((wrap(yaw-np.median(yaw)), pitch-np.median(pitch)))
    return {"median_yaw_deg": float(np.median(yaw)),
            "median_pitch_deg": float(np.median(pitch)),
            "yaw_range_deg": float(np.ptp(yaw)), "pitch_range_deg": float(np.ptp(pitch)),
            "angle_rms_about_median_deg": float(np.sqrt(np.mean(np.sum(residual**2,axis=1))))}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("directory")
    ap.add_argument("--config-dir",default="asset/extrinsic_ab_20261012")
    ap.add_argument("--output-dir",required=True)
    args=ap.parse_args()
    data=Path(args.directory).resolve()
    cfg=Path(args.config_dir).resolve()
    output=Path(args.output_dir).resolve()
    output.mkdir(parents=True,exist_ok=False)
    origin=json.loads((cfg/"origin_reference.json").read_text())
    pivot_imu=np.array(origin["T_pivot_to_imu_m"])
    gs,rtcs,ttcs,ids=load(str(data))
    if len(ids)<3:raise ValueError("fewer than 3 detectable paired images")
    center=np.array([(BOARD[0]-1)*SQUARE_M/2.,(BOARD[1]-1)*SQUARE_M/2.,0.])
    pcs=np.array([(r@center.reshape(3,1)+t).reshape(3) for r,t in zip(rtcs,ttcs)])
    ypr=np.array([np.loadtxt(data/f"euler_{i}.txt").reshape(-1)[:3] for i in ids])
    results={};rows=[];arrays={}
    for label in ["A","B"]:
        fs=cv2.FileStorage(str(cfg/f"camera_{label}.yml"),cv2.FILE_STORAGE_READ)
        if not fs.isOpened():raise ValueError(f"missing camera {label}")
        r=fs.getNode("R_c2i").mat();t=fs.getNode("T_c2i").mat().reshape(3)
        fs.release()
        pi=np.array([r@pc+t for pc in pcs])
        pp=pi+pivot_imu
        pw=np.array([g@point for g,point in zip(gs,pp)])
        if np.any(pi[:,2]<=0) or np.any(pp[:,2]<=0):raise ValueError("target behind reference")
        # Match current Planner::cal_gimbal_target with no drop and no fixed compensation.
        legacy_imu_yaw=wrap(ypr[:,0]+np.degrees(np.arctan(pi[:,0]/pi[:,2])))
        legacy_imu_pitch=ypr[:,1]+np.degrees(np.arctan(pi[:,1]/pi[:,2]))
        legacy_pivot_yaw=wrap(ypr[:,0]+np.degrees(np.arctan(pp[:,0]/pp[:,2])))
        legacy_pivot_pitch=ypr[:,1]+np.degrees(np.arctan(pp[:,1]/pp[:,2]))
        ideal_yaw=wrap(np.degrees(np.arctan2(-pw[:,0],pw[:,1])))
        ideal_pitch=np.degrees(np.arctan2(pw[:,2],np.hypot(pw[:,0],pw[:,1])))
        center_world=np.median(pw,axis=0)
        position_error=np.linalg.norm(pw-center_world,axis=1)*1000.
        coupling=np.column_stack((wrap(legacy_pivot_yaw-ideal_yaw),legacy_pivot_pitch-ideal_pitch))
        results[label]={
          "R":r.tolist(),"T_true_IMU_to_camera_m":t.tolist(),
          "T_pivot_to_camera_m":(t+pivot_imu).tolist(),
          "sample_ids":list(map(int,ids)),
          "fixed_target_position_rms_mm":float(np.sqrt(np.mean(position_error**2))),
          "fixed_target_position_p95_mm":float(np.percentile(position_error,95)),
          "static_current_Planner_formula_IMU_origin":stats(legacy_imu_yaw,legacy_imu_pitch),
          "static_same_formula_pivot_origin":stats(legacy_pivot_yaw,legacy_pivot_pitch),
          "ideal_world_bearing_pivot_origin":stats(ideal_yaw,ideal_pitch),
          "angle_addition_vs_ideal_pivot_rms_deg":float(np.sqrt(np.mean(np.sum(coupling**2,axis=1)))),
          "angle_addition_vs_ideal_pivot_max_deg":float(np.max(np.linalg.norm(coupling,axis=1)))}
        arrays[label]=(legacy_imu_yaw,legacy_imu_pitch,ideal_yaw,ideal_pitch)
        for j,sid in enumerate(ids):
            rows.append({"candidate":label,"sample_id":int(sid),
             "RX_yaw_deg":ypr[j,0],"RX_pitch_deg":ypr[j,1],"RX_roll_deg":ypr[j,2],
             "camera_center_x_m":pcs[j,0],"camera_center_y_m":pcs[j,1],"camera_center_z_m":pcs[j,2],
             "world_pivot_x_m":pw[j,0],"world_pivot_y_m":pw[j,1],"world_pivot_z_m":pw[j,2],
             "formula_yaw_IMU_origin_deg":legacy_imu_yaw[j],"formula_pitch_IMU_origin_deg":legacy_imu_pitch[j],
             "formula_yaw_pivot_origin_deg":legacy_pivot_yaw[j],"formula_pitch_pivot_origin_deg":legacy_pivot_pitch[j],
             "ideal_world_yaw_deg":ideal_yaw[j],"ideal_world_pitch_deg":ideal_pitch[j],
             "position_error_about_model_center_mm":position_error[j]})
    delta_yaw=wrap(arrays["B"][0]-arrays["A"][0])
    delta_pitch=arrays["B"][1]-arrays["A"][1]
    summary={"data_directory":str(data),"sample_count":len(ids),"origin_reference":origin,
     "candidate_statistics":results,
     "B_minus_A_current_static_formula":{"median_yaw_deg":float(np.median(delta_yaw)),
       "median_pitch_deg":float(np.median(delta_pitch)),
       "yaw_min_max_deg":[float(delta_yaw.min()),float(delta_yaw.max())],
       "pitch_min_max_deg":[float(delta_pitch.min()),float(delta_pitch.max())]},
     "limitations":["These are static formula outputs, NOT recorded UART TX.",
       "B was fitted on these same 34 images; lower training residual is not independent validation.",
       "Ideal bearing assumes intersecting ideal yaw/pitch axes; no measured barrel aiming ground truth.",
       "Both extrinsics use true IMU origin; add measured pivot-to-IMU offset for fixed-pivot world geometry."]}
    with (output/"per_frame.csv").open("w",newline="") as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    (output/"summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    report="# 同一数据的 A/B 几何对照\n\n"
    report+="所有结果来自同一批图像和 RX，目标为棋盘几何中心，弹道和固定角补偿均为零。\n"
    report+="A/B 配置均使用真实 IMU 原点；固定目标几何计算另外加入已确认的交点到 IMU 向量。\n"
    report+="下面的角度是静态公式输出，不是实际串口 TX。B 使用本批数据拟合，不能以训练误差直接认定实车更准。\n\n"
    report+="| 指标 | A：R-only＋机械 | B：联合，换到同一原点 |\n|---|---:|---:|\n"
    for label,key in [("固定目标位置 RMS / mm","fixed_target_position_rms_mm")]:
        report+=f"| {label} | {results['A'][key]:.6f} | {results['B'][key]:.6f} |\n"
    for title,key in [("当前公式 yaw 变化范围 / deg","yaw_range_deg"),
                      ("当前公式 pitch 变化范围 / deg","pitch_range_deg")]:
        report+=f"| {title} | {results['A']['static_current_Planner_formula_IMU_origin'][key]:.6f} | {results['B']['static_current_Planner_formula_IMU_origin'][key]:.6f} |\n"
    report+="\n完整三维方向参考及转轴原点对照见 summary.json；逐样本结果见 per_frame.csv。\n"
    (output/"结果说明.md").write_text(report)
    print(json.dumps(summary,ensure_ascii=False,indent=2))
    print("OUTPUT_DIRECTORY",output)

if __name__=="__main__":main()
