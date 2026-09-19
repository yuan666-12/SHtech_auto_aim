#!/usr/bin/env python3
import cv2
import numpy as np
import os

def main():
    # 保留之前手眼标定得到的旋转矩阵
    R = np.array([
        [ 0.94645043,  0.31673275, -0.06254564],
        [-0.29834896,  0.93208501,  0.20543961],
        [ 0.12336731, -0.17577798,  0.97666914]
    ])

    print("=" * 60)
    print("外参修正：手动填入平移向量")
    print("=" * 60)
    print("\n之前平移 74.8cm 过大，原因是 euler 文件缺少平移数据。")
    print("旋转矩阵已保留，平移向量需要物理测量。\n")

    print("测量方法：")
    print("  1. 找到云台旋转中心（yaw 轴与 pitch 轴的交点）")
    print("  2. 找到相机光心（镜头中心）")
    print("  3. 测量两者在云台坐标系下的相对位置\n")

    print("云台坐标系约定（RoboMaster 常见）：")
    print("  x: 左(+) / 右(-)")
    print("  y: 前(+) / 后(-)   <- 相机通常在云台前方")
    print("  z: 上(+) / 下(-)   <- 相机通常在云台上方\n")

    print("如果你暂时无法测量，可参考 RM 步兵常见估算值：")
    print("  [0, 120, 80] mm  （前方 12cm，上方 8cm）")
    print("  ⚠️ 这只是估算，后续务必精确测量！\n")

    tx = float(input("平移 x (mm) [默认 0]: ") or "0")
    ty = float(input("平移 y (mm) [默认 120]: ") or "120")
    tz = float(input("平移 z (mm) [默认 80]: ") or "80")

    t = np.array([[tx], [ty], [tz]])

    # 保存到 asset/
    output_dir = os.path.expanduser("~/rm_project/SHtech_auto_aim/asset")
    os.makedirs(output_dir, exist_ok=True)

    fs = cv2.FileStorage(os.path.join(output_dir, "extrinsic-param.yml"), cv2.FILE_STORAGE_WRITE)
    fs.write("R_cam2gimbal", R)
    fs.write("t_cam2gimbal", t)
    fs.release()

    print(f"\n[已保存] {output_dir}/extrinsic-param.yml")
    print(f"\n>>> 修正后的外参 <<<")
    print(f"旋转矩阵:\n{R}")
    print(f"平移向量: [{tx:.2f}, {ty:.2f}, {tz:.2f}] mm")

    # 验证旋转矩阵
    det = np.linalg.det(R)
    print(f"\n旋转矩阵行列式: {det:.6f} (应接近 1.0)")

    # 提取欧拉角
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    if sy > 1e-6:
        roll = np.degrees(np.arctan2(R[2,1], R[2,2]))
        pitch = np.degrees(np.arctan2(-R[2,0], sy))
        yaw = np.degrees(np.arctan2(R[1,0], R[0,0]))
        print(f"对应欧拉角 (ZYX): roll={roll:.2f}°, pitch={pitch:.2f}°, yaw={yaw:.2f}°")

    print("\n" + "=" * 60)
    print("下一步：")
    print("1. 把 extrinsic-param.yml 的路径填入 launch.cfg")
    print("2. 运行自瞄程序，做 PnP 测距验证")
    print("3. 如果测距误差大，微调平移向量")
    print("=" * 60)

if __name__ == "__main__":
    main()