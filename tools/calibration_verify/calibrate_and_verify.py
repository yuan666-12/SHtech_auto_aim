#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机标定 + 验证脚本
用法（从当前项目根目录执行，数据目录按实际采集位置填写）：
    python3 tools/calibration_verify/calibrate_and_verify.py \
        --intrinsic_dir ./calib_intrinsic \
        --extrinsic_dir ./calib_extrinsic \
        --output ./asset \
        --pattern 11 8 \
        --square 20.0
"""

import cv2
import numpy as np
import os
import glob
import argparse
from collections import namedtuple

CalibrationResult = namedtuple('CalibrationResult', ['mtx', 'dist', 'rms', 'img_size'])


def main():
    parser = argparse.ArgumentParser(description='相机内外参标定与验证')
    parser.add_argument('--intrinsic_dir', required=True, help='内参图片目录路径')
    parser.add_argument('--extrinsic_dir', default='', help='外参图片目录路径（可选）')
    parser.add_argument('--output', default='.', help='camera-param.yml 输出目录')
    parser.add_argument('--pattern', type=int, nargs=2, default=[11, 8], metavar=('W', 'H'),
                        help='棋盘格内角点数量，默认 11 8')
    parser.add_argument('--square', type=float, default=20.0, help='棋盘格方格边长 mm')
    parser.add_argument('--show', action='store_true', help='是否显示角点检测过程（NUC 无桌面时别开）')
    args = parser.parse_args()

    pattern_size = (args.pattern[0], args.pattern[1])
    square_size = args.square

    # ==================== Phase 1: 内参标定 ====================
    print("=" * 60)
    print("Phase 1: 内参标定")
    print("=" * 60)

    intrinsic = calibrate_intrinsic(args.intrinsic_dir, pattern_size, square_size, args.show)
    if intrinsic is None:
        print("[错误] 内参标定失败，请检查图片和棋盘格参数")
        return

    mtx, dist, rms, img_size = intrinsic
    print(f"\n>>> 标定结果 <<<")
    print(f"图像分辨率: {img_size[0]} x {img_size[1]}")
    print(f"重投影误差 RMS: {rms:.4f} px")
    print(f"内参矩阵 (camera_matrix):\n{mtx}")
    print(f"畸变系数 (distortion):\n{dist.ravel()}")

    # 保存 YAML
    os.makedirs(args.output, exist_ok=True)
    yaml_path = os.path.join(args.output, "camera-param.yml")
    save_opencv_yaml(yaml_path, mtx, dist, img_size)
    print(f"\n[已保存] {yaml_path}")

    # ==================== Phase 2: 外参标定（手眼） ====================
    if args.extrinsic_dir and os.path.exists(args.extrinsic_dir):
        print("\n" + "=" * 60)
        print("Phase 2: 外参标定（手眼标定）")
        print("=" * 60)
        calibrate_extrinsic(args.extrinsic_dir, mtx, dist, pattern_size, square_size)
    else:
        print("\n[跳过] 未提供外参目录，跳过手眼标定")

    # ==================== Phase 3: 验证 ====================
    print("\n" + "=" * 60)
    print("Phase 3: 内参验证")
    print("=" * 60)
    verify_intrinsic(args.intrinsic_dir, mtx, dist, pattern_size, square_size, args.output, args.show)

    print("\n" + "=" * 60)
    print("全部完成。查看上面的 RMS 误差判断标定质量。")
    print("=" * 60)


# -----------------------------------------------------------------------------
# 内参标定
# -----------------------------------------------------------------------------
def calibrate_intrinsic(image_dir, pattern_size, square_size, show=False):
    """
    基于棋盘格图片标定相机内参。
    pattern_size: (宽方向角点数, 高方向角点数)
    """
    # 准备世界坐标系下的 3D 点
    objp = np.zeros((pattern_size[1] * pattern_size[0], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size  # 缩放到实际物理尺寸

    objpoints = []   # 3D 点集列表
    imgpoints = []   # 2D 角点列表
    valid_images = []

    # 读取所有图片
    exts = ['*.png', '*.jpg', '*.jpeg', '*.bmp']
    images = []
    for ext in exts:
        images.extend(glob.glob(os.path.join(image_dir, ext)))
    images = sorted(images)

    if len(images) == 0:
        print(f"[错误] 目录 {image_dir} 中没有找到图片")
        return None

    print(f"找到 {len(images)} 张图片，开始检测角点...")

    h, w = None, None
    for i, fname in enumerate(images):
        img = cv2.imread(fname)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if h is None:
            h, w = gray.shape[:2]

        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if ret:
            # 亚像素精化
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            objpoints.append(objp)
            imgpoints.append(corners2)
            valid_images.append(fname)

            if show:
                cv2.drawChessboardCorners(img, pattern_size, corners2, ret)
                cv2.imshow('Corners', cv2.resize(img, None, fx=0.5, fy=0.5))
                cv2.waitKey(200)

            print(f"  [{len(objpoints)}/{i+1}] {os.path.basename(fname)} -> 角点检测成功")
        else:
            print(f"  [跳过] {os.path.basename(fname)} -> 未检测到角点")

    if show:
        cv2.destroyAllWindows()

    if len(objpoints) < 5:
        print(f"[错误] 仅 {len(objpoints)} 张有效图片，至少需要 5 张")
        return None

    print(f"\n有效图片: {len(objpoints)} 张，开始计算内参...")

    # 标定
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, (w, h), None, None
    )

    if not ret:
        print("[错误] cv2.calibrateCamera 返回失败")
        return None

    # 计算每张图的重投影误差
    total_error = 0
    max_error = 0
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], proj, cv2.NORM_L2) / len(proj)
        total_error += error
        max_error = max(max_error, error)

    rms = total_error / len(objpoints)

    return CalibrationResult(mtx=mtx, dist=dist, rms=rms, img_size=(w, h))


# -----------------------------------------------------------------------------
# 保存 YAML（OpenCV 格式）
# -----------------------------------------------------------------------------
def save_opencv_yaml(path, mtx, dist, img_size):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", img_size[0])
    fs.write("image_height", img_size[1])
    fs.write("camera_matrix", mtx)
    fs.write("distortion_coefficients", dist)
    fs.write("calibration_date", "2026-08-03")
    fs.release()


# -----------------------------------------------------------------------------
# 外参标定（手眼标定框架）
# -----------------------------------------------------------------------------
def calibrate_extrinsic(image_dir, mtx, dist, pattern_size, square_size):
    """
    读取 frame_###.png 和 euler_###.txt 进行手眼标定。
    注意：需要确认 euler 文件的格式（yaw/pitch/roll 顺序、单位、坐标系定义）。
    """
    objp = np.zeros((pattern_size[1] * pattern_size[0], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    images = sorted(glob.glob(os.path.join(image_dir, "frame_*.png")))
    if len(images) == 0:
        print(f"[错误] {image_dir} 中没有 frame_*.png")
        return

    print(f"找到 {len(images)} 张外参图片")

    # 收集数据
    R_target2cam = []   # 标定板到相机
    t_target2cam = []
    R_gripper2base = [] # 云台/相机到基座（来自 euler）
    t_gripper2base = []

    euler_dir = image_dir  # 假设 euler 文件与图片同目录

    for img_path in images:
        basename = os.path.basename(img_path)
        # 提取编号，例如 frame_001.png -> 001
        num = basename.replace('frame_', '').split('.')[0]
        euler_path = os.path.join(euler_dir, f"euler_{num}.txt")

        if not os.path.exists(euler_path):
            print(f"  [跳过] 缺少 {euler_path}")
            continue

        # 读取 euler
        with open(euler_path, 'r') as f:
            content = f.read().strip()
        try:
            angles = [float(x) for x in content.split()]
        except ValueError:
            print(f"  [跳过] {euler_path} 解析失败")
            continue

        print(f"  {basename} -> euler: {angles}")

        # TODO: 根据你们 euler 的实际定义（顺序、单位、坐标系）转换为旋转矩阵
        # 这里给一个示例：假设 angles = [pitch, yaw, roll]，单位度，ZYX 顺序
        # 你需要根据实际数据修改下面的转换逻辑
        if len(angles) >= 3:
            # 示例：假设为 [pitch, yaw, roll] 度
            pitch, yaw, roll = np.radians(angles[0]), np.radians(angles[1]), np.radians(angles[2])
            R_gripper = euler_to_rotation_matrix(pitch, yaw, roll, order='ZYX')
            R_gripper2base.append(R_gripper)
            t_gripper2base.append(np.zeros((3, 1)))  # 如果只有旋转没有平移，需确认
        else:
            continue

        # PnP 求标定板到相机的位姿
        img = cv2.imread(img_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if not ret:
            print(f"    [跳过] 未检测到角点")
            continue

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        ret, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist)
        if ret:
            R_target2cam.append(cv2.Rodrigues(rvec)[0])
            t_target2cam.append(tvec)

    if len(R_target2cam) < 3:
        print("[错误] 有效数据不足，无法执行手眼标定")
        print("提示：请确认 euler 文件格式，并修改 euler_to_rotation_matrix 中的转换逻辑")
        return

    # 手眼标定
    print(f"\n开始手眼标定（有效样本: {len(R_target2cam)}）...")
    try:
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=cv2.CALIB_HAND_EYE_TSAI
        )
        print(">>> 手眼标定结果（相机到云台）<<<")
        print(f"旋转矩阵:\n{R_cam2gripper}")
        print(f"平移向量:\n{t_cam2gripper.ravel()}")
        print("\n[注意] 请将上述结果填入 predict/tools.hpp 或 launch.cfg 的坐标变换部分")
    except Exception as e:
        print(f"[错误] 手眼标定失败: {e}")
        print("常见原因：euler 格式不匹配、坐标系定义错误、平移向量全为0")


def euler_to_rotation_matrix(pitch, yaw, roll, order='ZYX'):
    """
    欧拉角转旋转矩阵。根据实际定义修改。
    默认 ZYX 顺序（先绕 Z 偏航 yaw，再绕 Y 俯仰 pitch，再绕 X 横滚 roll）
    """
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0, 0, 1]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])
    return Rz @ Ry @ Rx


# -----------------------------------------------------------------------------
# 验证：重投影误差 + 去畸变
# -----------------------------------------------------------------------------
def verify_intrinsic(image_dir, mtx, dist, pattern_size, square_size, output_dir, show=False):
    objp = np.zeros((pattern_size[1] * pattern_size[0], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    images = sorted(glob.glob(os.path.join(image_dir, "*.png")))
    if not images:
        images = sorted(glob.glob(os.path.join(image_dir, "*.jpg")))

    print(f"\n验证图片数: {len(images)}")

    # 1. 逐图重投影误差
    errors = []
    for fname in images[:5]:  # 只看前5张做详细验证
        img = cv2.imread(fname)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if not ret:
            continue

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        ret, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist)
        if not ret:
            continue

        proj, _ = cv2.projectPoints(objp, rvec, tvec, mtx, dist)
        error = cv2.norm(corners, proj, cv2.NORM_L2) / len(proj)
        errors.append((os.path.basename(fname), error))

        # 画对比图：绿色=原始角点，红色=重投影点
        if show:
            for i in range(len(corners)):
                pt_orig = tuple(corners[i].ravel().astype(int))
                pt_proj = tuple(proj[i].ravel().astype(int))
                cv2.circle(img, pt_orig, 6, (0, 255, 0), 2)   # 绿：实际
                cv2.circle(img, pt_proj, 6, (0, 0, 255), 2)   # 红：投影
            cv2.imshow("Green=Actual, Red=Reprojected", cv2.resize(img, None, fx=0.5, fy=0.5))
            cv2.waitKey(500)

    if show:
        cv2.destroyAllWindows()

    print("\n>>> 逐图重投影误差 <<<")
    for name, err in errors:
        status = "✓" if err < 0.5 else ("△" if err < 1.0 else "✗")
        print(f"  {status} {name}: {err:.4f} px")

    # 2. 去畸变验证（选一张图做对比）
    if images:
        sample = cv2.imread(images[0])
        h, w = sample.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        undistorted = cv2.undistort(sample, mtx, dist, None, newcameramtx)

        # 保存对比图到输出目录
        compare_path = os.path.join(output_dir, "verify_undistort.png")
        # 左右拼接
        compare = np.hstack((sample, undistorted))
        cv2.imwrite(compare_path, compare)
        print(f"\n[已保存] 去畸变对比图: {compare_path}")
        print("  左=原始（注意边缘弯曲），右=去畸变（边缘应变直）")

    # 3. 给出结论
    avg_err = np.mean([e[1] for e in errors]) if errors else 999
    print("\n" + "=" * 60)
    print(">>> 验证结论 <<<")
    if avg_err < 0.3:
        print(f"  平均误差 {avg_err:.4f} px -> 【优秀】可直接用于 PnP 测距")
    elif avg_err < 0.5:
        print(f"  平均误差 {avg_err:.4f} px -> 【良好】可用")
    elif avg_err < 1.0:
        print(f"  平均误差 {avg_err:.4f} px -> 【一般】建议重新采集标定板图片")
    else:
        print(f"  平均误差 {avg_err:.4f} px -> 【较差】必须重新标定")
    print("=" * 60)


if __name__ == "__main__":
    main()
