#!/usr/bin/env python3
"""Validate translation candidates using manual images and nearest timestamped poses."""

import argparse
import glob
import os

import cv2
import numpy as np

from solve_hero_extrinsic import BOARD, SQUARE_M, K, D, imu_to_world, mean_rotation, angle_deg

R_C2I = np.array([
    [-0.998790276393, 0.012919578830, 0.047445424083],
    [-0.012490475786, -0.999878466675, 0.009329517372],
    [0.047560191318, 0.008725615314, 0.998830261776],
])
CANDIDATES = {
    "mechanical": np.array([0.0681, 0.1640, 0.0145]),
    "run0_free_fit": np.array([0.108154, 0.134477, 0.069071]),
    "length_constrained": np.array([0.10367, 0.12890, 0.06620]),
}


def fit_translation(r_i2w, p_board_c):
    a, b = [], []
    for g, pc in zip(r_i2w, p_board_c):
        a.append(np.hstack((g, -np.eye(3))))
        b.append(-g @ (R_C2I @ pc))
    a = np.vstack(a)
    x, _, _, _ = np.linalg.lstsq(a, np.hstack(b), rcond=None)
    return x[:3], float(np.linalg.cond(a))


def evaluate(label, r_i2w, p_board_c, translation):
    p = np.asarray([g @ (R_C2I @ pc + translation) for g, pc in zip(r_i2w, p_board_c)])
    center = np.mean(p, axis=0)
    errors = np.linalg.norm(p - center, axis=1) * 1000.
    std = np.std(p, axis=0) * 1000.
    print(f'{label}: T_mm={np.array2string(translation*1000., precision=2)} '
          f'axis_std_mm={np.array2string(std, precision=2)} '
          f'rms={np.sqrt(np.mean(errors**2)):.3f} median={np.median(errors):.3f} '
          f'max={np.max(errors):.3f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--images', default='calib_frames')
    ap.add_argument('--poses', default='hero_extrinsic_validation_20260905_run2')
    ap.add_argument('--max-time-delta', type=float, default=0.8)
    args = ap.parse_args()

    pose_files = glob.glob(os.path.join(args.poses, 'euler_*.txt'))
    pose_times = np.asarray([os.path.getmtime(p) for p in pose_files])
    if not pose_files:
        raise SystemExit('no timestamped pose files')
    obj = np.zeros((BOARD[0] * BOARD[1], 3), np.float32)
    obj[:, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2)
    obj *= SQUARE_M
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, .001)
    samples = []
    rejects = {'time': 0, 'image': 0, 'corners': 0, 'pnp': 0}
    time_deltas = []
    for image_path in sorted(glob.glob(os.path.join(args.images, 'raw_*.png'))):
        image_time = os.path.getmtime(image_path)
        nearest = int(np.argmin(np.abs(pose_times - image_time)))
        dt = float(abs(pose_times[nearest] - image_time))
        if dt > args.max_time_delta:
            rejects['time'] += 1
            continue
        gray = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            rejects['image'] += 1
            continue
        small = cv2.resize(gray, None, fx=.5, fy=.5, interpolation=cv2.INTER_AREA)
        ok, corners = cv2.findChessboardCorners(
            small, BOARD, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            rejects['corners'] += 1
            continue
        corners *= 2.
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        ok, rv, tv = cv2.solvePnP(obj, corners, K, D)
        if not ok:
            rejects['pnp'] += 1
            continue
        projected, _ = cv2.projectPoints(obj, rv, tv, K, D)
        reproj = float(np.sqrt(np.mean(np.sum((projected-corners)**2, axis=2))))
        ypr = np.loadtxt(pose_files[nearest], dtype=float).reshape(-1)[:3]
        samples.append((reproj, ypr, imu_to_world(ypr), cv2.Rodrigues(rv)[0], tv.reshape(3), image_path, dt))
        time_deltas.append(dt)

    # One sharpest image per 1-degree attitude bin prevents long-held poses dominating.
    bins = {}
    for sample in samples:
        key = tuple(np.rint(sample[1]).astype(int))
        if key not in bins or sample[0] < bins[key][0]:
            bins[key] = sample
    samples = list(bins.values())
    print(f'manual_images={len(glob.glob(os.path.join(args.images,"raw_*.png")))} '
          f'matched_valid={sum(rejects.values()) + len(samples)} unique_pose_bins={len(samples)} '
          f'rejects={rejects}')
    if time_deltas:
        print(f'timestamp_delta_ms median={np.median(time_deltas)*1000.:.1f} '
              f'p95={np.percentile(time_deltas,95)*1000.:.1f} max={np.max(time_deltas)*1000.:.1f}')
    if len(samples) < 8:
        raise SystemExit('not enough valid unique manual/pose pairs')
    r_i2w = [s[2] for s in samples]
    r_board_c = [s[3] for s in samples]
    p_board_c = [s[4] for s in samples]
    board_world_r = np.asarray([g @ R_C2I @ r for g, r in zip(r_i2w, r_board_c)])
    center_r = mean_rotation(board_world_r)
    rot_err = np.asarray([angle_deg(center_r.T @ r) for r in board_world_r])
    print(f'rotation_check_deg rms={np.sqrt(np.mean(rot_err**2)):.4f} '
          f'median={np.median(rot_err):.4f} max={np.max(rot_err):.4f}')
    for label, translation in CANDIDATES.items():
        evaluate(label, r_i2w, p_board_c, translation)
    fitted, condition = fit_translation(r_i2w, p_board_c)
    evaluate('manual_free_fit', r_i2w, p_board_c, fitted)
    print(f'manual_free_fit_condition={condition:.3f}')


if __name__ == '__main__':
    main()
