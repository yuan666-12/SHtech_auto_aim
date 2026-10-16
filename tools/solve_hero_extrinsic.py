#!/usr/bin/env python3
"""Solve and cross-check Hero camera-to-IMU rotation from hand-eye pairs."""

import argparse
import glob
import os

import cv2
import numpy as np

BOARD = (9, 7)
SQUARE_M = 0.020
K = np.array([[1479.0807932250757, 0., 730.6996848643015],
              [0., 1478.1439161714959, 548.1801460793905],
              [0., 0., 1.]], np.float64)
D = np.array([-0.13372672103392352, 0.21795172844506383,
              -0.00045119642764971177, 0.0008142750903548959,
              -0.11184162108320281], np.float64)
R_INITIAL = np.array([[-1., 0., 0.], [0., 0., 1.], [0., 1., 0.]])


def rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def imu_to_world(yaw_pitch_roll):
    yaw, pitch, roll = np.radians(yaw_pitch_roll)
    q = rz(yaw) @ rx(pitch) @ ry(roll)
    return q @ R_INITIAL.T


def mean_rotation(rotations):
    u, _, vt = np.linalg.svd(np.sum(rotations, axis=0))
    r = u @ vt
    if np.linalg.det(r) < 0:
        u[:, -1] *= -1
        r = u @ vt
    return r


def angle_deg(r):
    return np.degrees(np.arccos(np.clip((np.trace(r) - 1.) / 2., -1., 1.)))


def load(directory):
    def sid(p): return int(os.path.splitext(os.path.basename(p))[0].split('_', 1)[1])
    images = {sid(p): p for p in glob.glob(os.path.join(directory, 'frame_*.png'))}
    poses = {sid(p): p for p in glob.glob(os.path.join(directory, 'euler_*.txt'))}
    obj = np.zeros((BOARD[0] * BOARD[1], 3), np.float32)
    obj[:, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2)
    obj *= SQUARE_M
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, .001)
    rgb, rtc, ttc, used = [], [], [], []
    for i in sorted(images.keys() & poses.keys()):
        gray = cv2.imread(images[i], cv2.IMREAD_GRAYSCALE)
        if gray is None: continue
        small = cv2.resize(gray, None, fx=.5, fy=.5, interpolation=cv2.INTER_AREA)
        ok, corners = cv2.findChessboardCorners(
            small, BOARD, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok: continue
        corners *= 2.
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        ok, rv, tv = cv2.solvePnP(obj, corners, K, D)
        if not ok: continue
        ypr = np.loadtxt(poses[i], dtype=float).reshape(-1)[:3]
        rgb.append(imu_to_world(ypr))
        rtc.append(cv2.Rodrigues(rv)[0])
        ttc.append(tv)
        used.append(i)
    return rgb, rtc, ttc, used


def score(rgb, rtc, rcg):
    # target->base = gripper->base * camera->gripper * target->camera
    rtb = np.asarray([g @ rcg @ t for g, t in zip(rgb, rtc)])
    center = mean_rotation(rtb)
    errors = np.asarray([angle_deg(center.T @ r) for r in rtb])
    return float(np.sqrt(np.mean(errors ** 2))), float(np.median(errors)), float(np.max(errors))


def errors_for(rgb, rtc, rcg):
    rtb = np.asarray([g @ rcg @ t for g, t in zip(rgb, rtc)])
    center = mean_rotation(rtb)
    return np.asarray([angle_deg(center.T @ r) for r in rtb])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('directory', nargs='?', default='hero_extrinsic_data_20260904_run2')
    args = ap.parse_args()
    rgb, rtc, ttc, used = load(args.directory)
    if len(used) < 8:
        raise SystemExit(f'not enough valid pairs: {len(used)}')
    zeros = [np.zeros((3, 1)) for _ in used]
    methods = {
        'TSAI': cv2.CALIB_HAND_EYE_TSAI,
        'PARK': cv2.CALIB_HAND_EYE_PARK,
        'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
        'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF,
        'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    print(f'valid_pairs={len(used)} ids={used}')
    results = []
    for name, method in methods.items():
        try:
            rcg, _ = cv2.calibrateHandEye(rgb, zeros, rtc, ttc, method=method)
            rms, median, maximum = score(rgb, rtc, rcg)
            results.append((rms, name, rcg, median, maximum))
            print(f'{name}: world_board_rotation_rms={rms:.4f}deg '
                  f'median={median:.4f}deg max={maximum:.4f}deg')
            print(np.array2string(rcg, precision=10, suppress_small=False))
        except cv2.error as exc:
            print(f'{name}: FAILED {exc}')
    if not results:
        raise SystemExit('all hand-eye methods failed')
    results.sort(key=lambda x: x[0])
    best = results[0]
    print(f'BEST={best[1]} RMS={best[0]:.4f}deg')
    print('R_c2i_candidate=')
    print(np.array2string(best[2], precision=12, separator=', '))

    # Robust refinement: remove only large world-board rotation inconsistencies.
    err = errors_for(rgb, rtc, best[2])
    med = float(np.median(err))
    mad = float(np.median(np.abs(err - med)))
    threshold = max(1.0, med + 3.0 * 1.4826 * mad)
    keep = np.flatnonzero(err <= threshold)
    rr = [rgb[i] for i in keep]
    rt = [rtc[i] for i in keep]
    rz0 = [np.zeros((3, 1)) for _ in keep]
    robust_r, _ = cv2.calibrateHandEye(
        rr, rz0, rt, [ttc[i] for i in keep], method=cv2.CALIB_HAND_EYE_HORAUD)
    robust_score = score(rr, rt, robust_r)
    print(f'ROBUST_HORAUD kept={len(keep)}/{len(used)} threshold={threshold:.4f}deg '
          f'RMS={robust_score[0]:.4f}deg median={robust_score[1]:.4f}deg '
          f'max={robust_score[2]:.4f}deg')
    print('R_c2i_robust=')
    print(np.array2string(robust_r, precision=12, separator=', '))

    # Deterministic subset stability check.
    rng = np.random.default_rng(20260904)
    subset_delta = []
    subset_size = max(12, int(len(keep) * 0.8))
    for _ in range(40):
        chosen = rng.choice(keep, size=subset_size, replace=False)
        try:
            r_sub, _ = cv2.calibrateHandEye(
                [rgb[i] for i in chosen], [np.zeros((3, 1)) for _ in chosen],
                [rtc[i] for i in chosen], [ttc[i] for i in chosen],
                method=cv2.CALIB_HAND_EYE_HORAUD)
            subset_delta.append(angle_deg(robust_r.T @ r_sub))
        except cv2.error:
            pass
    if subset_delta:
        print(f'subset_stability_deg median={np.median(subset_delta):.4f} '
              f'p95={np.percentile(subset_delta, 95):.4f} max={np.max(subset_delta):.4f}')
    print('T_c2i=NOT_OBSERVABLE_FROM_ROTATION_ONLY_DATA; use physical/CAD measurement')


if __name__ == '__main__':
    main()
