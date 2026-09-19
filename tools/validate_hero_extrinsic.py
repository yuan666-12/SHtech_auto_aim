#!/usr/bin/env python3
"""Validate paired Hero hand-eye images and yaw/pitch/roll samples."""

import argparse
import glob
import os

import cv2
import numpy as np


BOARD_SIZE = (11, 8)
SQUARE_SIZE_M = 0.015
K = np.array([
    [1479.0807932250757, 0.0, 730.6996848643015],
    [0.0, 1478.1439161714959, 548.1801460793905],
    [0.0, 0.0, 1.0],
], dtype=np.float64)
D = np.array([
    -0.13372672103392352,
    0.21795172844506383,
    -0.00045119642764971177,
    0.0008142750903548959,
    -0.11184162108320281,
], dtype=np.float64)


def sample_id(path):
    return int(os.path.splitext(os.path.basename(path))[0].split("_", 1)[1])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", nargs="?", default="hero_extrinsic_data_20260904_v2")
    args = parser.parse_args()
    directory = args.directory
    images = {sample_id(p): p for p in glob.glob(os.path.join(directory, "frame_*.png"))}
    poses = {sample_id(p): p for p in glob.glob(os.path.join(directory, "euler_*.txt"))}
    ids = sorted(images.keys() & poses.keys())

    obj = np.zeros((BOARD_SIZE[0] * BOARD_SIZE[1], 3), np.float32)
    obj[:, :2] = np.mgrid[0:BOARD_SIZE[0], 0:BOARD_SIZE[1]].T.reshape(-1, 2)
    obj *= SQUARE_SIZE_M
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)

    valid = []
    angles = []
    rejected = []
    for sid in ids:
        try:
            angle = np.loadtxt(poses[sid], dtype=float).reshape(-1)[:3]
        except Exception as exc:
            rejected.append((sid, f"bad_pose:{exc}"))
            continue
        if angle.size != 3 or not np.isfinite(angle).all():
            rejected.append((sid, "bad_pose_values"))
            continue

        gray = cv2.imread(images[sid], cv2.IMREAD_GRAYSCALE)
        if gray is None:
            rejected.append((sid, "bad_image"))
            continue
        blur = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        small = cv2.resize(gray, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
        found, corners = cv2.findChessboardCorners(
            small, BOARD_SIZE,
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if not found:
            rejected.append((sid, f"no_corners blur={blur:.1f}"))
            continue
        corners *= 2.0
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        ok, rvec, tvec = cv2.solvePnP(obj, corners, K, D)
        if not ok:
            rejected.append((sid, "solvepnp_failed"))
            continue
        projected, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
        rms = float(np.sqrt(np.mean(np.sum((corners - projected) ** 2, axis=2))))
        if rms > 1.0:
            rejected.append((sid, f"reprojection_rms={rms:.3f}"))
            continue
        valid.append((sid, angle, rms, blur))
        angles.append(angle)

    print(f"images={len(images)} poses={len(poses)} pairs={len(ids)}")
    print(f"valid={len(valid)} rejected={len(rejected)}")
    if valid:
        a = np.asarray(angles)
        print("ranges_deg: " + " ".join(
            f"{name}={a[:, i].min():.2f}..{a[:, i].max():.2f}"
            for i, name in enumerate(("yaw", "pitch", "roll"))
        ))
        print(f"reprojection_rms_px: mean={np.mean([v[2] for v in valid]):.3f} "
              f"max={np.max([v[2] for v in valid]):.3f}")
        print("valid_ids=" + ",".join(str(v[0]) for v in valid))
    if rejected:
        print("rejected=" + ", ".join(f"{sid}:{reason}" for sid, reason in rejected[:30]))


if __name__ == "__main__":
    main()
