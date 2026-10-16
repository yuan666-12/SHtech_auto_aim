#!/usr/bin/env python3
"""Fit camera rotation and fixed-rotation-reference translation from fixed-board samples.

The translation assumes a stationary rotation reference point. It is not automatically
the translation to the true IMU origin, which may move during yaw/pitch rotation.
Manual P-key samples bypass the legacy adjacent-pose test; the operator must stop first.
No running camera configuration is modified.
"""
import argparse
import json
import os
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

from solve_hero_extrinsic import load, BOARD, SQUARE_M, angle_deg


def wrapped_delta(a, b):
    return (a - b + 180.) % 360. - 180.


def prepare(directory, sample_mode="manual"):
    g, rtc, ttc, used = load(directory)
    if len(used) < 3:
        raise ValueError(f"{directory}: fewer than 3 detectable paired samples")
    center_obj = np.array([(BOARD[0] - 1) * SQUARE_M / 2.,
                           (BOARD[1] - 1) * SQUARE_M / 2., 0.])
    pc = np.asarray([(r @ center_obj.reshape(3, 1) + t).reshape(3)
                     for r, t in zip(rtc, ttc)])
    keep = np.arange(len(used))
    if sample_mode == "adjacent":
        ypr = np.asarray([np.loadtxt(os.path.join(directory, f"euler_{i}.txt")).reshape(-1)[:3]
                          for i in used])
        motion = np.full(len(used), np.inf)
        steps = np.linalg.norm(wrapped_delta(ypr[1:], ypr[:-1]), axis=1)
        motion[1:] = np.minimum(motion[1:], steps)
        motion[:-1] = np.minimum(motion[:-1], steps)
        keep = np.flatnonzero(motion < 0.35)
    if len(keep) < 3:
        raise ValueError(f"{directory}: only {len(keep)} samples kept in {sample_mode} mode")
    ids = [int(used[i]) for i in keep]
    return np.asarray([g[i] for i in keep]), pc[keep], {
        "directory": str(Path(directory).resolve()), "detectable_ids": list(map(int, used)),
        "used_ids": ids, "sample_mode": sample_mode}


def linear_translation(runs, rotation):
    # Unknown translation and one fixed board center per run, all in metres.
    count = len(runs)
    rows, rhs = [], []
    for j, (gs, pcs) in enumerate(runs):
        for g, pc in zip(gs, pcs):
            row = np.zeros((3, 3 + 3 * count))
            row[:, :3] = g
            row[:, 3+3*j:6+3*j] = -np.eye(3)
            rows.append(row)
            rhs.append(-g @ (rotation @ pc))
    matrix = np.vstack(rows)
    fitted, _, rank, singular = np.linalg.lstsq(matrix, np.hstack(rhs), rcond=None)
    return fitted, {"rank": int(rank), "columns": int(matrix.shape[1]),
                    "condition": float(np.linalg.cond(matrix)),
                    "singular_values": singular.tolist()}


def optimize(runs, r_init, t_init=None, length_m=None, fixed_rotation=False):
    linear, condition = linear_translation(runs, r_init)
    t0 = linear[:3] if t_init is None else np.asarray(t_init, dtype=float)
    centers0 = [np.mean([g @ (r_init @ p + t0) for g, p in zip(gs, pcs)], axis=0)
                for gs, pcs in runs]
    x0 = np.hstack((t0, *centers0)) if fixed_rotation else np.hstack((np.zeros(3), t0, *centers0))

    def unpack(x):
        if fixed_rotation:
            return r_init, x[:3], x[3:]
        return r_init @ cv2.Rodrigues(x[:3])[0], x[3:6], x[6:]

    def residual(x):
        r, t, centers = unpack(x)
        chunks = []
        for j, (gs, pcs) in enumerate(runs):
            center = centers[3*j:3*j+3]
            chunks.append(np.asarray([g @ (r @ p + t) - center
                                      for g, p in zip(gs, pcs)]).ravel())
        if length_m is not None:
            # Explicit constraint on the fixed-reference-to-camera length only.
            chunks.append(np.array([(np.linalg.norm(t) - length_m) / 0.02]))
        return np.hstack(chunks)

    result = least_squares(residual, x0, loss="huber", f_scale=0.008,
                           max_nfev=300, verbose=0)
    rotation, translation, centers = unpack(result.x)
    return {"R": rotation.tolist(), "T_fixed_reference_to_camera_m": translation.tolist(),
            "T_mm": (translation * 1000.).tolist(),
            "norm_mm": float(np.linalg.norm(translation) * 1000.),
            "board_centers_world_m": centers.reshape(-1, 3).tolist(),
            "R_delta_from_R_only_deg": float(angle_deg(r_init.T @ rotation)),
            "cost": float(result.cost), "optimality": float(result.optimality),
            "success": bool(result.success), "nfev": int(result.nfev),
            "message": result.message, "linear_fixed_R_condition": condition,
            "length_constraint_mm": None if length_m is None else length_m * 1000.}


def score(run, rotation, translation):
    gs, pcs = run
    pos = np.asarray([g @ (rotation @ p + translation) for g, p in zip(gs, pcs)])
    center = np.median(pos, axis=0)
    error = np.linalg.norm(pos - center, axis=1) * 1000.
    return {"rms_mm": float(np.sqrt(np.mean(error**2))), "median_mm": float(np.median(error)),
            "p95_mm": float(np.percentile(error, 95)), "max_mm": float(np.max(error))}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("directories", nargs="+")
    ap.add_argument("--r-init-file", required=True, help="3x3 camera-to-runtime-IMU-axis rotation, text")
    ap.add_argument("--t-init-mm", nargs=3, type=float, help="optional FIXED-reference translation seed")
    ap.add_argument("--sample-mode", choices=["manual", "adjacent"], default="manual")
    ap.add_argument("--length-mm", type=float, help="optional FIXED-reference-to-camera length, not true IMU length")
    ap.add_argument("--output-json", help="save all candidates; existing output is refused")
    args = ap.parse_args()
    r_init = np.loadtxt(args.r_init_file)
    if r_init.shape != (3, 3) or not np.isfinite(r_init).all() or \
       not np.allclose(r_init.T @ r_init, np.eye(3), atol=1e-6) or \
       not np.isclose(np.linalg.det(r_init), 1., atol=1e-6):
        raise ValueError("--r-init-file must contain a finite proper 3x3 rotation")
    if args.length_mm is not None and args.length_mm <= 0:
        raise ValueError("--length-mm must be positive")
    if args.output_json and Path(args.output_json).exists():
        raise FileExistsError(args.output_json)
    prepared = [prepare(directory, args.sample_mode) for directory in args.directories]
    runs = [(x[0], x[1]) for x in prepared]
    t_init = None if args.t_init_mm is None else np.array(args.t_init_mm) / 1000.
    metadata = [x[2] for x in prepared]
    for i, meta in enumerate(metadata, 1):
        print(f"run{i}: detectable={len(meta['detectable_ids'])} used={len(meta['used_ids'])} "
              f"mode={args.sample_mode} ids={meta['used_ids']}")
    fits = {}
    for fixed, name in [(False, "fit_combined"), (True, "fixed_R_fit_combined")]:
        fits[name] = optimize(runs, r_init, t_init, fixed_rotation=fixed)
        if args.length_mm is not None:
            fits[name+"_length_constrained"] = optimize(
                runs, r_init, t_init, args.length_mm / 1000., fixed_rotation=fixed)
    for name, fit in fits.items():
        r = np.array(fit["R"])
        t = np.array(fit["T_fixed_reference_to_camera_m"])
        fit["scores"] = [score(run, r, t) for run in runs]
        print(f"{name}: success={fit['success']} cost={fit['cost']:.6f} "
              f"optimality={fit['optimality']:.3e} nfev={fit['nfev']}")
        print(f"  R=\n{np.array2string(r, precision=12, separator=', ')}")
        print(f"  T_fixed_reference_mm={np.array2string(t*1000., precision=6, separator=', ')} "
              f"norm_mm={fit['norm_mm']:.6f}")
        print(f"  R_delta_from_R_only_deg={fit['R_delta_from_R_only_deg']:.6f}")
        for i, stats in enumerate(fit["scores"], 1):
            print(f"  run{i}: "+ " ".join(f"{key}={value:.6f}" for key,value in stats.items()))
        print("  fixed_R_translation_condition=", fit["linear_fixed_R_condition"]["condition"])
    result = {"translation_origin": "fixed rotation reference point; NOT automatically true IMU origin",
              "R_init": r_init.tolist(), "runs": metadata, "fits": fits,
              "length_constraint_mm": args.length_mm,
              "running_camera_configuration_modified": False}
    if args.output_json:
        with open(args.output_json, "x") as target:
            json.dump(result, target, ensure_ascii=False, indent=2)
        print("RESULT_JSON="+str(Path(args.output_json).resolve()))


if __name__ == "__main__":
    main()
