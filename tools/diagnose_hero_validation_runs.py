#!/usr/bin/env python3
"""Diagnose motion contamination and rotation repeatability in validation runs."""

import argparse
import os
import cv2
import numpy as np

from solve_hero_extrinsic import load, score, angle_deg
from validate_hero_translation import R_C2I, T_MEASURED


def wrapped_delta(a, b):
    return (a - b + 180.) % 360. - 180.


def diagnose(directory):
    rgb, rtc, ttc, used = load(directory)
    ypr = np.asarray([
        np.loadtxt(os.path.join(directory, f'euler_{i}.txt')).reshape(-1)[:3]
        for i in used
    ])
    neighbor_motion = np.full(len(used), np.inf)
    if len(used) > 1:
        steps = np.linalg.norm(wrapped_delta(ypr[1:], ypr[:-1]), axis=1)
        neighbor_motion[1:] = np.minimum(neighbor_motion[1:], steps)
        neighbor_motion[:-1] = np.minimum(neighbor_motion[:-1], steps)
    stable = np.flatnonzero(neighbor_motion < 0.35)
    if len(stable) < 12:
        stable = np.flatnonzero(neighbor_motion < 1.0)
    g = [rgb[i] for i in stable]
    t = [rtc[i] for i in stable]
    zeros = [np.zeros((3, 1)) for _ in stable]
    r_fit, _ = cv2.calibrateHandEye(g, zeros, t, [ttc[i] for i in stable],
                                    method=cv2.CALIB_HAND_EYE_HORAUD)
    fixed_score = score(g, t, R_C2I)
    fit_score = score(g, t, r_fit)
    print(f'{directory}: valid={len(used)} stable={len(stable)}')
    print(f'  ypr_min={np.array2string(ypr.min(0), precision=2)}')
    print(f'  ypr_max={np.array2string(ypr.max(0), precision=2)}')
    print(f'  existing_R_rms={fixed_score[0]:.4f}deg fit_R_rms={fit_score[0]:.4f}deg '
          f'fit_vs_existing={angle_deg(R_C2I.T @ r_fit):.4f}deg')
    print(f'  R_fit=\n{np.array2string(r_fit, precision=10)}')

    rows, rhs = [], []
    for gi, pci in zip(g, [ttc[i] for i in stable]):
        rows.append(np.hstack((gi, -np.eye(3))))
        rhs.append(-gi @ (r_fit @ pci.reshape(3)))
    a = np.vstack(rows)
    x, _, _, _ = np.linalg.lstsq(a, np.hstack(rhs), rcond=None)
    t_fit = x[:3]
    t_constrained = t_fit * (np.linalg.norm(T_MEASURED) / np.linalg.norm(t_fit))
    print(f'  T_fit_mm={np.array2string(t_fit*1000., precision=3)} '
          f'norm_mm={np.linalg.norm(t_fit)*1000.:.3f} condition={np.linalg.cond(a):.3f}')
    print(f'  T_length_constrained_mm={np.array2string(t_constrained*1000., precision=3)}')
    for name, translation in [('mechanical', T_MEASURED), ('fit', t_fit),
                              ('length_constrained', t_constrained)]:
        positions = np.asarray([gi @ (r_fit @ pci.reshape(3) + translation)
                                for gi, pci in zip(g, [ttc[i] for i in stable])])
        errors = np.linalg.norm(positions - np.mean(positions, axis=0), axis=1) * 1000.
        print(f'    {name}: position_rms={np.sqrt(np.mean(errors**2)):.3f} '
              f'median={np.median(errors):.3f} p95={np.percentile(errors,95):.3f} mm')
    return r_fit


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('directories', nargs='+')
    args = ap.parse_args()
    rotations = [diagnose(d) for d in args.directories]
    if len(rotations) == 2:
        print(f'run_rotation_delta={angle_deg(rotations[0].T @ rotations[1]):.4f}deg')


if __name__ == '__main__':
    main()
