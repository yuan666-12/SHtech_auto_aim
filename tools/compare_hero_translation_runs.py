#!/usr/bin/env python3
"""Cross-validate Hero camera-to-IMU translation on independent fixed-board runs."""

import argparse
import numpy as np

from solve_hero_extrinsic import load, angle_deg, BOARD, SQUARE_M
from validate_hero_translation import R_C2I, T_MEASURED, fit_translation, world_positions


def prepare(directory):
    rgb, rtc, ttc, used = load(directory)
    # tvec is the first chessboard corner. Convert it to the geometric center;
    # the center is invariant when OpenCV reverses the corner order by 180 deg.
    center_obj = np.array([(BOARD[0] - 1) * SQUARE_M / 2.,
                           (BOARD[1] - 1) * SQUARE_M / 2., 0.])
    ttc = [r @ center_obj.reshape(3, 1) + t for r, t in zip(rtc, ttc)]
    # Filter by the board normal only. A 180-degree in-plane ordering flip leaves
    # the normal unchanged, unlike a full board-rotation residual.
    normals = np.asarray([(g @ R_C2I @ r)[:, 2] for g, r in zip(rgb, rtc)])
    center_n = np.mean(normals, axis=0)
    center_n /= np.linalg.norm(center_n)
    rot_err = np.degrees(np.arccos(np.clip(normals @ center_n, -1., 1.)))
    med = float(np.median(rot_err))
    mad = float(np.median(np.abs(rot_err - med)))
    threshold = max(1.0, med + 3.0 * 1.4826 * mad)
    keep = np.flatnonzero(rot_err <= threshold)
    return ([rgb[i] for i in keep], [ttc[i] for i in keep], len(used), len(keep),
            float(np.sqrt(np.mean(rot_err[keep] ** 2))), threshold)


def position_score(g, pc, translation):
    positions = world_positions(g, pc, translation)
    center = np.mean(positions, axis=0)
    errors = np.linalg.norm(positions - center, axis=1) * 1000.
    return (float(np.sqrt(np.mean(errors ** 2))), float(np.median(errors)),
            float(np.percentile(errors, 95)), float(np.max(errors)))


def fit_combined(runs):
    # One shared T, but an independent stationary board-world position for every run.
    rows, rhs = [], []
    run_count = len(runs)
    for run_index, (g_list, pc_list) in enumerate(runs):
        for g, pc in zip(g_list, pc_list):
            row = np.zeros((3, 3 + 3 * run_count))
            row[:, :3] = g
            row[:, 3 + 3 * run_index:6 + 3 * run_index] = -np.eye(3)
            rows.append(row)
            rhs.append(-g @ (R_C2I @ pc.reshape(3)))
    a = np.vstack(rows)
    x, _, _, _ = np.linalg.lstsq(a, np.hstack(rhs), rcond=None)
    return x[:3], float(np.linalg.cond(a))


def constrained_length(t):
    return t * (np.linalg.norm(T_MEASURED) / np.linalg.norm(t))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run1')
    ap.add_argument('run2')
    args = ap.parse_args()
    data = [prepare(args.run1), prepare(args.run2)]
    for i, item in enumerate(data, 1):
        print(f'run{i}: valid={item[2]} kept={item[3]} rotation_rms_deg={item[4]:.4f} '
              f'threshold_deg={item[5]:.4f}')

    fit1, _, cond1 = fit_translation(data[0][0], data[0][1], return_condition=True)
    fit2, _, cond2 = fit_translation(data[1][0], data[1][1], return_condition=True)
    fit_both, cond_both = fit_combined([(data[0][0], data[0][1]), (data[1][0], data[1][1])])
    old_fit = np.array([0.108154, 0.134477, 0.069071])
    candidates = {
        'mechanical': T_MEASURED,
        'old_fit': old_fit,
        'old_fit_length_constrained': constrained_length(old_fit),
        'fit_run1': fit1,
        'fit_run2': fit2,
        'fit_both': fit_both,
        'fit_both_length_constrained': constrained_length(fit_both),
    }
    print(f'condition_numbers run1={cond1:.3f} run2={cond2:.3f} combined={cond_both:.3f}')
    print(f'fit_run_delta_mm={np.linalg.norm(fit1-fit2)*1000.:.3f}')
    for name, t in candidates.items():
        print(f'{name}: T_mm={np.array2string(t*1000., precision=3)} norm_mm={np.linalg.norm(t)*1000.:.3f}')
        for i, item in enumerate(data, 1):
            rms, med, p95, maximum = position_score(item[0], item[1], t)
            print(f'  run{i}: rms={rms:.3f} median={med:.3f} p95={p95:.3f} max={maximum:.3f} mm')


if __name__ == '__main__':
    main()
