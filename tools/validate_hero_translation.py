#!/usr/bin/env python3
"""Validate/estimate camera-to-IMU translation from a fixed-board rotation run."""

import argparse
import itertools
import numpy as np

from solve_hero_extrinsic import load, mean_rotation, angle_deg

R_C2I = np.array([
    [-0.998790276393, 0.012919578830, 0.047445424083],
    [-0.012490475786, -0.999878466675, 0.009329517372],
    [0.047560191318, 0.008725615314, 0.998830261776],
])
T_MEASURED = np.array([0.0681, 0.1640, 0.0145])


def fit_translation(r_i2w, p_board_c, return_condition=False):
    # For a stationary board: P_board_w = R_i2w (R_c2i p_board_c + T_c2i).
    # Unknowns are T_c2i and the single stationary P_board_w.
    a, b = [], []
    for g, pc in zip(r_i2w, p_board_c):
        a.append(np.hstack((g, -np.eye(3))))
        b.append(-g @ (R_C2I @ pc.reshape(3)))
    a = np.vstack(a)
    x, _, _, _ = np.linalg.lstsq(a, np.hstack(b), rcond=None)
    result = (x[:3], x[3:])
    if return_condition:
        return (*result, float(np.linalg.cond(a)))
    return result


def world_positions(r_i2w, p_board_c, translation):
    return np.asarray([
        g @ (R_C2I @ pc.reshape(3) + translation)
        for g, pc in zip(r_i2w, p_board_c)
    ])


def report(label, positions):
    center = np.mean(positions, axis=0)
    errors = np.linalg.norm(positions - center, axis=1)
    axis_std = np.std(positions, axis=0)
    print(f'{label}_center_m={np.array2string(center, precision=6)}')
    print(f'{label}_axis_std_mm={np.array2string(axis_std * 1000., precision=3)}')
    print(f'{label}_position_error_mm rms={np.sqrt(np.mean(errors**2))*1000.:.3f} '
          f'median={np.median(errors)*1000.:.3f} max={np.max(errors)*1000.:.3f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('directory', nargs='?', default='hero_extrinsic_data_20260904_run2')
    args = ap.parse_args()
    rgb, rtc, ttc, used = load(args.directory)
    if len(used) < 8:
        raise SystemExit(f'not enough valid pairs: {len(used)}')

    # Apply the same rotation-consistency rejection used for the rotation result.
    board_world_r = np.asarray([g @ R_C2I @ t for g, t in zip(rgb, rtc)])
    center_r = mean_rotation(board_world_r)
    rot_err = np.asarray([angle_deg(center_r.T @ r) for r in board_world_r])
    med = float(np.median(rot_err))
    mad = float(np.median(np.abs(rot_err - med)))
    threshold = max(1.0, med + 3.0 * 1.4826 * mad)
    keep = np.flatnonzero(rot_err <= threshold)
    g = [rgb[i] for i in keep]
    pc = [ttc[i] for i in keep]
    print(f'valid_pairs={len(used)} kept={len(keep)} rotation_threshold_deg={threshold:.4f}')

    fitted, board_world, condition = fit_translation(g, pc, return_condition=True)
    print(f'T_measured_m={np.array2string(T_MEASURED, precision=6)}')
    print(f'T_fitted_m={np.array2string(fitted, precision=6)}')
    print(f'linear_system_condition={condition:.3f}')
    print(f'T_delta_mm={np.array2string((T_MEASURED-fitted)*1000., precision=3)}')
    report('measured', world_positions(g, pc, T_MEASURED))
    report('fitted', world_positions(g, pc, fitted))

    # Diagnose a possible axis/sign interpretation error in the three measured magnitudes.
    candidates = []
    magnitudes = np.abs(T_MEASURED)
    for perm in itertools.permutations(magnitudes):
        for signs in itertools.product((-1., 1.), repeat=3):
            t = np.asarray(perm) * np.asarray(signs)
            positions = world_positions(g, pc, t)
            errors = np.linalg.norm(positions - np.mean(positions, axis=0), axis=1)
            candidates.append((float(np.sqrt(np.mean(errors**2))), t))
    print('best_measured_axis_sign_candidates:')
    for rms, t in sorted(candidates, key=lambda item: item[0])[:5]:
        print(f'  T_m={np.array2string(t, precision=4)} position_rms_mm={rms*1000.:.3f}')

    rng = np.random.default_rng(20260905)
    subset_size = max(12, int(len(g) * 0.8))
    subset_t = []
    for _ in range(200):
        chosen = rng.choice(len(g), size=subset_size, replace=False)
        t, _ = fit_translation([g[i] for i in chosen], [pc[i] for i in chosen])
        subset_t.append(t)
    subset_t = np.asarray(subset_t)
    delta = np.linalg.norm(subset_t - fitted, axis=1) * 1000.
    print(f'T_subset_std_mm={np.array2string(np.std(subset_t, axis=0)*1000., precision=3)}')
    print(f'T_subset_delta_mm median={np.median(delta):.3f} '
          f'p95={np.percentile(delta,95):.3f} max={np.max(delta):.3f}')


if __name__ == '__main__':
    main()
