#!/usr/bin/env python3
"""Robustly fit R_c2i/T_c2i from stationary chessboard centers across runs."""

import argparse
import os
import cv2
import numpy as np
from scipy.optimize import least_squares

from solve_hero_extrinsic import load, BOARD, SQUARE_M
from validate_hero_translation import R_C2I, T_MEASURED


def wrapped_delta(a, b):
    return (a - b + 180.) % 360. - 180.


def prepare(directory):
    g, rtc, ttc, used = load(directory)
    center_obj = np.array([(BOARD[0] - 1) * SQUARE_M / 2.,
                           (BOARD[1] - 1) * SQUARE_M / 2., 0.])
    pc = np.asarray([(r @ center_obj.reshape(3, 1) + t).reshape(3)
                     for r, t in zip(rtc, ttc)])
    ypr = np.asarray([np.loadtxt(os.path.join(directory, f'euler_{i}.txt')).reshape(-1)[:3]
                      for i in used])
    motion = np.full(len(used), np.inf)
    if len(used) > 1:
        steps = np.linalg.norm(wrapped_delta(ypr[1:], ypr[:-1]), axis=1)
        motion[1:] = np.minimum(motion[1:], steps)
        motion[:-1] = np.minimum(motion[:-1], steps)
    keep = np.flatnonzero(motion < 0.35)
    return np.asarray([g[i] for i in keep]), pc[keep], len(used), len(keep)


def optimize(runs, constrain_length=False):
    rvec0 = cv2.Rodrigues(R_C2I)[0].reshape(3)
    t0 = T_MEASURED.copy()
    centers0 = [np.median(np.asarray([g @ (R_C2I @ p + t0) for g, p in zip(gs, pcs)]), axis=0)
                for gs, pcs in runs]
    x0 = np.hstack((rvec0, t0, *centers0))

    def residual(x):
        r = cv2.Rodrigues(x[:3])[0]
        t = x[3:6]
        chunks = []
        for j, (gs, pcs) in enumerate(runs):
            center = x[6 + 3*j:9 + 3*j]
            chunks.append(np.asarray([g @ (r @ p + t) - center for g, p in zip(gs, pcs)]).ravel())
        if constrain_length:
            chunks.append(np.array([(np.linalg.norm(t) - np.linalg.norm(T_MEASURED)) / 0.02]))
        return np.hstack(chunks)

    result = least_squares(residual, x0, loss='huber', f_scale=0.008,
                           max_nfev=300, verbose=0)
    return cv2.Rodrigues(result.x[:3])[0], result.x[3:6], result.cost, result.optimality


def optimize_fixed_rotation(runs, constrain_length=False):
    t0 = T_MEASURED.copy()
    centers0 = [np.median(np.asarray([g @ (R_C2I @ p + t0) for g, p in zip(gs, pcs)]), axis=0)
                for gs, pcs in runs]
    x0 = np.hstack((t0, *centers0))
    measured_length = np.linalg.norm(T_MEASURED)

    def translation(x):
        if not constrain_length:
            return x[:3]
        return measured_length * x[:3] / np.linalg.norm(x[:3])

    def residual(x):
        t = translation(x)
        chunks = []
        for j, (gs, pcs) in enumerate(runs):
            center = x[3 + 3*j:6 + 3*j]
            chunks.append(np.asarray([g @ (R_C2I @ p + t) - center
                                      for g, p in zip(gs, pcs)]).ravel())
        return np.hstack(chunks)

    result = least_squares(residual, x0, loss='huber', f_scale=0.008,
                           max_nfev=300, verbose=0)
    return translation(result.x), result.cost, result.optimality


def score(run, r, t):
    gs, pcs = run
    pos = np.asarray([g @ (r @ p + t) for g, p in zip(gs, pcs)])
    center = np.median(pos, axis=0)
    e = np.linalg.norm(pos - center, axis=1) * 1000.
    return np.sqrt(np.mean(e**2)), np.median(e), np.percentile(e, 95), np.max(e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('directories', nargs='+')
    args = ap.parse_args()
    prepared = [prepare(d) for d in args.directories]
    runs = [(x[0], x[1]) for x in prepared]
    for i, x in enumerate(prepared, 1):
        print(f'run{i}: valid={x[2]} stationary={x[3]}')
    fits = []
    for i, run in enumerate(runs, 1):
        fits.append((f'fit_run{i}', *optimize([run])))
    fits.append(('fit_combined', *optimize(runs)))
    fits.append(('fit_combined_length_constrained', *optimize(runs, True)))
    for name, r, t, cost, optimality in fits:
        print(f'{name}: cost={cost:.6f} optimality={optimality:.3e}')
        print(f'  R=\n{np.array2string(r, precision=10)}')
        print(f'  T_mm={np.array2string(t*1000., precision=3)} norm_mm={np.linalg.norm(t)*1000.:.3f}')
        for i, run in enumerate(runs, 1):
            s = score(run, r, t)
            print(f'  run{i}: rms={s[0]:.3f} median={s[1]:.3f} p95={s[2]:.3f} max={s[3]:.3f} mm')

    fixed_fits = []
    for i, run in enumerate(runs, 1):
        fixed_fits.append((f'fixed_R_fit_run{i}', *optimize_fixed_rotation([run])))
    fixed_fits.append(('fixed_R_fit_combined', *optimize_fixed_rotation(runs)))
    fixed_fits.append(('fixed_R_fit_combined_length_constrained',
                       *optimize_fixed_rotation(runs, True)))
    print('FIXED_ORIGINAL_ROTATION_RESULTS')
    for name, t, cost, optimality in fixed_fits:
        print(f'{name}: cost={cost:.6f} optimality={optimality:.3e} '
              f'T_mm={np.array2string(t*1000., precision=3)} norm_mm={np.linalg.norm(t)*1000.:.3f}')
        for i, run in enumerate(runs, 1):
            s = score(run, R_C2I, t)
            print(f'  run{i}: rms={s[0]:.3f} median={s[1]:.3f} p95={s[2]:.3f} max={s[3]:.3f} mm')


if __name__ == '__main__':
    main()
