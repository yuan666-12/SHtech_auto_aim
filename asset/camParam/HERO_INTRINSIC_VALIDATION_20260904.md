# Hero intrinsic validation - 2026-09-04

Input data: `calib_frames/raw_*.png`, captured 2026-09-03 after replacing the same-model Hikrobot camera.

## Summary
- Frames scanned: 66
- Valid chessboard frames: 57
- Invalid frames: 9
- Image size: 1440 x 1080
- Pattern: 11 x 8 inner corners, square 15.0 mm
- OpenCV calibrateCamera global RMS: 0.410486 px
- Mean per-frame L2/corner error: 0.030354 px
- Mean per-frame RMS reprojection error: 0.284750 px
- Active K/D solvePnP mean L2/corner error: 0.030354 px
- Active K/D solvePnP mean RMS error: 0.284750 px

## Recomputed Intrinsics
```
K =
[[1.47908079e+03 0.00000000e+00 7.30699685e+02]
 [0.00000000e+00 1.47814392e+03 5.48180146e+02]
 [0.00000000e+00 0.00000000e+00 1.00000000e+00]]
D =
[-0.13372672  0.21795173 -0.0004512   0.00081428 -0.11184162]
```

## Difference vs active cam_param_hero.yml
```
K_recomputed - K_active =
[[ 3.3353444451e-08  0.0000000000e+00 -3.4140157368e-10]
 [ 0.0000000000e+00  3.3505330066e-08  1.2606733435e-09]
 [ 0.0000000000e+00  0.0000000000e+00  0.0000000000e+00]]
D_recomputed - D_active =
[-7.3629990993e-12  2.8374969041e-11 -4.2337509366e-13  4.1206003453e-13
 -3.4053634912e-11]
```

## Worst frames by recomputed RMS
- raw_4.png: L2/corner=0.261922px, RMS=2.457047px
- raw_5.png: L2/corner=0.053709px, RMS=0.503831px
- raw_0.png: L2/corner=0.042020px, RMS=0.394187px
- raw_60.png: L2/corner=0.036754px, RMS=0.344779px
- raw_41.png: L2/corner=0.033640px, RMS=0.315570px
- raw_58.png: L2/corner=0.033137px, RMS=0.310850px
- raw_59.png: L2/corner=0.032712px, RMS=0.306870px
- raw_7.png: L2/corner=0.031728px, RMS=0.297632px
- raw_39.png: L2/corner=0.030962px, RMS=0.290450px
- raw_8.png: L2/corner=0.030885px, RMS=0.289731px

## Invalid frames
- raw_1.png: no_corners
- raw_6.png: no_corners
- raw_11.png: no_corners
- raw_12.png: no_corners
- raw_14.png: no_corners
- raw_29.png: no_corners
- raw_32.png: no_corners
- raw_34.png: no_corners
- raw_35.png: no_corners

## Visual check
- Wrote undistortion montage: `asset/camParam/hero_undistort_check_20260904.png`

## Caveat
- This validates only intrinsics K/D. Hero extrinsics R_c2i/T_c2i are still legacy placeholders and remain marked TODO in cam_param_hero.yml.
