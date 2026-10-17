# Hero algorithm fixes 2026-10-16

Target: actual robot repository /home/rcs/rm_project/SHtech_auto_aim.
Branch: ax650-dev-2026.
Before-change snapshot: a3e297a824bf510dc38b9a61c6febb46bee2d1d8
Original snapshot 32d85c16cab3716054daee672834a5331ea88190 was reworded; source trees are identical..
Fixes remain in the robot working tree; no remote push.

TX yaw/pitch are FINAL absolute angles in degrees. MCU tracks these angles.
Rates are used only for PC extrapolation. Unused wire rate fields are zero.
No-target/invalid-result handling holds latest RX angles and resets smoothing.
Plans expire after 200 ms.

Changes: active vehicle IEKF iteration sign, angle wrapping, covariance update/reset;
tracking timestamps/reacquisition/lost-frame prediction; full world-normal rotation;
pivot-to-IMU mechanical translation in asset/gimbal_geometry_hero.yml;
absolute yaw/pitch and full horizontal pitch geometry; radius averaging and plate signs;
consistent ballistic angle/time with unreachable-input rejection;
PnP validity, orientation gate, image center and distortion-aware overlay;
finite-value guards and diagnostic planner rejection logs.

Camera K/D/R/T unchanged. SHA256:
63c48d47ebc05ab4a6a6c0de92b85f9b76d25eab2433ea655b6b5f9b5176970e

Validation: actual build_real/auto-aim built successfully with -j2.
python3 tools/run_hero_algorithm_check.py: 35 numerical checks passed against deployed objects.
Checks cover IEKF iteration/wrap/reset, ballistics, transforms/PnP at different poses,
world normals, distorted projection, final absolute angles and impossible-range rejection.
No camera or UART opened; no auto-aim started; no system environment/display changes.

Not verified: full historical-frame replay or real tracking with the new binary.
These fixes do not establish the unique first trigger of the historical divergence.
Existing exposure/IMU timing remains. Existing fixed bullet speed 10 m/s remains.
Ballistics currently use pivot origin; physical muzzle offset was not provided.

Backups: log/algorithm-fix-20261016-164250/
auto-aim.before and source.before.tar.gz preserve original deployment.
source.after.tar.gz, changes.final.patch, result.final.json preserve fixes/results.
For rollback stop auto-aim first, restore backup binary to build_real/auto-aim.
Source can be restored per changed file from snapshot commit.
Do not clean untracked files: collected calibration data must be preserved.
