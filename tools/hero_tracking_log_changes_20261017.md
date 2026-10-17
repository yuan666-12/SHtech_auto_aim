# Hero tracking and logging fixes - 2026-10-17

Deployment: /home/rcs/rm_project/SHtech_auto_aim
Before-change Git snapshot: 46a1df2f7aa688f3daf4a63b9a6a452544774239
Snapshot title: see git log -1 46a1df2. No remote push was performed.

## Visual observation lifetime
- Track the timestamp of the last accepted REAL observation.
- Maximum age is 200 ms at tracker, planner and serial output.
- TEMP_LOST briefly holds the previous final absolute angle, with zero
  extrapolation rates and no fire advice. It does not chase a predicted ghost.
- After expiry cancel the target and hold the latest RX attitude.
- New plans cannot extend an old visual observation's deadline.
- Genuine visual reacquisition resets filters, covariance and target velocity.

## Measurement and prediction quality
- A position jump over the existing 0.2 m threshold needs another coherent
  observation. A repeated new position or consistent fast motion can confirm it.
- Confirmed continuous translation retains velocity and is not treated as an
  armor switch simply because it moved over 0.2 m between frames.
- Actual armor-switch handling also requires a normal change of at least 45 deg.
- PnP rejects non-finite/non-convex/degenerate corner sets and reprojection
  RMS over 5 pixels. This threshold still needs field validation.
- Predicted range below half or above twice the accepted observed range falls
  back to the accepted observation and suppresses fire advice. This is a gross
  prediction guard, not a proof that every finite measurement is correct.

## Image / attitude timing
- Save frame receipt time BEFORE Bayer conversion, plus raw SDK frame number,
  host timestamp and device timestamp (raw units; no undocumented conversion).
- Match frame receipt time against a two-second RX attitude history.
- Interpolate short adjacent samples, handling yaw wrap. Reject unavailable
  pose history or nearest samples over 30 ms away; do not invent an old pose.
- AUTOAIM_IMAGE_DELAY_MS optionally subtracts a measured image latency
  (0..200 ms). Default is 0: no unmeasured exposure delay is assumed.
- Exact exposure/MCU sampling synchronization remains uncalibrated. Frame
  receipt time is not exposure time; this change does not claim otherwise.
- RX callbacks and TX UART writes occur before diagnostic disk formatting.

## Logs
Same directory: log/xuc/xuc_YYYY-MM-DD-HH-MM-SS.log
Same optional override: AUTOAIM_XUC_LOG_FILE
Normal RX/TX/VISION records: at most one sampled record per stream per 100 ms.
State/mode/target/quality transitions bypass the sampling interval.
Console RX/TX summaries: about once per second, plus state transitions.
File flush: once per second and immediately for state transitions.
Pitch limits retain [0,43] deg; mechanically clamped targets suppress fire advice.
Raw hex packet dumps are OFF by default.
No historical logs or calibration images are deleted.

Optional command-prefix settings (process-local; no system settings changed):
- AUTOAIM_XUC_LOG_INTERVAL_MS=100 : default 10 Hz file sampling.
- AUTOAIM_XUC_LOG_INTERVAL_MS=0 : explicit full-rate diagnostic capture.
- AUTOAIM_XUC_LOG_RAW=1 : include raw protocol bytes when required.
- AUTOAIM_IMAGE_DELAY_MS=... : measured delay only; leave unset otherwise.

New [VISION] records include state, model, aim mode, observation age,
observation/reacquisition/rejection flags, raw and accepted world measurement,
filtered position/velocity, predicted aim point, planned angles, pose timing,
camera timestamps and frame number. RX/TX records include sequence and skipped
counts. TX includes target_id, actual write result and pitch-clamp status.

## Validation and rollback
Build: cmake --build build_real --target auto-aim -j2
Numeric regression: python3 tools/run_hero_algorithm_check.py
Lifecycle/timing/logging regression: python3 tools/run_hero_tracking_check.py
Tests use in-memory driver and DRY_RUN only; no camera or UART is opened.

Backup directory is recorded in the task result and snapshot.json.
auto-aim.before preserves the old executable. Restore only after stopping
auto-aim. Source rollback point is 46a1df2. Preserve unrelated concurrent edits.
Camera K/D/R/T, fixed 10 m/s bullet speed and mechanical calibration are unchanged.
PID parameters and lower-controller firmware are unchanged.

Final verification: build exit 0; 35 numeric checks plus 47 lifecycle/timing/log
checks passed. The actual driver emitted two TX rows for 201 rapid normal/state
packets in the isolated no-UART test. No camera or UART was opened.
The completed fixes remain in the working tree after the before-change snapshot.
