# Hero camera parameters - active set

Active files as of 2026-09-04:

- cam_param_hero.yml: current Hero runtime camera file. K/D use the 2026-09-03 intrinsic calibration for the replacement Hikrobot MV-CS016-10UC camera.
- cam_param_hero_intrinsics_20260903.yml: standalone intrinsic calibration output from 2026-09-03.

Important caveat:

- R_c2i and T_c2i in cam_param_hero.yml are retained legacy values only because the runtime expects extrinsics. They are not verified for the replacement camera and must be replaced by Hero-specific hand-eye/physical calibration.
- Old sentry/infantry/aerial/balance camera files were moved to _archive_nonhero_20260904/asset/camParam/ to avoid accidental selection.
