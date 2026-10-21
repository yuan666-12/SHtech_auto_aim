#!/usr/bin/env python3
"""Give each offline test a fresh diagnostic log, including repeated runs."""
from pathlib import Path
import os
import subprocess
import sys

log = Path(sys.argv[1])
log.parent.mkdir(parents=True, exist_ok=True)
log.write_text('', encoding='utf-8')
env = os.environ.copy()
env.update(AUTOAIM_HEADLESS='1', AUTOAIM_XUC_DRY_RUN='1', AUTOAIM_XUC_LOG_FILE=str(log))
raise SystemExit(subprocess.run(sys.argv[2:], env=env).returncode)
