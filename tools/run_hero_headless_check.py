#!/usr/bin/env python3
"""Compatibility entry point; compile tests from current source."""
import subprocess
import sys
from pathlib import Path

if __name__ == "__main__":
    raise SystemExit(subprocess.call([sys.executable, str(Path(__file__).with_name("run_offline_checks.py")), "--case", '^(headless|gui_delegation)$'] + sys.argv[1:]))
