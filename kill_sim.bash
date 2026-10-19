#!/usr/bin/env bash
# Gracefully stop only this user's Super Sim launch processes.
set -e
exec python3 - <<'PY'
import os
from pathlib import Path
import signal

if not Path('/proc').is_dir():
    raise SystemExit('Run inside Linux, or use Ctrl+C in the launch terminal.')
count = 0
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit():
        continue
    try:
        if entry.stat().st_uid != os.getuid():
            continue
        args = (entry / 'cmdline').read_bytes().decode().split('\0')
        for i, arg in enumerate(args):
            if Path(arg).name == 'ros2' and args[i+1:i+4] == ['launch', 'super_sim_dynamics', 'super_sim.launch.py']:
                os.kill(int(entry.name), signal.SIGINT)
                count += 1
                break
    except (OSError, UnicodeError):
        continue
print(f'Sent graceful shutdown to {count} Super Sim launch process(es).')
PY
