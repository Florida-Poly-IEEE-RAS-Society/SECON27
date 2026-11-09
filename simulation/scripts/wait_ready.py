#!/usr/bin/env python3
"""Wait until a running sim is ready to use, then exit.

Ready means: the field manager has published the field layout and the
robot's camera is sending frames (so the world is up and the robot is
spawned). Used by `run_sim.sh --no-gui`; also handy in your own scripts:

    python3 simulation/scripts/wait_ready.py --timeout 120

Exit code 0 = ready, 1 = timed out, 2 = the sim process (--pid) exited.

Only light topics are watched. Don't subscribe to /clock or
/world/track/pose/info from Python to check readiness: Gazebo publishes those
every physics step (1000 times a second), which swamps a Python process.
"""

import argparse
import os
import sys
import threading
import time

from gz.msgs10.image_pb2 import Image
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--timeout', type=float, default=120.0, help='wall seconds (default 120)')
    p.add_argument('--pid', type=int, help='give up early if this process exits')
    args = p.parse_args()

    seen = {'layout': threading.Event(), 'camera': threading.Event()}
    node = Node()
    node.subscribe(StringMsg, '/field/layout', lambda _m: seen['layout'].set())
    node.subscribe(Image, '/camera/image', lambda _m: seen['camera'].set())

    end = time.time() + args.timeout
    while time.time() < end:
        if all(e.is_set() for e in seen.values()):
            return 0
        if args.pid and not os.path.exists(f'/proc/{args.pid}'):
            return 2
        time.sleep(0.2)
    missing = [k for k, e in seen.items() if not e.is_set()]
    print(f'wait_ready: no {" or ".join(missing)} after {args.timeout:.0f} s', file=sys.stderr)
    return 1


if __name__ == '__main__':
    code = main()
    # Skip the normal interpreter teardown: gz-transport's threads can abort
    # the process on the way out (exit 134), which run_sim.sh would read as
    # "the sim didn't start".
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
