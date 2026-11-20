#!/usr/bin/env python3
"""Run the match on the running sim, like the referee.

    ./simulation/run_sim.sh --start-round     (match.py start)
    ./simulation/run_sim.sh --reset           (match.py reset)

start: the field manager lowers the start flag, resumes the sim if it's
       paused, raises the flag ROUND_FLAG_DELAY (3) sim seconds later, and
       pauses the sim when the 3:00 match is over. This stays attached and
       reports the round: flag up, time left, round over. Ctrl+C only stops
       the reporting; the round keeps running in the sim.
reset: the robot is stopped and put back in the start box, the flag goes
       down and any round ends. The field stays the same; /odom does not
       reset. Returns as soon as it's done.

Exit code 0 = done (round over / reset done), 1 = no sim answered or the
sim went away, 3 = the round was cancelled (flag lowered, reset, new field
or another round started).
"""

import json
import os
import sys
import threading
import time
import uuid

from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import field_spec as F  # noqa: E402

TIMEOUT_S = 10.0          # wall seconds to wait for the sim to take a request
RESEND_S = 0.5
LOST_S = 5.0              # no layout message for this long = the sim is gone
REPORT_LEFT = (120, 60, 30, 10)   # also say how much time is left at these marks


def clock(seconds):
    s = max(0, int(round(seconds)))
    return f'{s // 60}:{s % 60:02d}'


class Field:
    """Talks to the field manager: sends commands, follows /field/layout."""

    def __init__(self):
        self.lock = threading.Lock()
        self.round = None
        self.layout_at = 0.0
        self.sim_time = None
        self.node = Node()
        self.node.subscribe(StringMsg, '/field/layout', self._on_layout)
        self.node.subscribe(WorldStatistics, '/world/track/stats', self._on_stats)
        self.pub = self.node.advertise('/field/command', StringMsg)

    def _on_layout(self, msg):
        try:
            r = json.loads(msg.data).get('round') or {}
        except ValueError:
            return
        with self.lock:
            self.round, self.layout_at = r, time.time()

    def _on_stats(self, msg):
        with self.lock:
            self.sim_time = msg.sim_time.sec + msg.sim_time.nsec * 1e-9

    def snapshot(self):
        with self.lock:
            return dict(self.round or {}), self.layout_at, self.sim_time

    def send(self, cmd):
        """Send a command and wait until /field/layout shows it; returns its id.

        A single publish right after connecting can be lost before the field
        manager is discovered, so resend until the layout's round carries this
        request's id (the field manager ignores repeats of the same id).
        """
        rid = uuid.uuid4().hex
        req = StringMsg()
        req.data = json.dumps(dict(cmd, id=rid))
        end = time.time() + TIMEOUT_S
        while time.time() < end:
            self.pub.publish(req)
            stop = time.time() + RESEND_S
            while time.time() < stop:
                if self.snapshot()[0].get('id') == rid:
                    return rid
                time.sleep(0.02)
        return None


def follow(field, rid):
    """Report the round until it's over. Returns the exit code."""
    said_countdown = said_flag = False
    marks = list(REPORT_LEFT)
    while True:
        r, layout_at, now = field.snapshot()
        if time.time() - layout_at > LOST_S:
            print('Lost the sim (stopped?). The round result is unknown.', file=sys.stderr)
            return 1
        if r.get('id') != rid:
            print('Round ended by a match reset.' if r.get('state') == 'idle' else
                  'Round cancelled: another round was started.')
            return 3
        state = r.get('state')
        if state == 'countdown' and not said_countdown:
            print(f'Start flag goes up in {r["delay"]:g} s ...', flush=True)
            said_countdown = True
        elif state in ('running', 'over') and not said_flag:
            print(f'GREEN FLAG at sim {r["flag_at"]:.1f} s: the {clock(r["duration"])} '
                  'match is on.', flush=True)
            said_flag = True
        if state == 'running' and now is not None and r.get('ends_at') is not None:
            left = r['ends_at'] - now
            while marks and left <= marks[0]:
                if left > marks[0] - 5:          # don't report marks already long past
                    print(f'  {clock(marks[0])} left', flush=True)
                marks.pop(0)
        if state == 'over':
            print(f'ROUND OVER at sim {r["ended_at"]:.1f} s: {clock(r["duration"])} played, '
                  'sim paused.\nNext round: ./simulation/run_sim.sh --reset, then '
                  '--start-round.  Close the sim: --stop')
            return 0
        if state == 'cancelled':
            print('Round cancelled (start flag lowered, reset or new field).')
            return 3
        time.sleep(0.1)


def main():
    action = sys.argv[1] if len(sys.argv) == 2 else ''
    if action not in ('start', 'reset'):
        print('usage: match.py start | reset', file=sys.stderr)
        return 2
    field = Field()
    if action == 'reset':
        if field.send({'cmd': 'reset'}) is None:
            return no_sim()
        print('Match reset: robot stopped in the start box, flag down, round ended.')
        return 0

    rid = field.send({'cmd': 'round', 'delay': F.ROUND_FLAG_DELAY,
                      'duration': F.MATCH_SECONDS})
    if rid is None:
        return no_sim()
    try:
        return follow(field, rid)
    except KeyboardInterrupt:
        print('\nStopped watching. The round keeps running in the sim '
              '(see --tui, or end it with --reset).')
        return 0


def no_sim():
    print('No sim answered. Start one first: ./simulation/run_sim.sh  (or --no-gui)',
          file=sys.stderr)
    return 1


if __name__ == '__main__':
    code = main()
    # Exit without the normal interpreter teardown: gz-transport's threads can
    # abort the process on the way out ("terminate called without an active
    # exception", exit 134) after the work is already done.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
