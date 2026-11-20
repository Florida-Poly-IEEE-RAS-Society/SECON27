#!/usr/bin/env python3
"""Run your robot code against many random fields, one headless sim per run.

    ./simulation/run_sim.sh --batch --runs 10 \\
        --cmd "ros2 run my_pkg race_node --ros-args -p use_sim_time:=true"

Any other launch option (see README, "Launch options") goes after the batch
options as key:=value and applies to every run, e.g. a camera mount:

    ./simulation/run_sim.sh --batch --runs 5 --cmd "..." cam_x:=-0.11 cam_yaw:=15

For each run the script:
  1. starts the sim without its window (gui:=false) with that run's seed
  2. starts your command (optional; without it, this is a smoke test)
  3. starts a round, the same one as `run_sim.sh --start-round`: the start
     flag goes up --flag-delay sim seconds later, and the sim pauses
     --duration sim seconds after the flag
  4. watches the robot until the round is over or your command exits
  5. stops everything and moves on to the next seed

It does not score runs. It reports facts you can't easily see when nobody is
watching: did the robot move before the flag (early start, rule G05), did it
leave the start box, how far it drove, roughly how
many laps it made, where it ended, which cars it hit (the cars are dynamic,
so a hit moves them), and whether it left the field or stalled.

Each run is a fresh sim, so /odom starts at zero every time.

Results go to simulation/test_results/<date-time>/:
  summary.md     table of all runs
  results.json   the same, machine readable, with each run's field layout
  seed_<N>/      sim.log and robot.log for that run
"""

import argparse
import json
import math
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
import uuid

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node

SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.dirname(SCRIPTS_DIR)
REPO_DIR = os.path.dirname(SIM_DIR)
WORKSPACE_SETUP = os.path.join(REPO_DIR, 'ros2', 'install', 'setup.bash')
LAUNCH = os.path.join(SIM_DIR, 'launch', 'sim.launch.py')
sys.path.insert(0, SCRIPTS_DIR)
import field_spec as F  # noqa: E402
from sim_tui import zone  # noqa: E402

ROBOT = 'race_robot'
CAR_HIT_M = 0.02          # a car that moved more than this was hit
STALL_WINDOW_S = 10.0     # no movement for this long (sim time) = stalled
STALL_M = 0.05
FROZEN_S = 30.0           # wall seconds without sim time moving = sim froze
TRACK_CENTER = (0.0, F.m(F.RAIL_YC))
# Launch options the batch sets per run, and the batch option to use instead.
BATCH_OWNED = {'seed': '--seeds', 'gui': '--gui', 'obstacles': '--obstacles',
               'start_tag': '--start-tag', 'start_turn': '--start-turn',
               'tolerance': '--tolerance', 'randomize': '(nothing: every run is a new field)',
               'interval': '(nothing: one field per run)'}


def parse_seeds(text):
    seeds = []
    for part in text.split(','):
        part = part.strip()
        if '-' in part[1:]:
            a, b = part.split('-', 1) if not part.startswith('-') else part[1:].split('-', 1)
            seeds.extend(range(int(a), int(b) + 1))
        elif part:
            seeds.append(int(part))
    return seeds


def is_wsl():
    try:
        with open('/proc/version') as fh:
            return 'microsoft' in fh.read().lower()
    except OSError:
        return False


def sim_running():
    # Any `gz sim ...` process. With gui:=false there is no separate
    # "gz sim server" process, so don't look for that name.
    return subprocess.run(['pgrep', '-f', '^gz sim( |$)'],
                          stdout=subprocess.DEVNULL).returncode == 0


def kill_leftover_gazebo():
    """Gazebo's server and window run in their own process groups. If the
    launch had to be force-killed they can outlive it; clean them up so the
    next run doesn't start a second sim next to them."""
    for sig in ('-INT', '-KILL'):
        subprocess.run(['pkill', sig, '-f', '^gz sim( |$)'], stdout=subprocess.DEVNULL)
        end = time.time() + 10
        while sim_running() and time.time() < end:
            time.sleep(0.5)
        if not sim_running():
            return


class SimFroze(Exception):
    pass


def stop_group(proc, timeout=15):
    """SIGINT a process group, then SIGKILL if it hangs."""
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=timeout)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()


class Watcher:
    """Collects sim time, the field layout and model poses from Gazebo."""

    def __init__(self):
        self.lock = threading.Lock()
        self.sim_time = None
        self.layout = None
        self.poses = {}
        self.node = Node()
        self.node.subscribe(WorldStatistics, '/world/track/stats', self._stats)
        self.node.subscribe(StringMsg, '/field/layout', self._layout)
        self.node.subscribe(Pose_V, '/world/track/dynamic_pose/info', self._poses)
        self.cmd_pub = self.node.advertise('/field/command', StringMsg)

    def _stats(self, msg):
        with self.lock:
            self.sim_time = msg.sim_time.sec + msg.sim_time.nsec * 1e-9

    def _layout(self, msg):
        try:
            layout = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.layout = layout

    def _poses(self, msg):
        with self.lock:
            for p in msg.pose:
                if p.name == ROBOT:
                    # base_link is the rear axle; track the footprint center.
                    q = p.orientation
                    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                    cx, cy = F.robot_center(p.position.x, p.position.y, yaw)
                    self.poses[p.name] = (cx, cy, p.position.z)
                elif p.name.startswith('obstacle_'):
                    self.poses[p.name] = (p.position.x, p.position.y, p.position.z)

    def snapshot(self):
        with self.lock:
            return self.sim_time, self.layout, dict(self.poses)

    def wait_sim(self, sim, until):
        """Sleep a little; raise SimFroze if Gazebo exited or stopped advancing."""
        if sim.poll() is not None:
            raise SimFroze('sim stopped early (see sim.log)')
        t = self.snapshot()[0]
        if t != getattr(self, '_last_t', None):
            self._last_t, self._last_change = t, time.time()
        elif time.time() - self._last_change > FROZEN_S:
            raise SimFroze(f'sim froze (no progress for {FROZEN_S:.0f} s, see sim.log)')
        time.sleep(until)

    def reset(self):
        with self.lock:
            self.sim_time, self.layout, self.poses = None, None, {}
        self._last_t = self._last_change = None

    def start_round(self, rid, delay, duration):
        """Ask the field manager for a round (resend until the layout shows it)."""
        msg = StringMsg()
        msg.data = json.dumps({'cmd': 'round', 'id': rid, 'delay': delay, 'duration': duration})
        self.cmd_pub.publish(msg)

    def round(self, rid):
        """This round's state from the layout, or None until it's accepted."""
        with self.lock:
            r = (self.layout or {}).get('round') or {}
        return r if r.get('id') == rid else None


def run_one(seed, args, watcher, out_dir):
    run_dir = os.path.join(out_dir, f'seed_{seed}')
    os.makedirs(run_dir, exist_ok=True)
    result = {'seed': seed, 'status': 'ok'}

    launch_cmd = ['ros2', 'launch', LAUNCH, f'seed:={seed}',
                  f'gui:={"true" if args.gui else "false"}',
                  f'obstacles:={args.obstacles}', f'start_tag:={args.start_tag}',
                  f'start_turn:={args.start_turn}',
                  f'tolerance:={"true" if args.tolerance else "false"}'] + args.launch_args
    env = dict(os.environ)
    if not is_wsl():
        # Same rule as run_sim.sh: these crash Gazebo on native Linux / VMs,
        # but WSL needs them.
        env.pop('QT_QUICK_BACKEND', None)
        env.pop('LIBGL_ALWAYS_SOFTWARE', None)
    sim_log = open(os.path.join(run_dir, 'sim.log'), 'w')
    sim = subprocess.Popen(launch_cmd, stdout=sim_log, stderr=subprocess.STDOUT,
                           start_new_session=True, env=env)
    robot = robot_log = None
    watcher.reset()
    try:
        # Wait for this run's field and the robot.
        deadline = time.time() + args.boot_timeout
        while True:
            t, layout, poses = watcher.snapshot()
            if layout and layout.get('seed') == seed and ROBOT in poses and t is not None:
                break
            if sim.poll() is not None or time.time() > deadline:
                result['status'] = 'sim did not start (see sim.log)'
                return result
            time.sleep(0.2)
        result['assigned_pit'] = layout['assigned_pit']
        result['obstacles'] = [f"{o['name'].replace('obstacle_', '')} Q{o['quadrant']} {o['lane']}"
                               for o in layout['obstacles']]
        result['layout'] = layout
        used = {o['name'] for o in layout['obstacles']}
        time.sleep(0.5)  # let the cars settle on the board
        _, _, poses = watcher.snapshot()
        car_start = {n: p for n, p in poses.items() if n in used}

        if args.cmd:
            robot_log = open(os.path.join(run_dir, 'robot.log'), 'w')
            # Run in bash with the team workspace sourced, so `ros2 run <pkg>`
            # finds packages built in ros2/ even if your shell didn't source it.
            cmd = args.cmd
            if os.path.isfile(WORKSPACE_SETUP):
                cmd = f'source {shlex.quote(WORKSPACE_SETUP)} && {cmd}'
            robot = subprocess.Popen(['bash', '-c', cmd], stdout=robot_log,
                                     stderr=subprocess.STDOUT, start_new_session=True,
                                     env=env)

        # Start the round: the field manager raises the flag after flag_delay
        # sim seconds and pauses the sim after the match. Moving before the
        # flag is an early start.
        _, _, poses = watcher.snapshot()
        pre = poses[ROBOT]
        rid = uuid.uuid4().hex
        last_send = 0.0
        try:
            while True:
                r = watcher.round(rid)
                if r is None and time.time() - last_send > 0.5:
                    watcher.start_round(rid, args.flag_delay, args.duration)
                    last_send = time.time()
                if r is not None and r.get('state') in ('running', 'over'):
                    break
                if r is not None and r.get('state') == 'cancelled':
                    result['status'] = 'round cancelled (flag lowered or new field)'
                    return result
                watcher.wait_sim(sim, 0.05)
        except SimFroze as e:
            result['status'] = str(e)
            return result
        flag_t = r['flag_at']
        _, _, poses = watcher.snapshot()
        early = math.hypot(poses[ROBOT][0] - pre[0], poses[ROBOT][1] - pre[1]) > CAR_HIT_M

        # Watch the run.
        start_box = (F.m(F.START_BOX_X), F.m(F.START_BOX_Y))
        half_box = F.m(F.PIT_BOX / 2)
        last = None
        distance = 0.0
        left_box_t = None
        angle = 0.0
        last_angle = None
        off_field = False
        history = []          # (t, x, y) for stall detection
        end_reason = 'round over'
        while True:
            t, _, poses = watcher.snapshot()
            el = t - flag_t
            # The round ends when the field manager pauses the sim. The time
            # check is only a fallback if it never says so.
            if (watcher.round(rid) or {}).get('state') == 'over' or el >= args.duration + 5.0:
                break
            if args.cmd and robot.poll() is not None:
                end_reason = f'robot command exited ({robot.returncode})'
                break
            try:
                watcher.wait_sim(sim, 0.0)
            except SimFroze as e:
                end_reason = 'sim problem'
                result['status'] = str(e)
                break
            rx, ry, rz = poses[ROBOT]
            if last is not None:
                distance += math.hypot(rx - last[0], ry - last[1])
            last = (rx, ry)
            if left_box_t is None and (abs(rx - start_box[0]) > half_box or
                                       abs(ry - start_box[1]) > half_box):
                left_box_t = el
            if ry > F.m(F.DIVIDER_Y):          # count laps on the racecourse only
                a = math.atan2(ry - TRACK_CENTER[1], rx - TRACK_CENTER[0])
                if last_angle is not None:
                    d = (a - last_angle + math.pi) % (2 * math.pi) - math.pi
                    angle += d
                last_angle = a
            else:
                last_angle = None
            if zone(rx, ry) == 'OFF THE FIELD' or rz < -0.05:
                off_field = True
            history.append((el, rx, ry))
            time.sleep(0.05)

        _, _, poses = watcher.snapshot()
        rx, ry, _ = poses[ROBOT]
        hits = sorted(n.replace('obstacle_', '') for n, p in car_start.items()
                      if n in poses and math.hypot(poses[n][0] - p[0],
                                                   poses[n][1] - p[1]) > CAR_HIT_M)
        recent = [h for h in history if h[0] >= history[-1][0] - STALL_WINDOW_S] if history else []
        stalled = bool(recent) and history[-1][0] >= STALL_WINDOW_S and left_box_t is not None and \
            math.hypot(recent[-1][1] - recent[0][1], recent[-1][2] - recent[0][2]) < STALL_M
        result.update({
            'ended': end_reason,
            'early_start': early,
            'sim_seconds': round(history[-1][0], 1) if history else 0.0,
            'left_start_box_s': None if left_box_t is None else round(left_box_t, 2),
            'distance_m': round(distance, 2),
            'laps_approx': round(abs(angle) / (2 * math.pi), 2),
            'final_zone': zone(rx, ry),
            'cars_hit': hits,
            'off_field': off_field,
            'stalled_at_end': stalled,
        })
        return result
    finally:
        # A second Ctrl+C (or --stop's SIGTERM) must not abort cleanup half way.
        old_int = signal.signal(signal.SIGINT, signal.SIG_IGN)
        old_term = signal.signal(signal.SIGTERM, signal.SIG_IGN)
        stop_group(robot)
        stop_group(sim)
        for fh in (sim_log, robot_log):
            if fh:
                fh.close()
        # Wait for Gazebo to exit so the next run starts clean.
        end = time.time() + 20
        while sim_running() and time.time() < end:
            time.sleep(0.5)
        if sim_running():
            kill_leftover_gazebo()
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)


def write_summary(results, args, out_dir):
    with open(os.path.join(out_dir, 'results.json'), 'w') as fh:
        json.dump({'args': vars(args), 'runs': results}, fh, indent=2)
    lines = ['# Batch test', '',
             f'Command: `{args.cmd or "(none: smoke test)"}`  ',
             f'Duration: {args.duration:.0f} sim s after the flag. Obstacles: {args.obstacles}.'
             + (f" Launch options: `{' '.join(args.launch_args)}`." if args.launch_args else ''), '',
             '| Seed | Status | Pit | Early start | Left box (s) | Distance (m) | Laps ~ | Final zone | Cars hit | Off field | Stalled | Ended |',
             '|---|---|---|---|---|---|---|---|---|---|---|---|']
    for r in results:
        lines.append('| {seed} | {status} | {pit} | {early} | {left} | {dist} | {laps} | {zone} | {hits} | {off} | {stall} | {ended} |'.format(
            seed=r['seed'], status=r['status'], pit=r.get('assigned_pit', ''),
            early='YES' if r.get('early_start') else '-',
            left='' if r.get('left_start_box_s') is None else r['left_start_box_s'],
            dist=r.get('distance_m', ''), laps=r.get('laps_approx', ''),
            zone=r.get('final_zone', ''), hits=', '.join(r.get('cars_hit', [])) or '-',
            off='YES' if r.get('off_field') else '-', stall='YES' if r.get('stalled_at_end') else '-',
            ended=r.get('ended', '')))
    ok = [r for r in results if r['status'] == 'ok']
    if ok:
        lines += ['', f'{len(ok)}/{len(results)} runs completed. '
                      f'Early starts: {sum(r["early_start"] for r in ok)}. '
                      f'Left the start box: {sum(r["left_start_box_s"] is not None for r in ok)}. '
                      f'Hit a car: {sum(bool(r["cars_hit"]) for r in ok)}. '
                      f'Left the field: {sum(r["off_field"] for r in ok)}. '
                      f'Average laps: {sum(r["laps_approx"] for r in ok) / len(ok):.2f}.']
    text = '\n'.join(lines) + '\n'
    with open(os.path.join(out_dir, 'summary.md'), 'w') as fh:
        fh.write(text)
    return text


def main():
    p = argparse.ArgumentParser(
        description='Run robot code against many random fields (headless).',
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    g = p.add_mutually_exclusive_group()
    g.add_argument('--runs', type=int, default=5, help='number of random seeds (default 5)')
    g.add_argument('--seeds', help='exact seeds, e.g. "1,2,10-15"')
    p.add_argument('--cmd', default='', help='command that starts your robot code')
    p.add_argument('--duration', type=float, default=F.MATCH_SECONDS,
                   help='sim seconds from the flag until the sim pauses '
                        f'(default {F.MATCH_SECONDS:g} = one match)')
    p.add_argument('--flag-delay', type=float, default=F.ROUND_FLAG_DELAY,
                   help='sim seconds between your code starting and the flag '
                        f'(default {F.ROUND_FLAG_DELAY:g})')
    p.add_argument('--obstacles', type=int, default=F.DEFAULT_NUM_OBSTACLES)
    p.add_argument('--start-tag', type=int, default=0, choices=(0, 1, 2, 3))
    p.add_argument('--start-turn', type=float, default=0.0)
    p.add_argument('--tolerance', action='store_true')
    p.add_argument('--gui', action='store_true', help='show the Gazebo window (slower)')
    p.add_argument('--boot-timeout', type=float, default=120.0,
                   help='wall seconds to wait for each sim to start')
    p.add_argument('--out', default=None,
                   help='results folder (default simulation/test_results/<date-time>)')
    p.add_argument('launch_args', nargs='*', metavar='key:=value',
                   help='other launch options for every run, e.g. cam_x:=-0.11 cam_yaw:=15')
    args = p.parse_intermixed_args()
    for a in args.launch_args:
        key = a.split(':=', 1)[0]
        if ':=' not in a or not key:
            p.error(f'launch option {a!r} must look like key:=value')
        if key in BATCH_OWNED:
            p.error(f'{key}:= is set by the batch itself; use {BATCH_OWNED[key]} instead')

    # Clean up on Ctrl+C *and* on SIGTERM (e.g. ./run_sim.sh --stop): both
    # end up in run_one's `finally`, which stops your command and the sim.
    # Set explicitly, because a shell that starts this in the background can
    # leave SIGINT ignored, and Python then never sees Ctrl+C.
    def _interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)

    if sim_running():
        print('A Gazebo simulation is already running. Close it first:\n'
              '  ./simulation/run_sim.sh --stop', file=sys.stderr)
        return 1
    if args.seeds:
        seeds = parse_seeds(args.seeds)
    else:
        import random
        seeds = random.SystemRandom().sample(range(1, 1_000_000), args.runs)
    out_dir = args.out or os.path.join(SIM_DIR, 'test_results', time.strftime('%Y%m%d-%H%M%S'))
    os.makedirs(out_dir, exist_ok=True)

    watcher = Watcher()
    results = []
    interrupted = False
    for i, seed in enumerate(seeds, 1):
        print(f'[{i}/{len(seeds)}] seed {seed} ...', end=' ', flush=True)
        try:
            r = run_one(seed, args, watcher, out_dir)
        except KeyboardInterrupt:
            print('interrupted')
            interrupted = True
            break
        results.append(r)
        if r['status'] != 'ok':
            print(r['status'])
        else:
            print(f"pit {r['assigned_pit']}, {'EARLY START, ' if r['early_start'] else ''}"
                  f"left box {r['left_start_box_s']} s, "
                  f"{r['distance_m']} m, ~{r['laps_approx']} laps, ends in {r['final_zone']}, "
                  f"cars hit: {', '.join(r['cars_hit']) or 'none'}")
    print()
    print(write_summary(results, args, out_dir))
    print(f'Results: {out_dir}')
    return 130 if interrupted else 0


if __name__ == '__main__':
    sys.exit(main())
