#!/usr/bin/env python3
"""Randomize the race field while Gazebo runs, the way the referees do.

Each layout follows the Game Manual pre-match rules:
  * A die roll picks the start-area AprilTag: 1/4 -> ID 1, 2/5 -> ID 2,
    3/6 -> ID 3. That tag is put in the start-area slot; the other two are
    hidden. The ID is the robot's assigned pit box.
  * N plush cars (default 3) go in N different Appendix E quadrants on the
    straightaways, split across the inner and outer lanes (at least one each).
    Position inside the quadrant and heading are random.
  * The start flag goes back down.
  * Optional: every tag shifts up to +/- 1 in (<G02> field tolerance).

The script only talks to Gazebo (gz-transport), so it works with or without
ROS. ros_gz_bridge maps these topics to ROS 2 (config/bridge.yaml):

  Gazebo topic          ROS 2 topic           Type
  /field/layout         /field/layout         String (JSON, ground truth, 1 Hz)
  /field/randomize      /field/randomize      Int32  (seed; < 0 = random seed)
  /field/start_flag     /field/start_flag     Bool   (true = raise the flag)

Gazebo-only command topic, used by sim_tui.py (gz.msgs.StringMsg, JSON):
  /field/command  {"cmd": "layout", "seed": -1 | N | "same",
                   "start_tag": 0-3, "obstacles": 0-4, "tolerance": bool}
                  Keys left out keep their current value. start_tag,
                  obstacles and tolerance stay in effect for later layouts.
                  "same" re-applies the current seed with the new settings.
                  {"cmd": "flag", "up": true | false}

Examples (with the sim running):
  ros2 topic pub --once /field/randomize std_msgs/msg/Int32 "{data: -1}"
  ros2 topic pub --once /field/randomize std_msgs/msg/Int32 "{data: 42}"
  ros2 topic pub --once /field/start_flag std_msgs/msg/Bool "{data: true}"
  ros2 topic echo /field/layout

Run by hand:
  python3 field_manager.py                 # random layout, then serve commands
  python3 field_manager.py --seed 7 --once # one fixed layout, then exit
  python3 field_manager.py --start-tag 2 --obstacles 2
  python3 field_manager.py --interval 30   # new layout every 30 s
"""

import argparse
import json
import math
import os
import queue
import random
import sys
import threading
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.int32_pb2 import Int32
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import field_spec as F  # noqa: E402

WORLD = 'track'
CAR_NAMES = [f'obstacle_{c}' for c in F.CAR_COLORS]
CAR_Z = 0.002            # drop the car onto the board from 2 mm up
YAW_JITTER = math.radians(5)   # cars are set roughly along the lane


# --------------------------------------------------------------------------
# Layout generation (pure, no Gazebo)
# --------------------------------------------------------------------------
def make_layout(seed, num_obstacles, start_tag=None, tolerance=False):
    rng = random.Random(seed)
    die = rng.randint(1, 6)
    tag = start_tag if start_tag else (die - 1) % 3 + 1

    n = max(0, min(num_obstacles, len(F.QUADRANTS), len(CAR_NAMES)))
    quads = sorted(rng.sample(sorted(F.QUADRANTS), n))
    lanes = [rng.choice(('inner', 'outer')) for _ in quads]
    if n >= 2 and len(set(lanes)) == 1:
        i = rng.randrange(n)
        lanes[i] = 'outer' if lanes[i] == 'inner' else 'inner'
    cars = rng.sample(CAR_NAMES, n)

    obstacles = []
    # Footprint half-width / half-length with the worst-case yaw jitter.
    c, s = math.cos(YAW_JITTER), math.sin(YAW_JITTER)
    half_w = F.CAR_W / 2 * c + F.CAR_L / 2 * s
    half_l = F.CAR_L / 2 * c + F.CAR_W / 2 * s
    for q, lane, name in zip(quads, lanes, cars):
        quad = F.QUADRANTS[q]
        # Keep the whole car inside its lane (Appendix E photo). The inner
        # lane is ~11 in wide and the car 8.2 in, so there is little play.
        if lane == 'inner':   # rail edge .. lane line edge
            lo = F.RAIL_W / 2 + half_w
            hi = F.LANE_LINE_X - F.LINE_W / 2 - half_w
        else:                 # lane line edge .. wall
            lo = F.LANE_LINE_X + F.LINE_W / 2 + half_w
            hi = F.WALL_IN_X - half_w
        ax = rng.uniform(lo, max(lo, hi))
        y = rng.uniform(quad['y'][0] + half_l, quad['y'][1] - half_l)
        yaw = rng.choice((math.pi / 2, -math.pi / 2)) + rng.uniform(-YAW_JITTER, YAW_JITTER)
        obstacles.append({
            'name': name, 'quadrant': q, 'lane': lane,
            'x': round(F.m(quad['side'] * ax), 4), 'y': round(F.m(y), 4),
            'yaw': round(yaw, 4),
        })

    def shift():
        return F.m(rng.uniform(-F.TOLERANCE, F.TOLERANCE)) if tolerance else 0.0

    sx, sy, sz, *_ = F.tag_pose_start()
    start_tag_pose = [sx, sy + shift(), sz + shift() / 2]
    pit_tag_poses = {}
    for p in F.TAG_IDS:
        px, py, pz, *_ = F.tag_pose_pit(p)
        pit_tag_poses[p] = [px + shift(), py, pz + shift() / 2]

    return {
        'seed': seed,
        'die_roll': None if start_tag else die,
        'start_tag_id': tag,
        'assigned_pit': tag,
        'tag_family': F.TAG_FAMILY,
        'tag_size_m': round(F.TAG_SIZE_FOR_DETECTOR_M, 4),
        'start_tag_pose': [round(v, 4) for v in start_tag_pose],
        'pit_tag_poses': {k: [round(v, 4) for v in p] for k, p in pit_tag_poses.items()},
        'obstacles': obstacles,
        'start_flag': 'down',
        'tolerance_jitter': tolerance,
    }


# --------------------------------------------------------------------------
# Gazebo side
# --------------------------------------------------------------------------
def to_pose(name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    msg = Pose()
    msg.name = name
    msg.position.x, msg.position.y, msg.position.z = x, y, z
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    msg.orientation.w = cr * cp * cy + sr * sp * sy
    msg.orientation.x = sr * cp * cy - cr * sp * sy
    msg.orientation.y = cr * sp * cy + sr * cp * sy
    msg.orientation.z = cr * cp * sy - sr * sp * cy
    return msg


class FieldManager:
    def __init__(self, args):
        self.args = args
        self.node = Node()
        self.lock = threading.Lock()
        self.layout = None
        self.commands = queue.Queue()
        self.layout_pub = self.node.advertise('/field/layout', StringMsg)
        self.node.subscribe(Int32, '/field/randomize', self._on_randomize)
        self.node.subscribe(Boolean, '/field/start_flag', self._on_flag)
        self.node.subscribe(StringMsg, '/field/command', self._on_command)

    def _set_poses(self, poses):
        req = Pose_V()
        req.pose.extend(poses)
        ok, rep = self.node.request(f'/world/{WORLD}/set_pose_vector', req,
                                    Pose_V, Boolean, 3000)
        return ok and rep.data

    def wait_for_world(self, timeout):
        """Block until the world answers set_pose_vector."""
        end = time.time() + timeout
        probe = to_pose('field', 0, 0, 0)
        while time.time() < end:
            if self._set_poses([probe]):
                return True
            time.sleep(0.5)
        return False

    def apply(self, layout):
        poses = []
        used = set()
        for ob in layout['obstacles']:
            poses.append(to_pose(ob['name'], ob['x'], ob['y'], CAR_Z, yaw=ob['yaw']))
            used.add(ob['name'])
        for i, name in enumerate(CAR_NAMES):
            if name not in used:
                poses.append(to_pose(name, *F.car_pose_parked(i)))

        for n in F.TAG_IDS:
            if n == layout['start_tag_id']:
                x, y, z = layout['start_tag_pose']
                poses.append(to_pose(f'start_tag_{n}', x, y, z, yaw=F.START_TAG_YAW))
            else:
                poses.append(to_pose(f'start_tag_{n}', *F.tag_pose_parked(n)))
            x, y, z = layout['pit_tag_poses'][n]
            poses.append(to_pose(f'pit_tag_{n}', x, y, z, yaw=F.PIT_TAG_YAW))

        poses.append(to_pose('start_flag', *F.flag_pose(False)))
        if not self._set_poses(poses):
            print('field_manager: set_pose_vector failed', file=sys.stderr, flush=True)
            return False
        with self.lock:
            self.layout = layout
        self.publish()
        print('field_manager: layout ' + json.dumps(summary(layout)), flush=True)
        return True

    def randomize(self, seed=None):
        if seed is None or seed < 0:
            seed = random.SystemRandom().randrange(1_000_000)
        layout = make_layout(seed, self.args.obstacles, self.args.start_tag,
                             self.args.tolerance)
        layout['settings'] = {'start_tag': self.args.start_tag,
                              'obstacles': self.args.obstacles}
        layout['robot_start'] = [round(v, 4) for v in F.robot_spawn(self.args.start_turn)]
        return self.apply(layout)

    def keep_default(self):
        """Publish the world file's built-in layout (randomize:=false) without
        moving anything, so the control panel and /field/layout have it."""
        tag = F.DEFAULT_START_TAG
        layout = {
            'seed': None, 'die_roll': None,
            'start_tag_id': tag, 'assigned_pit': tag,
            'tag_family': F.TAG_FAMILY,
            'tag_size_m': round(F.TAG_SIZE_FOR_DETECTOR_M, 4),
            'start_tag_pose': [round(v, 4) for v in F.tag_pose_start()[:3]],
            'pit_tag_poses': {n: [round(v, 4) for v in F.tag_pose_pit(n)[:3]] for n in F.TAG_IDS},
            'obstacles': [],
            'start_flag': 'down',
            'tolerance_jitter': False,
            'settings': {'start_tag': tag, 'obstacles': len(F.DEFAULT_CARS)},
            'robot_start': [round(v, 4) for v in F.robot_spawn(self.args.start_turn)],
            'default_world_layout': True,
        }
        for color, q, lane in F.DEFAULT_CARS:
            x, y, _, _, _, yaw = F.default_car_pose(color)
            layout['obstacles'].append({'name': f'obstacle_{color}', 'quadrant': q,
                                        'lane': lane, 'x': round(x, 4), 'y': round(y, 4),
                                        'yaw': round(yaw, 4)})
        with self.lock:
            self.layout = layout
        self.publish()
        print('field_manager: keeping the world file layout ' + json.dumps(summary(layout)),
              flush=True)

    def set_flag(self, up):
        if not self._set_poses([to_pose('start_flag', *F.flag_pose(up))]):
            print('field_manager: start flag move failed', file=sys.stderr, flush=True)
            return
        with self.lock:
            if self.layout:
                self.layout['start_flag'] = 'up' if up else 'down'
        self.publish()
        print(f'field_manager: start flag {"up" if up else "down"}', flush=True)

    def command(self, cmd):
        """Handle one /field/command request (already parsed JSON)."""
        kind = cmd.get('cmd')
        if kind == 'flag':
            self.set_flag(bool(cmd.get('up', True)))
        elif kind == 'layout':
            if 'start_tag' in cmd and cmd['start_tag'] in (0,) + F.TAG_IDS:
                self.args.start_tag = int(cmd['start_tag'])
            if 'obstacles' in cmd:
                self.args.obstacles = max(0, min(len(CAR_NAMES), int(cmd['obstacles'])))
            if 'tolerance' in cmd:
                self.args.tolerance = bool(cmd['tolerance'])
            seed = cmd.get('seed', -1)
            if seed == 'same':
                with self.lock:
                    seed = self.layout['seed'] if self.layout else -1
            self.randomize(int(seed))
        else:
            print(f'field_manager: unknown command {cmd!r}', file=sys.stderr, flush=True)

    def publish(self):
        with self.lock:
            if self.layout is None:
                return
            msg = StringMsg()
            msg.data = json.dumps(self.layout)
        self.layout_pub.publish(msg)

    # gz-transport service requests fail when made from its callback threads,
    # so callbacks only queue work for the main loop (run_forever).
    def _on_randomize(self, msg):
        self.commands.put((self.randomize, msg.data))

    def _on_flag(self, msg):
        self.commands.put((self.set_flag, msg.data))

    def _on_command(self, msg):
        try:
            cmd = json.loads(msg.data)
        except ValueError:
            print(f'field_manager: bad command {msg.data!r}', file=sys.stderr, flush=True)
            return
        if isinstance(cmd, dict):
            self.commands.put((self.command, cmd))

    def run_forever(self, interval):
        last_pub = last_layout = time.time()
        while True:
            try:
                fn, arg = self.commands.get(timeout=0.2)
            except queue.Empty:
                fn = None
            if fn is not None:
                try:
                    fn(arg)
                except (ValueError, TypeError, KeyError) as e:
                    print(f'field_manager: ignored bad command {arg!r}: {e}',
                          file=sys.stderr, flush=True)
                if fn in (self.randomize, self.command):
                    last_layout = time.time()
            now = time.time()
            if now - last_pub >= 1.0:
                self.publish()
                last_pub = now
            if interval > 0 and now - last_layout >= interval:
                self.randomize()
                last_layout = now


def summary(layout):
    return {
        'seed': layout['seed'],
        'die_roll': layout['die_roll'],
        'assigned_pit': layout['assigned_pit'],
        'obstacles': [f"{o['name'].replace('obstacle_', '')}:Q{o['quadrant']}/{o['lane']}"
                      for o in layout['obstacles']],
    }


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--seed', type=int, default=-1,
                   help='layout seed; < 0 picks a random one (default)')
    p.add_argument('--obstacles', type=int, default=F.DEFAULT_NUM_OBSTACLES,
                   help='number of plush cars on the course, 0-4 (default 3)')
    p.add_argument('--start-tag', type=int, choices=(0,) + F.TAG_IDS, default=0,
                   help='force the start-area tag ID; 0 rolls a die (default)')
    p.add_argument('--tolerance', action='store_true',
                   help='shift tags up to +/- 1 in (field tolerance <G02>)')
    p.add_argument('--interval', type=float, default=0.0,
                   help='re-randomize every N seconds (0 = only on request)')
    p.add_argument('--once', action='store_true',
                   help='apply one layout and exit')
    p.add_argument('--keep', action='store_true',
                   help="keep the world file's default layout at startup")
    p.add_argument('--start-turn', type=float, default=0.0,
                   help='robot start heading, degrees left of straight ahead '
                        '(published in /field/layout for the control panel)')
    p.add_argument('--wait', type=float, default=60.0,
                   help='seconds to wait for Gazebo to come up')
    args = p.parse_args()

    fm = FieldManager(args)
    if not fm.wait_for_world(args.wait):
        print(f'field_manager: world "{WORLD}" not reachable after {args.wait:.0f} s',
              file=sys.stderr)
        return 1

    if args.keep:
        fm.keep_default()
    else:
        fm.randomize(args.seed)
    if args.once:
        time.sleep(0.5)   # let the layout message go out
        return 0

    print('field_manager: ready. Publish to /field/randomize or /field/start_flag.',
          flush=True)
    try:
        fm.run_forever(args.interval)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
