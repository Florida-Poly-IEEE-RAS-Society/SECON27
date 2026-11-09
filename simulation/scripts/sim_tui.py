#!/usr/bin/env python3
"""Terminal control panel for the race simulation.

Run in a second terminal while the sim is running:

    ./simulation/run_sim.sh --tui

Shows the sim clock, match timer, field layout and robot position, and gives
one-key commands: raise the start flag, reset the match, new random field,
force the AprilTag, change the obstacle count, drive the robot, pause, ...

Talks to Gazebo directly over gz-transport (no ROS nodes needed). Field
changes go through field_manager.py's /field/command topic, so the layout on
/field/layout always matches what is on screen.
"""

import curses
import json
import math
import os
import sys
import threading
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.world_control_pb2 import WorldControl
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import field_spec as F  # noqa: E402
from field_manager import to_pose  # noqa: E402

WORLD = 'track'
ROBOT = 'race_robot'
MATCH_SECONDS = 180

DRIVE_LIN_STEP, DRIVE_LIN_MAX = 0.05, 0.6     # m/s
DRIVE_ANG_STEP, DRIVE_ANG_MAX = 0.3, 2.5      # rad/s

MENU = [
    ('MATCH', [
        ('1', 'Raise start flag (starts timer)'),
        ('2', 'Lower start flag'),
        ('3', 'Reset match'),
        ('4', 'New match (reset + new field)'),
    ]),
    ('FIELD', [
        ('5', 'New random field'),
        ('6', 'New field from seed...'),
        ('7', 'Start tag: die / 1 / 2 / 3...'),
        ('8', 'Number of obstacles...'),
        ('9', 'Tolerance jitter on/off'),
    ]),
    ('ROBOT / SIM', [
        ('d', 'Drive the robot (WASD)'),
        ('x', 'Stop the robot'),
        ('b', 'Robot back to start box'),
        ('p', 'Pause / resume'),
        ('q', 'Quit'),
    ]),
]


# --------------------------------------------------------------------------
# Field geometry helpers
# --------------------------------------------------------------------------
def _in_box(x, y, cx, cy, half):
    return abs(x - cx) <= half and abs(y - cy) <= half


def _in_polygon(x, y, pts):
    inside = False
    for i in range(len(pts)):
        (x0, y0), (x1, y1) = pts[i], pts[i - 1]
        if (y0 > y) != (y1 > y) and x < (x1 - x0) * (y - y0) / (y1 - y0) + x0:
            inside = not inside
    return inside


OCTAGON = F.octagon_vertices()


def zone(x_m, y_m):
    """Where the robot center is, in plain words."""
    x, y = x_m / F.INCH, y_m / F.INCH
    if abs(x) > F.BOARD_W / 2 or abs(y) > F.BOARD_L / 2:
        return 'OFF THE FIELD'
    half = F.PIT_BOX / 2
    if _in_box(x, y, F.START_BOX_X, F.START_BOX_Y, half):
        return 'start box'
    for n, px in F.PIT_BOX_X.items():
        if _in_box(x, y, px, F.PIT_BOX_Y, half):
            return f'pit box {n}'
    if y < F.DIVIDER_Y - 0.5:
        return 'pit area'
    if y < F.DIVIDER_Y + 1.5:
        return 'start/finish line'
    lane = 'inner' if _in_polygon(x, y, OCTAGON) else 'outer'
    return f'racecourse, {lane} lane'


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def fmt_time(seconds):
    if seconds is None:
        return '--:--'
    sign = '-' if seconds < 0 else ''
    tenths = round(abs(seconds) * 10)     # round once, so 59.96 s -> 1:00.0
    return f'{sign}{tenths // 600}:{tenths % 600 / 10:04.1f}'


# --------------------------------------------------------------------------
# State shared with gz-transport callback threads
# --------------------------------------------------------------------------
class SimState:
    def __init__(self):
        self.lock = threading.Lock()
        self.sim_time = None
        self.paused = False
        self.rtf = 0.0
        self.stats_at = 0.0
        self.layout = None
        self.layout_at = 0.0
        self.robot = None          # (x, y, yaw)
        self.robot_speed = 0.0
        self._last_pose = None     # (t, x, y)

    def on_stats(self, msg):
        with self.lock:
            self.sim_time = msg.sim_time.sec + msg.sim_time.nsec * 1e-9
            self.paused = msg.paused
            self.rtf = msg.real_time_factor
            self.stats_at = time.time()

    def on_layout(self, msg):
        try:
            layout = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.layout = layout
            self.layout_at = time.time()

    def on_poses(self, msg):
        for p in msg.pose:
            if p.name == ROBOT:
                t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
                x, y = p.position.x, p.position.y
                with self.lock:
                    if self._last_pose and t > self._last_pose[0] + 0.05:
                        t0, x0, y0 = self._last_pose
                        self.robot_speed = math.hypot(x - x0, y - y0) / (t - t0)
                        self._last_pose = (t, x, y)
                    elif not self._last_pose or t < self._last_pose[0]:
                        self._last_pose = (t, x, y)
                    self.robot = (x, y, yaw_of(p.orientation))
                return


# --------------------------------------------------------------------------
# The TUI
# --------------------------------------------------------------------------
class SimTui:
    def __init__(self, screen):
        self.scr = screen
        self.state = SimState()
        self.node = Node()
        self.node.subscribe(WorldStatistics, f'/world/{WORLD}/stats', self.state.on_stats)
        self.node.subscribe(StringMsg, '/field/layout', self.state.on_layout)
        self.node.subscribe(Pose_V, f'/world/{WORLD}/dynamic_pose/info', self.state.on_poses)
        self.field_cmd = self.node.advertise('/field/command', StringMsg)
        self.cmd_vel = self.node.advertise('/cmd_vel', Twist)

        self.log = []
        self.match_start = None      # sim time the flag went up
        self.match_end = None        # sim time the flag went down
        self.last_flag = None
        self.drive = None            # [linear, angular] while driving

        curses.curs_set(0)
        self.scr.nodelay(True)
        self.scr.timeout(100)
        self.colors = curses.has_colors()
        if self.colors:
            curses.start_color()
            curses.use_default_colors()
            for i, c in enumerate((curses.COLOR_GREEN, curses.COLOR_YELLOW,
                                   curses.COLOR_RED, curses.COLOR_CYAN), start=1):
                curses.init_pair(i, c, -1)

    # ---- helpers ---------------------------------------------------------
    def color(self, name):
        if not self.colors:
            return 0
        return curses.color_pair({'green': 1, 'yellow': 2, 'red': 3, 'cyan': 4}[name])

    def note(self, text):
        self.log.append(f'{time.strftime("%H:%M:%S")}  {text}')
        self.log = self.log[-6:]

    def send_field(self, **cmd):
        msg = StringMsg()
        msg.data = json.dumps(cmd)
        self.field_cmd.publish(msg)

    def twist(self, lin, ang):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.cmd_vel.publish(msg)

    def request(self, service, req, req_type):
        ok, rep = self.node.request(service, req, req_type, Boolean, 3000)
        return ok and rep.data

    def connected(self):
        with self.state.lock:
            return time.time() - self.state.stats_at < 2.0

    def manager_up(self):
        with self.state.lock:
            return time.time() - self.state.layout_at < 3.0

    # ---- commands --------------------------------------------------------
    def set_flag(self, up):
        self.send_field(cmd='flag', up=up)
        self.note('Start flag ' + ('raised' if up else 'lowered'))

    def robot_to_start(self):
        self.twist(0.0, 0.0)
        with self.state.lock:
            start = (self.state.layout or {}).get('robot_start')
        x, y, z, yaw = start if start else F.ROBOT_SPAWN
        pose = to_pose(ROBOT, x, y, z, yaw=yaw)
        if self.request(f'/world/{WORLD}/set_pose', pose, type(pose)):
            self.note('Robot moved to the start box (note: /odom does not reset)')
        else:
            self.note('Could not move the robot')

    def reset_match(self, new_field=False):
        self.drive = None
        self.robot_to_start()
        if new_field:
            self.send_field(cmd='layout', seed=-1)     # also lowers the flag
        else:
            self.send_field(cmd='flag', up=False)
        self.match_start = self.match_end = None
        self.last_flag = 'down'
        # Show the flag as down right away. Otherwise the last "up" layout
        # message, still on screen until field_manager answers, would look
        # like a new match start.
        with self.state.lock:
            if self.state.layout:
                self.state.layout['start_flag'] = 'down'
        self.note('New match: new field, robot in start box' if new_field
                  else 'Match reset: robot in start box, flag down')

    def toggle_pause(self):
        with self.state.lock:
            paused = self.state.paused
        req = WorldControl()
        req.pause = not paused
        if self.request(f'/world/{WORLD}/control', req, WorldControl):
            self.note('Simulation ' + ('resumed' if paused else 'paused'))

    def prompt(self, question):
        """Read a line of input at the bottom of the screen."""
        h, w = self.scr.getmaxyx()
        self.scr.nodelay(False)
        curses.echo()
        curses.curs_set(1)
        self.scr.move(h - 1, 0)
        self.scr.clrtoeol()
        self.scr.addstr(h - 1, 0, (question + ' ')[:w - 1], curses.A_BOLD)
        try:
            text = self.scr.getstr(h - 1, min(len(question) + 1, w - 2), 20)
            return text.decode(errors='ignore').strip()
        finally:
            curses.noecho()
            curses.curs_set(0)
            self.scr.nodelay(True)
            self.scr.timeout(100)

    def handle(self, key):
        if self.drive is not None:
            return self.handle_drive(key)
        k = chr(key).lower() if 0 <= key < 256 else ''
        if k == '1':
            self.set_flag(True)
        elif k == '2':
            self.set_flag(False)
        elif k == '3':
            self.reset_match()
        elif k == '4':
            self.reset_match(new_field=True)
        elif k == '5':
            self.send_field(cmd='layout', seed=-1)
            self.note('New random field')
        elif k == '6':
            text = self.prompt('Seed (whole number):')
            if text.lstrip('-').isdigit():
                self.send_field(cmd='layout', seed=int(text))
                self.note(f'New field from seed {int(text)}')
            elif text:
                self.note(f'Not a number: {text!r}')
        elif k == '7':
            text = self.prompt('Start tag: 0 = roll a die, or 1 / 2 / 3:')
            if text in ('0', '1', '2', '3'):
                self.send_field(cmd='layout', seed='same', start_tag=int(text))
                self.note('Start tag: ' + ('die roll' if text == '0' else f'always ID {text}'))
            elif text:
                self.note(f'Start tag must be 0-3, got {text!r}')
        elif k == '8':
            text = self.prompt('Number of obstacles (0-4):')
            if text in ('0', '1', '2', '3', '4'):
                self.send_field(cmd='layout', seed='same', obstacles=int(text))
                self.note(f'Obstacles: {text}')
            elif text:
                self.note(f'Obstacles must be 0-4, got {text!r}')
        elif k == '9':
            with self.state.lock:
                on = bool(self.state.layout and self.state.layout.get('tolerance_jitter'))
            self.send_field(cmd='layout', seed='same', tolerance=not on)
            self.note('Tolerance jitter ' + ('off' if on else 'on (tags shift up to 1 in)'))
        elif k == 'd':
            self.drive = [0.0, 0.0]
            self.note('Drive mode on. Make sure your own controller is not running.')
        elif k == 'x':
            self.twist(0.0, 0.0)
            self.note('Robot stopped')
        elif k == 'b':
            self.robot_to_start()
        elif k == 'p':
            self.toggle_pause()
        elif k == 'q':
            return False
        return True

    def handle_drive(self, key):
        k = chr(key).lower() if 0 <= key < 256 else ''
        lin, ang = self.drive
        if k == 'w' or key == curses.KEY_UP:
            lin = min(DRIVE_LIN_MAX, lin + DRIVE_LIN_STEP)
        elif k == 's' or key == curses.KEY_DOWN:
            lin = max(-DRIVE_LIN_MAX, lin - DRIVE_LIN_STEP)
        elif k == 'a' or key == curses.KEY_LEFT:
            ang = min(DRIVE_ANG_MAX, ang + DRIVE_ANG_STEP)
        elif k == 'd' or key == curses.KEY_RIGHT:
            ang = max(-DRIVE_ANG_MAX, ang - DRIVE_ANG_STEP)
        elif k == ' ':
            lin = ang = 0.0
        elif k in ('q', 'e') or key == 27:      # q, e or Esc leaves drive mode
            self.drive = None
            self.twist(0.0, 0.0)
            self.note('Drive mode off, robot stopped')
            return True
        self.drive = [round(lin, 3), round(ang, 3)]
        return True

    # ---- match timer -----------------------------------------------------
    def update_timer(self):
        with self.state.lock:
            flag = (self.state.layout or {}).get('start_flag')
            now = self.state.sim_time
        if flag == self.last_flag or now is None:
            return
        if flag == 'up':
            self.match_start, self.match_end = now, None
            self.note('Match started (sim time)')
        elif flag == 'down' and self.match_start is not None and self.match_end is None:
            self.match_end = now
        self.last_flag = flag

    # ---- drawing ---------------------------------------------------------
    def put(self, y, x, text, attr=0):
        h, w = self.scr.getmaxyx()
        if 0 <= y < h and x < w - 1:
            self.scr.addstr(y, x, text[:w - 1 - x], attr)

    def draw(self):
        s = self.state
        self.scr.erase()
        h, w = self.scr.getmaxyx()
        if h < 22 or w < 78:
            self.put(0, 0, f'Make the terminal at least 78 x 22 (now {w} x {h}). q quits.')
            return

        bold = curses.A_BOLD
        self.put(0, 0, ' SECON27 RACE SIM ', bold | curses.A_REVERSE)
        self.put(0, 20, 'control panel for ./simulation/run_sim.sh', curses.A_DIM)

        with s.lock:
            sim_time, paused, rtf = s.sim_time, s.paused, s.rtf
            layout = dict(s.layout) if s.layout else None
            robot, speed = s.robot, s.robot_speed

        # Sim line
        if not self.connected():
            self.put(2, 1, 'Sim', bold)
            self.put(2, 9, 'NOT RUNNING. Start it in another terminal: ./simulation/run_sim.sh',
                     self.color('red') | bold)
        else:
            self.put(2, 1, 'Sim', bold)
            self.put(2, 9, 'PAUSED ' if paused else 'running', self.color('yellow' if paused else 'green') | bold)
            self.put(2, 18, f'time {fmt_time(sim_time)}   real-time factor {rtf:.2f}')

        # Match line
        flag = (layout or {}).get('start_flag', '?')
        self.put(3, 1, 'Match', bold)
        self.put(3, 9, f'flag {flag.upper():4}', self.color('green' if flag == 'up' else 'yellow') | bold)
        if self.match_start is None or sim_time is None:
            self.put(3, 20, 'timer --:-- / 3:00   (raise the flag to start)')
        else:
            end = self.match_end if self.match_end is not None else sim_time
            elapsed = end - self.match_start
            left = MATCH_SECONDS - elapsed
            attr = self.color('red') | bold if left < 0 else (self.color('yellow') if left < 30 else 0)
            state = 'stopped' if self.match_end is not None else 'left'
            self.put(3, 20, f'timer {fmt_time(elapsed)} / 3:00   {fmt_time(left)} {state}', attr)

        # Field lines
        self.put(4, 1, 'Field', bold)
        if layout is None:
            msg = ('field manager not running' if self.connected() else 'waiting for the sim')
            self.put(4, 9, msg, self.color('red'))
        else:
            settings = layout.get('settings', {})
            forced = settings.get('start_tag', 0)
            how = f'forced to {forced}' if forced else f'die rolled {layout.get("die_roll")}'
            seed = 'default' if layout.get('seed') is None else layout['seed']
            if layout.get('default_world_layout'):
                how = 'world file default'
            self.put(4, 9, f'seed {seed:<7} start tag ID {layout["start_tag_id"]} ({how})'
                           f'  ->  PIT BOX {layout["assigned_pit"]}')
            cars = '  '.join(f'{o["name"].replace("obstacle_", "")} Q{o["quadrant"]} {o["lane"]}'
                             for o in layout['obstacles']) or 'none'
            jitter = '  tolerance jitter ON' if layout.get('tolerance_jitter') else ''
            self.put(5, 9, f'obstacles {len(layout["obstacles"])}: {cars}{jitter}')

        # Robot line
        self.put(6, 1, 'Robot', bold)
        if robot is None:
            self.put(6, 9, 'no pose yet')
        else:
            x, y, yaw = robot
            where = zone(x, y)
            attr = self.color('red') | bold if where == 'OFF THE FIELD' else self.color('cyan')
            self.put(6, 9, f'x {x:+.3f} m  y {y:+.3f} m  heading {math.degrees(yaw):+4.0f} deg'
                           f'  speed {speed:.2f} m/s')
            self.put(7, 9, f'in: {where}', attr)

        # Menu
        top = 9
        col_w = max(26, (w - 2) // 3)
        for c, (title, items) in enumerate(MENU):
            x = 1 + c * col_w
            self.put(top, x, title, bold | curses.A_UNDERLINE)
            for i, (key, label) in enumerate(items):
                self.put(top + 1 + i, x, f'[{key}]', self.color('cyan') | bold)
                self.put(top + 1 + i, x + 4, label[:col_w - 5])

        # Drive panel
        y = top + 7
        if self.drive is not None:
            lin, ang = self.drive
            self.put(y, 1, 'DRIVE MODE', self.color('yellow') | bold | curses.A_REVERSE)
            self.put(y, 13, f'speed {lin:+.2f} m/s  turn {ang:+.2f} rad/s   '
                            'W/S faster/slower  A/D turn  SPACE stop  Q/Esc exit')
            self.twist(lin, ang)
            y += 1

        # Log
        self.put(y, 1, 'Log', bold)
        for i, line in enumerate(self.log):
            self.put(y + 1 + i, 3, line, curses.A_DIM if i < len(self.log) - 1 else 0)

    def run(self):
        self.note('Ready. Press a key from the menu.')
        while True:
            self.update_timer()
            self.draw()
            self.scr.refresh()
            key = self.scr.getch()
            if key == -1 or key == curses.KEY_RESIZE:
                continue
            if not self.handle(key):
                break
        if self.drive is not None:
            self.twist(0.0, 0.0)


def main():
    try:
        curses.wrapper(lambda scr: SimTui(scr).run())
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
