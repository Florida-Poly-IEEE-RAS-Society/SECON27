"""Single source of truth for the SoutheastCon 2027 Stock Car Race field.

Every dimension here comes from the Game Manual (Rev 3.0, 10/9/2026) or the
Arena Build Guide (AutoCAD drawing dated 6/12/2026). Values are in INCHES and
converted to meters with `m()`. Anything that the documents do not pin down is
marked ASSUMPTION so it is easy to find and fix after a Q&A answer.

World frame (same as the original sim):
    origin  = center of the board, top surface is z = 0
    +x      = across the 48 in width. The start box is on the +x side.
    +y      = along the 96 in length, away from the pit area
    pit area is the bottom 28 in of the board (y < -20 in)

Used by:
    generate_field.py  writes worlds/track.sdf and the models/ folder
    field_manager.py   randomizes obstacles and the start AprilTag at runtime
"""

import math

INCH = 0.0254


def m(inches):
    """Inches to meters."""
    return inches * INCH


# --------------------------------------------------------------------------
# Board and walls
# --------------------------------------------------------------------------
BOARD_W = 48.0              # x
BOARD_L = 96.0              # y
BOARD_T = 23.0 / 32.0       # 23/32 in RTD sheathing

WALL_T = 1.0                # 1 x 4 in furring strip (nominal)
WALL_H = 4.0
WALL_IN_X = BOARD_W / 2 - WALL_T    # inner face of the long walls: 23 in
WALL_IN_Y = BOARD_L / 2 - WALL_T    # inner face of the end walls:  47 in

# Pit / racecourse divider: 1 x 1 x 16 in rail, centered, 28 in from the pit end.
# Leaves a 15 in opening on each side (15 + 16 + 15 + 2 x 1 wall = 48).
DIVIDER_Y = -BOARD_L / 2 + 28.0 + 0.5    # -19.5 in (rail center)
DIVIDER_LEN = 16.0
DIVIDER_W = 1.0
DIVIDER_H = 1.0

# Racecourse center rail: 1 x 1 x 36 in. Starts 15 in past the divider and
# ends 16 in from the far wall (15 + 36 + 16 + 1 = 68 in racecourse).
RAIL_LEN = 36.0
RAIL_W = 1.0
RAIL_H = 1.0
RAIL_Y0 = DIVIDER_Y + 0.5 + 15.0        # -4 in   (south end)
RAIL_Y1 = RAIL_Y0 + RAIL_LEN            # 32 in   (north end)
RAIL_YC = (RAIL_Y0 + RAIL_Y1) / 2       # 14 in

# --------------------------------------------------------------------------
# Painted / taped markings (visual only)
# --------------------------------------------------------------------------
LINE_W = 1.0                # 1 in white lane line

# Octagon lane line splitting the inner and outer lane. From the drawing:
# long edges 36 3/4 in (parallel to the rail), end edges 12 in, chamfer
# edges 9 3/4 in. The chamfer angle is not dimensioned; 6 in across and
# 7.5 in along gives 9.6 in, which matches the drawing to within tolerance.
OCT_LONG = 36.75
OCT_END = 12.0
OCT_CHAMFER_X = 6.0         # ASSUMPTION (see above)
OCT_CHAMFER_Y = 7.5         # ASSUMPTION (see above)
OCT_CX = 0.0
OCT_CY = RAIL_YC


def octagon_vertices():
    """Lane-line centerline vertices (inches), counter-clockwise."""
    hx_long = OCT_END / 2 + OCT_CHAMFER_X   # x of the long edges: 12 in
    hy_long = OCT_LONG / 2                  # long edge half length
    hx_end = OCT_END / 2
    hy_end = OCT_LONG / 2 + OCT_CHAMFER_Y   # y of the end edges
    pts = [
        (hx_long, -hy_long), (hx_long, hy_long),
        (hx_end, hy_end), (-hx_end, hy_end),
        (-hx_long, hy_long), (-hx_long, -hy_long),
        (-hx_end, -hy_end), (hx_end, -hy_end),
    ]
    return [(OCT_CX + x, OCT_CY + y) for x, y in pts]


# Lane split used for obstacle placement: |x| of the lane line.
LANE_LINE_X = OCT_END / 2 + OCT_CHAMFER_X   # 12 in

# 12 x 12 in white boxes. 2 in margins at the walls, 3 in gaps (manual / field.md).
PIT_BOX = 12.0
PIT_GAP = 3.0
PIT_MARGIN = 2.0
# Pit boxes sit against the pit end wall, numbered from the start-box side.
# ASSUMPTION: Pit 1 is the box nearest the start box (Build Guide p.12 render
# reads Start Box, Pit 1, Pit 2, Pit 3 left to right). Confirm on Discord.
PIT_BOX_Y = -WALL_IN_Y + PIT_BOX / 2                        # -41 in
PIT_BOX_X = {
    1: WALL_IN_X - PIT_MARGIN - PIT_BOX / 2,                # +15 in
    2: 0.0,
    3: -(WALL_IN_X - PIT_MARGIN - PIT_BOX / 2),             # -15 in
}

# Start box: 12 x 12 in, against the +x wall, between pit box 1 and the divider.
START_BOX_X = PIT_BOX_X[1]
START_BOX_Y = -WALL_IN_Y + PIT_BOX + PIT_GAP + PIT_BOX / 2 - 0.5   # -26.5 in

# Checkered start / finish strips across both 15 in openings, 2 squares deep.
CHECKER = 1.0
CHECKER_ROWS = 2
OPENING_X0 = DIVIDER_LEN / 2                # 8 in
OPENING_X1 = WALL_IN_X                      # 23 in

# --------------------------------------------------------------------------
# AprilTags  (Game Manual Appendix C, Build Guide p.2 and p.12)
# --------------------------------------------------------------------------
TAG_FAMILY = 'tagStandard41h12'
TAG_IDS = (1, 2, 3)
TAG_PRINT = 68.0 / 25.4      # full 9 x 9 cell print width, 68 mm
TAG_CARD = 4.0               # Build Guide: tags are ~4 x 4 in, laminated
TAG_CARD_T = 0.08            # card + magnet, in
TAG_Z = WALL_H / 2           # card centered on the 4 in wall
# For pose estimation the detector's tag_size is the black border edge,
# which on 41h12 is 5 of the 9 cells: 68 mm * 5/9 = 37.8 mm.
TAG_SIZE_FOR_DETECTOR_M = 0.068 * 5.0 / 9.0

# Start-area tag slot: on the +x long wall, facing into the pit area,
# 17 to 19 in from the pit end wall (AutoCAD drawing).
START_TAG_X = WALL_IN_X
START_TAG_Y = -WALL_IN_Y + 18.0
START_TAG_YAW = math.pi              # card normal points -x

# One fixed tag per pit box on the pit end wall (Build Guide p.2 legend,
# "April Tags Pit Locations"). Tag ID n sits above pit box n.
PIT_TAG_Y = -WALL_IN_Y
PIT_TAG_YAW = math.pi / 2            # card normal points +y

# Bit grids for IDs 1-3 (1 = black), read from the Game Manual tag sheet and
# verified with the AprilTag detector (tagStandard41h12, hamming 0).
TAG_BITS = {
    1: [
        '..#....#.',
        '#########',
        '.#.....##',
        '##.....##',
        '.#.#...#.',
        '.#..#..##',
        '##.....##',
        '.#######.',
        '#.#...##.',
    ],
    2: [
        '..#....#.',
        '.#######.',
        '.#.....#.',
        '##...#.#.',
        '##..##.#.',
        '.#.#...#.',
        '##.....#.',
        '########.',
        '#.#.#.###',
    ],
    3: [
        '..#.....#',
        '.#######.',
        '.#.....#.',
        '##.###.#.',
        '.#...#.##',
        '##.....#.',
        '##.....##',
        '.########',
        '..#..###.',
    ],
}

# --------------------------------------------------------------------------
# Obstacles  (Game Manual 3.1 / 3.4.1 / Appendix E, Build Guide p.11)
# --------------------------------------------------------------------------
# Melissa & Doug pull-back plush vehicles: 8.55 L x 8.2 W x 4.5 H in.
CAR_L = 8.55
CAR_W = 8.2
CAR_H = 4.5
CAR_MASS = 0.2               # kg, ASSUMPTION
CAR_COLORS = {               # the four cars in the set
    'blue':   (0.15, 0.35, 0.85),
    'orange': (0.95, 0.50, 0.10),
    'red':    (0.85, 0.12, 0.12),
    'yellow': (0.98, 0.82, 0.15),
}

# Game Manual Rev 3.0: "There will be 3 randomly placed obstacles on the
# course ... placed in different lanes." At least one inner and one outer.
DEFAULT_NUM_OBSTACLES = 3

# Appendix E quadrants: 16 in wide, from 2 in off the side wall toward the
# rail; 16 in long from each end of the rail, 4 in gap in the middle.
# field_manager.py uses the quadrant's y range and side; across the lane it
# keeps the whole car inside its lane, like the Appendix E photo.
QUAD_W = 16.0
QUAD_LEN = 16.0
_QX1 = WALL_IN_X - 2.0                  # 21 in
_QX0 = _QX1 - QUAD_W                    # 5 in
_QS = (RAIL_Y0, RAIL_Y0 + QUAD_LEN)     # south half of the straight
_QN = (RAIL_Y1 - QUAD_LEN, RAIL_Y1)     # north half

# Numbering follows the Appendix E photo (taken from the far end, pit at
# the top): Q1/Q2 on the +x straight, Q3/Q4 on the -x straight, Q1/Q3 near
# the pit.
QUADRANTS = {
    1: {'x': (_QX0, _QX1), 'y': _QS, 'side': +1},
    2: {'x': (_QX0, _QX1), 'y': _QN, 'side': +1},
    3: {'x': (-_QX1, -_QX0), 'y': _QS, 'side': -1},
    4: {'x': (-_QX1, -_QX0), 'y': _QN, 'side': -1},
}

# Game Manual <G02>: field elements may be off by up to +/- 1 in each match.
TOLERANCE = 1.0

# Match: 3:00 from the green flag (Game Manual). A round (run_sim.sh
# --start-round, the control panel, the batch test) raises the flag
# ROUND_FLAG_DELAY sim seconds after it's requested and pauses the sim
# MATCH_SECONDS after the flag.
MATCH_SECONDS = 180.0
ROUND_FLAG_DELAY = 3.0

# --------------------------------------------------------------------------
# Start flag  (Appendix F: "Details will be provided in Version 3.1")
# --------------------------------------------------------------------------
# ASSUMPTION: Build Guide says the pole sits ~5 in from the divider on a tower,
# and the AutoCAD legend draws it at the start-side opening. Modeled as a
# tower on the +x end of the divider with an arm that is horizontal (down,
# lying along the divider) before the start and vertical (up) at the start.
FLAG_TOWER_X = DIVIDER_LEN / 2 - 1.0
FLAG_TOWER_Y = DIVIDER_Y
FLAG_TOWER_H = 6.0
FLAG_ARM_LEN = 7.0

# --------------------------------------------------------------------------
# Paint (Build Guide p.18)
# --------------------------------------------------------------------------
ONYX = (0.07, 0.07, 0.07)           # PPG1011-7: board and pit walls
WINTER_CHILL = (0.78, 0.82, 0.83)   # PPG1036-2: racecourse walls, rails
DELICATE_WHITE = (0.93, 0.92, 0.88)  # PPG1001-1: pit and start boxes
TAPE_WHITE = (0.97, 0.97, 0.97)     # 1 in white duct tape lane line

# --------------------------------------------------------------------------
# Poses shared by the generator and the runtime field manager
# (x, y, z in meters; roll, pitch, yaw in radians)
# --------------------------------------------------------------------------
def flag_pose(up):
    x, y = m(FLAG_TOWER_X), m(FLAG_TOWER_Y)
    z = m(DIVIDER_H + FLAG_TOWER_H)
    return (x, y, z, 0.0, (math.pi / 2) if up else 0.0, 0.0)


def tag_pose_start():
    t = m(TAG_CARD_T)
    return (m(START_TAG_X) - t, m(START_TAG_Y), m(TAG_Z), 0, 0, START_TAG_YAW)


def tag_pose_pit(n):
    t = m(TAG_CARD_T)
    return (m(PIT_BOX_X[n]), m(PIT_TAG_Y) + t, m(TAG_Z), 0, 0, PIT_TAG_YAW)


def tag_pose_parked(n):
    """Hidden below the floor."""
    return (m(-10.0 * n), m(-20.0), -0.5, 0, 0, 0)


def car_pose_parked(i):
    """Off the board, sitting on the floor next to the table."""
    return (0.95, -0.6 + 0.35 * i, -m(BOARD_T) + 0.001, 0, 0, 0)



# Fixed layout written into worlds/track.sdf. field_manager.py replaces it at
# startup, unless run with --keep (launch option randomize:=false).
DEFAULT_START_TAG = 1
DEFAULT_CARS = [            # (color, quadrant, lane), each car centered in its lane
    ('blue', 1, 'inner'),
    ('red', 2, 'outer'),
    ('orange', 4, 'outer'),
]


def default_car_pose(color):
    _, q, lane = next(c for c in DEFAULT_CARS if c[0] == color)
    quad = QUADRANTS[q]
    if lane == 'inner':
        ax = (RAIL_W / 2 + LANE_LINE_X - LINE_W / 2) / 2        # 6 in
    else:
        ax = (LANE_LINE_X + LINE_W / 2 + WALL_IN_X) / 2         # 17.75 in
    y = (quad['y'][0] + quad['y'][1]) / 2
    yaw = math.pi / 2 if quad['side'] > 0 else -math.pi / 2
    return (m(quad['side'] * ax), m(y), 0.002, 0, 0, yaw)


# Robot footprint (models/robot/robot.urdf.xacro, stock-car body).
# base_link is the body center at axle height; the footprint center sits
# ROBOT_CENTER_X ahead of it.
ROBOT_FRONT = 0.135          # m ahead of base_link (front splitter)
ROBOT_REAR = -0.126          # m (rear spoiler)
ROBOT_HALF_W = 0.1175        # m, including the tires
ROBOT_CENTER_X = (ROBOT_FRONT + ROBOT_REAR) / 2
ROBOT_WHEEL_R = 0.035

# Robot spawn: footprint centered in the start box, facing the racecourse
# (+y). base_link is placed so the footprint center lands on the box center.
# (x m, y m, z m, yaw rad)
ROBOT_SPAWN = (m(START_BOX_X), m(START_BOX_Y) - ROBOT_CENTER_X, ROBOT_WHEEL_R + 0.01,
               math.pi / 2)


def robot_spawn(turn_deg=0.0):
    """Start-box spawn pose (base_link), turned turn_deg to the left.

    The robot turns about its footprint center, so it stays centered in the
    start box. <G08> lets the technician set the robot in any orientation
    inside the start cube.
    """
    yaw = math.pi / 2 + math.radians(turn_deg)
    cx, cy = m(START_BOX_X), m(START_BOX_Y)
    return (cx - ROBOT_CENTER_X * math.cos(yaw), cy - ROBOT_CENTER_X * math.sin(yaw),
            ROBOT_SPAWN[2], yaw)


def robot_center(x, y, yaw):
    """Footprint center from a base_link pose."""
    return x + ROBOT_CENTER_X * math.cos(yaw), y + ROBOT_CENTER_X * math.sin(yaw)
