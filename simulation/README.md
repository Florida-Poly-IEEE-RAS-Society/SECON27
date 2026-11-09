# Simulation

Gazebo simulation of the SoutheastCon 2027 Stock Car Race field and robot. ROS 2 nodes in `ros2/src` control the robot.

The field follows the Game Manual (Rev 3.0, 10/9/2026) and the Arena Build Guide's AutoCAD drawing (6/12/2026). Each launch randomizes the obstacles and the start AprilTag the way the referees do before a match, and you can re-randomize while the sim is running.

## Quick start

You need Ubuntu 24.04 with [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). This works on native Linux, a VM, or WSL.

```bash
cd ~/SECON27
./simulation/run_sim.sh --install   # once: install the Gazebo packages (asks for sudo)
./simulation/run_sim.sh             # start the sim
./simulation/run_sim.sh --tui       # second terminal: control panel
./simulation/run_sim.sh --stop      # when you're done
```

## Commands

Everything goes through one script, `simulation/run_sim.sh`:

| Command | What it does |
|---|---|
| `./simulation/run_sim.sh` | Start the sim with the Gazebo window |
| `./simulation/run_sim.sh --no-gui` | Start the sim without the window (faster) |
| `./simulation/run_sim.sh --tui` | Control panel for a running sim (second terminal) |
| `./simulation/run_sim.sh --stop` | Stop everything the sim started |
| `./simulation/run_sim.sh --batch ...` | Test your robot code on many random fields, headless |
| `./simulation/run_sim.sh --install` | Install missing packages (once) |
| `./simulation/run_sim.sh --help` | Show this list |

Only one sim can run at a time. If one is already running, the script tells you to stop it first.

### Start the sim

```bash
./simulation/run_sim.sh
```

Gazebo opens with a random field. The robot sits in the start box facing the racecourse. Add launch options after the command, in any combination (see [Launch options](#launch-options)):

```bash
./simulation/run_sim.sh seed:=42 obstacles:=2
```

To stop, press Ctrl+C in that terminal, close the Gazebo window, or run `--stop`.

### `--no-gui`: no window

```bash
./simulation/run_sim.sh --no-gui
./simulation/run_sim.sh --no-gui seed:=42      # launch options still work
```

Runs only the Gazebo server, with no 3D window. Everything else works the same: the cameras, the IMU, ROS topics and the control panel. It runs faster, since drawing the window takes about 2.5 CPU cores on a machine without a GPU. Use it when your robot code or the control panel is all you need to see.

On a machine with no display at all (plain SSH, no desktop), the cameras use offscreen rendering. That needs a working GPU driver; on a VM without one, run it from the VM's desktop or remote-desktop session instead.

### `--tui`: control panel

With the sim running, open a second terminal:

```bash
./simulation/run_sim.sh --tui
```

The control panel shows the sim clock, the start flag and a 3:00 match timer, the field layout (seed, start tag, assigned pit box, where each car is), and the robot's position, including which zone it's in: start box, pit box 1–3, inner or outer lane, or off the field. Press a key to run a command:

| Key | Command |
|---|---|
| **Match** | |
| `1` | Raise the start flag. The match timer starts, in sim time. |
| `2` | Lower the start flag |
| `3` | Reset match: robot back to the start box and stopped, flag down, timer cleared. The field stays the same. |
| `4` | New match: same as reset, plus a new random field |
| **Field** | |
| `5` | New random field |
| `6` | New field from a seed you type, for repeatable tests |
| `7` | Start tag: `0` rolls a die, or `1` / `2` / `3` always uses that tag |
| `8` | Number of obstacles, 0–4 |
| `9` | Tolerance jitter on/off (tags shift up to ±1 in) |
| **Robot / sim** | |
| `d` | Drive the robot. W/S speed up and slow down, A/D turn, Space stops, Q or Esc leaves drive mode. |
| `x` | Stop the robot |
| `b` | Robot back to the start box (the field and flag are unchanged) |
| `p` | Pause / resume the simulation |
| `q` | Quit the control panel. The sim keeps running. |

Settings from keys 7, 8 and 9 stay in effect for later fields (keys 4, 5 and 6) until you change them.

Things to know:

- **`/odom` does not reset.** Keys 3, 4 and `b` move the robot back to the start box, so `/odom` jumps. For a fully clean start, restart the sim. Gazebo's built-in world reset is not used, because it deletes the spawned robot.
- **Drive mode publishes `/cmd_vel`.** Stop your own controller first, or the two will fight.
- **The terminal must be at least 78 x 22.**

### `--stop`: stop everything

```bash
./simulation/run_sim.sh --stop
```

Stops the sim and everything it started: Gazebo (server and window), the ROS nodes from the launch, the ROS–Gazebo bridge, the field manager, any open control panel and any running batch test. It asks each one to shut down cleanly first (like Ctrl+C), and force-stops anything that hangs. If nothing is running, it says so.

Use it when a sim was left running in another terminal, or after a crash leaves Gazebo half-open.

### `--batch`: test robot code on many fields

```bash
./simulation/run_sim.sh --batch --runs 10 \
    --cmd "ros2 run my_pkg race_node --ros-args -p use_sim_time:=true"
```

Runs your robot code once per random field, each in a fresh sim with no window (so `/odom` starts at zero every time). For each run it starts the sim, starts your command, raises the start flag after 3 sim seconds, watches for one match (3:00 of sim time) or until your command exits, then shuts everything down and moves on.

It doesn't score runs. It reports, per run:

| Column | Meaning |
|---|---|
| Early start | The robot moved before the flag went up (yellow card under rule G05) |
| Left box (s) | Sim seconds after the flag until the robot left the start box |
| Distance (m) | How far the robot drove |
| Laps ~ | Rough lap count, from how far the robot circled the center rail |
| Final zone | Where it ended: start box, pit box 1–3, inner/outer lane, ... |
| Cars hit | Cars that moved. They're physical objects, so a touch moves them. |
| Off field | The robot left the board |
| Stalled | The robot didn't move in the last 10 sim seconds |

Results go to `simulation/test_results/<date-time>/`: `summary.md` (the table), `results.json` (with each run's full field layout), and a `seed_<N>/` folder per run with `sim.log` and `robot.log`. To rerun a field that failed, pass its seed: `--seeds 1234`, or open it in the full sim with `./simulation/run_sim.sh seed:=1234`.

| Option | Default | Effect |
|---|---|---|
| `--runs N` | `5` | Number of random fields |
| `--seeds 1,2,10-15` | | Exact seeds instead of random ones |
| `--cmd "..."` | none | Command that starts your robot code. Without it, the batch just checks that the sim starts. |
| `--duration S` | `180` | Sim seconds to watch after the flag |
| `--flag-delay S` | `3` | Sim seconds between your code starting and the flag |
| `--obstacles N`, `--start-tag N`, `--start-turn DEG`, `--tolerance` | | Same as the launch options |
| `--gui` | off | Show the Gazebo window (slower) |
| `--out DIR` | `simulation/test_results/<date-time>` | Where to write results |

### `--install`

```bash
./simulation/run_sim.sh --install
```

Installs `ros-jazzy-ros-gz` and `python3-pil` if they're missing. ROS 2 Jazzy itself must already be installed. If packages are missing, the other commands tell you to run this.

## Launch options

Pass these after `./simulation/run_sim.sh` or `--no-gui`:

| Option | Default | Effect |
|---|---|---|
| `seed:=N` | `-1` | Layout seed. The same seed always gives the same field. `-1` picks a random seed. |
| `obstacles:=N` | `3` | Plush cars on the course, 0 to 4 |
| `start_tag:=N` | `0` | Force the start AprilTag to ID 1, 2 or 3. `0` rolls a die like the referees. |
| `tolerance:=true` | `false` | Shift each tag up to ±1 in (field tolerance, rule G02) |
| `interval:=S` | `0` | New layout every S seconds. `0` turns it off. |
| `randomize:=false` | `true` | Keep the world file's fixed layout at startup |
| `start_turn:=DEG` | `0` | Robot start heading, degrees left of facing the racecourse. See [Seeing the start flag](#seeing-the-start-flag). |
| `gui:=false` | `true` | Same as `--no-gui` |

## Topics

| Topic | Type | Use |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Drive the robot |
| `/camera/image_raw` | `sensor_msgs/Image` | OAK-D Lite color image |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera calibration (color and depth share it) |
| `/camera/depth_image` | `sensor_msgs/Image` | Depth, 32FC1 meters, aligned to the color image |
| `/camera/points` | `sensor_msgs/PointCloud2` | Point cloud from the depth |
| `/camera/imu` | `sensor_msgs/Imu` | OAK-D Lite IMU, 200 Hz |
| `/odom` | `nav_msgs/Odometry` | Robot position estimate (perfect wheel odometry, no slip) |
| `/field/layout` | `std_msgs/String` | Field ground truth (JSON) |
| `/field/randomize` | `std_msgs/Int32` | Request a new layout |
| `/field/start_flag` | `std_msgs/Bool` | Raise or lower the start flag |

Set `use_sim_time:=true` on every ROS 2 node that talks to the simulation.

## Changing the field from ROS 2

The control panel is the easy way. To script it, or to drive it from your own nodes, use these topics. Source ROS first:

```bash
source /opt/ros/jazzy/setup.bash

# New layout: random seed, or a fixed seed for a repeatable test
ros2 topic pub --once /field/randomize std_msgs/msg/Int32 "{data: -1}"
ros2 topic pub --once /field/randomize std_msgs/msg/Int32 "{data: 42}"

# Raise the start flag (race start), or lower it
ros2 topic pub --once /field/start_flag std_msgs/msg/Bool "{data: true}"

# Current layout as JSON: assigned pit, die roll, obstacle quadrants, lanes and poses
ros2 topic echo /field/layout
```

`/field/layout` is ground truth for testing and scoring your code. The robot can't read it in a real match, so robot code must not use it.

## How the field is randomized

`scripts/field_manager.py` runs with the sim and follows the Game Manual's pre-match rules:

- **AprilTag.** A die roll picks the start-area tag: 1 or 4 gives ID 1, 2 or 5 gives ID 2, 3 or 6 gives ID 3. The chosen tag goes in the start-area slot and the other two are hidden. The tag ID is the robot's assigned pit box.
- **Obstacles.** Each car goes in a different Appendix E quadrant on the straightaways, with at least one car in the inner lane and one in the outer lane. The position inside the lane and the heading (±5°) are random. Cars not in use sit on the floor next to the board.
- **Start flag.** Goes back down.

The obstacles are dynamic bodies (0.2 kg), so a robot that hits one pushes it, like the real plush cars.

## Camera: OAK-D Lite

The robot carries a simulated Luxonis OAK-D Lite, matched to the [published specs](https://docs.luxonis.com/hardware/products/OAK-D%20Lite):

| | Real OAK-D Lite | Sim |
|---|---|---|
| Body | 91 x 28 x 17.5 mm, 61 g | Same |
| Color camera | IMX214, 69 x 54 deg FOV | 69 deg HFOV, 640 x 480, 30 Hz |
| Stereo depth | 2x OV7251, 75 mm baseline | Aligned to the color image (DepthAI `setDepthAlign(CAM_A)`): same 640 x 480 grid and `camera_info` |
| Minimum depth | ~0.20 m with extended disparity, ~0.40 m without | 0.20 m. Anything closer reads as no depth. |
| Maximum depth | Ideal range to ~12 m | 12 m |
| IMU | Bosch BMI270, raw accel + gyro | 200 Hz, datasheet noise (see below) |

The camera is fixed, on a mast at the **rear** of the robot, level and facing forward (`camera_joint` in `models/robot/robot.urdf`). Two things follow from that placement:

- The robot's hood shows as a thin strip along the bottom edge of the image. Crop the bottom few rows in your vision code.
- With no downward tilt, the floor is visible from about 9 in ahead of the robot's nose. Lane lines right at the bumper are out of view.

The real sensor is 13 MP. The sim uses 640 x 480 so it runs in real time; raise `width`/`height` in the URDF to match your DepthAI pipeline if you need more detail.

### IMU

`/camera/imu` simulates the OAK-D Lite's [BMI270](https://bosch-sensortec.com/products/motion-sensors/imus/bmi270.html), in frame `imu_link` (x forward, y left, z up):

- **200 Hz.** DepthAI tops out around 250 Hz on this chip.
- **Noise from the Bosch datasheet:** gyro 0.007 °/s/√Hz and accelerometer 160 µg/√Hz, which is about 0.0017 rad/s and 0.022 m/s² per axis at 200 Hz. Each run also gets a small random bias, as on a calibrated sensor.
- **Gravity is included.** At rest, accel z reads about +9.81 m/s², like the real chip.
- **No orientation.** Like the real BMI270, it gives raw accel and gyro only. The orientation field in the ROS message is all zeros; ignore it, and integrate the gyro or fuse it yourself.

On the real camera the chip's axes differ from `imu_link`; the DepthAI ROS driver rotates them. Check your driver's frame before reusing sim-tuned code. Early Kickstarter OAK-D Lite units have no IMU at all.

## Seeing the start flag

The start flag tower sits in front of and to the left of the start box, so a camera centered at the front of the robot can't see it from the start box, flag up or down. Two placements were tested in the sim with the robot's own camera:

| Setup | Flag down | Flag up | Start trigger |
|---|---|---|---|
| **Rear mast, level, robot facing straight (current)** | Not in view (0 green pixels) | Green flag in the top-left corner (~4,900 green pixels) | Green appears in the top-left |
| Front mast tilted 20 deg down, robot turned 45-60 deg left (`start_turn:=45`) | Green flag at the top edge | Out of view | Green disappears; then the robot has to turn back |

The current setup was chosen because the robot leaves the start box without turning, and the camera stays forward-facing for obstacles.

**Caveat.** Appendix F (Start Flag Operations) is not published yet ("Details will be provided in Version 3.1"), so the flag tower position in this sim is an educated guess from the Build Guide. The green flag only just fits in the corner of the frame, so a few inches of difference on the real field, or the ±1 in tolerance, could move it out of view. Re-test once Appendix F is out, and keep the start button (no flag) as a backup. Starting without the flag only loses the 10-point green-flag bonus.

## AprilTags

| Property | Value |
|---|---|
| Family | `tagStandard41h12` |
| IDs | 1, 2, 3 |
| Print size | 68 mm across all 9 x 9 cells, on a white 4 x 4 in card |
| `tag_size` for pose estimation | 0.0378 m (black border edge, 5 of the 9 cells) |
| Start-area tag | On the +x wall beside the start box, facing into the pit area. Changes each layout. |
| Pit tags | One fixed tag on the pit end wall above each pit box. ID n is above pit box n. |

The tag images come straight from the Game Manual's tag sheet. A real AprilTag detector reads them at hamming distance 0 from the robot camera in the sim.

## Field coordinates

Origin is the center of the board, with the board's top surface at z = 0. +x points across the 48 in width toward the start box side. +y points along the 96 in length, away from the pit area.

## Files

| File | Contents |
|---|---|
| `run_sim.sh` | The one entry point: start, `--no-gui`, `--tui`, `--stop`, `--batch`, `--install` |
| `launch/sim.launch.py` | Starts Gazebo, spawns the robot, runs the field manager, connects topics to ROS 2 |
| `scripts/field_spec.py` | Every field dimension, in inches, with its source |
| `scripts/generate_field.py` | Builds `worlds/track.sdf` and the tag and car models from `field_spec.py` |
| `scripts/field_manager.py` | Randomizes obstacles, start tag and flag at runtime |
| `scripts/sim_tui.py` | Control panel (`--tui`) |
| `scripts/batch_test.py` | Batch tester (`--batch`) |
| `worlds/track.sdf` | The field (generated, do not edit by hand) |
| `models/apriltag_41h12_<id>/` | Tag card models (generated) |
| `models/plush_car_<color>/` | Obstacle car models (generated) |
| `models/robot/robot.urdf` | The robot, its OAK-D Lite camera and IMU |
| `config/bridge.yaml` | List of topics shared with ROS 2 |

To change the field, edit `scripts/field_spec.py` and regenerate:

```bash
python3 simulation/scripts/generate_field.py
```

## Troubleshooting

- **"A Gazebo simulation is already running".** Run `./simulation/run_sim.sh --stop`.
- **Field commands do nothing.** Make sure only one sim is running, and that you ran `source /opt/ros/jazzy/setup.bash` in that terminal.
- **Gazebo crashes on start on native Linux or a VM.** The script already unsets `QT_QUICK_BACKEND` and `LIBGL_ALWAYS_SOFTWARE`. Check that nothing in your shell sets them again. On WSL the script leaves them alone.
- **The sim runs slower than real time.** Use `--no-gui`. The control panel's "real-time factor" shows the speed (1.00 = real time).
- **A batch test was interrupted.** Ctrl+C or `--stop` both shut down the current run's sim and your robot command cleanly, and the summary keeps the runs that finished.

### WSL

- **Clone the repo inside WSL** (for example `~/SECON27`), not under `/mnt/c/...`. Windows folders are much slower from WSL and can lose the scripts' executable permission. Folder names with spaces do work.
- **If you clone with Git for Windows**, `simulation/.gitattributes` keeps the sim's files on Unix line endings. If you still see `bash\r: No such file or directory`, your copy has Windows line endings; re-clone from inside WSL.
- **Graphics.** `run_sim.sh` detects WSL and keeps `LIBGL_ALWAYS_SOFTWARE` and `QT_QUICK_BACKEND`, which WSL needs (the batch test does the same). On native Linux it removes them, because they crash Gazebo there. WSL needs WSLg (Windows 11, or Windows 10 with WSLg installed) for the Gazebo window and cameras.

## Known gaps

Values marked `ASSUMPTION` in `scripts/field_spec.py` are not pinned down by the documents. Check them on the Discord Q&A:

- **Pit box numbering.** Pit 1 is assumed to be the box next to the start box.
- **Start flag.** Appendix F says details come in Version 3.1. The flag is modeled as a tower on the start-side end of the divider, with an arm that swings up. The camera placement depends on this; see [Seeing the start flag](#seeing-the-start-flag).
- **Octagon lane line.** The chamfer angle is not dimensioned. The model fits the drawn 9 3/4 in chamfer edges to within the ±1 in tolerance.
- **Obstacle count.** The Game Manual says 3 obstacles. The Build Guide says at least 1 per lane, and `docs/field.md` says 2. The sim defaults to 3; use `obstacles:=2` to match the older docs.
- **Car mass and friction.** Not specified.

Not modeled yet, on purpose:

- **Robot chassis.** `models/robot/robot.urdf` is a placeholder until the real chassis is finished. Swap in its wheelbase, wheel size, mass and motor limits then.
- **Pit stop tires and school flag.** These wait until the mechanisms are designed.
