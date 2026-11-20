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
| `./simulation/run_sim.sh --no-gui` | Start the sim without the window (faster), in the background; returns once it's ready |
| `./simulation/run_sim.sh --start-round` | Start a round on the running sim: flag up in 3 s, sim pauses after the 3:00 match |
| `./simulation/run_sim.sh --reset` | Reset the match: robot back in the start box and stopped, flag down, round ended |
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

### `--no-gui`: no window, in the background

```bash
./simulation/run_sim.sh --no-gui               # returns once the sim is ready
./simulation/run_sim.sh --no-gui seed:=42      # launch options still work
./simulation/run_sim.sh --tui                  # same terminal is fine
./simulation/run_sim.sh --stop                 # when you're done
```

Runs only the Gazebo server, with no 3D window. Everything else works the same: the cameras, the IMU, ROS topics and the control panel. It runs faster, since drawing the window takes about 2.5 CPU cores on a machine without a GPU. Use it when your robot code or the control panel is all you need to see.

With no window to watch, it starts in the background and gives you the terminal back once the sim is ready to use: the field is set, the robot is spawned and its camera is sending frames. It prints where the sim's log goes (a file in `/tmp`). If the sim doesn't come up within 2 minutes, it prints the end of the log, stops everything and exits with an error. Stop the sim with `--stop`. Scripts can run their tests right after it returns, with no guessing how long to wait.

On a machine with no display at all (plain SSH, no desktop), the cameras use offscreen rendering. That needs a working GPU driver; on a VM without one, run it from the VM's desktop or remote-desktop session instead.

Your own scripts can wait the same way on a sim that is already starting: `python3 simulation/scripts/wait_ready.py` exits 0 when the sim is ready, 1 on timeout (`--timeout S`, default 120).

### `--start-round` and `--reset`: run a round

```bash
./simulation/run_sim.sh --no-gui        # or with the window
./simulation/run_sim.sh --start-round   # start your robot code first, then this
# ... the round runs; the sim pauses after 3:00 ...
./simulation/run_sim.sh --reset         # robot back in the start box for the next round
./simulation/run_sim.sh --start-round
```

Runs one round on the sim that's already running, the way the referee does: the start flag goes down (if it was up), goes up 3 sim seconds later, and the sim **pauses** when the 3:00 match is over. Nothing moves after the buzzer, so you can see where the robot ended (`--tui`, topics, the Gazebo window). For the next round, `--reset` puts the robot back in the start box (stopped, flag down, same field), and `--start-round` starts it again (it resumes the sim). `--start-round` doesn't move the robot by itself. `--reset` doesn't reset `/odom`; for a fully clean start, restart the sim. `--reset` stops the robot once, but it can't stop your robot code: if your code keeps publishing `/cmd_vel`, the robot drives off again right away. Stop or reset your own nodes first.

`--start-round` stays in your terminal and reports the round until it ends:

```
Start flag goes up in 3 s ...
GREEN FLAG at sim 40.1 s: the 3:00 match is on.
  2:00 left
  1:00 left
  0:30 left
  0:10 left
ROUND OVER at sim 220.1 s: 3:00 played, sim paused.
```

Ctrl+C only stops the reporting; the round keeps running in the sim. It exits 0 when the round is over, 1 if no sim answered (or the sim went away), and 3 if the round was cancelled: lowering the flag by hand, `--reset`, a new field or another `--start-round` cancels it.

The timing is sim time, so a slow machine still gets a fair 3:00. The control panel shows the same countdown and match timer, and `/field/layout` has it under `"round"` (`state`: `countdown`, `running`, `over`, or `cancelled`). The control panel's `1` key and every `--batch` run start the same round.

### `--tui`: control panel

With the sim running, open a second terminal:

```bash
./simulation/run_sim.sh --tui
```

The control panel shows the sim clock, the start flag and the round's 3:00 match timer, the field layout (seed, start tag, assigned pit box, where each car is), and the robot's position, including which zone it's in: start box, pit box 1–3, inner or outer lane, or off the field. Press a key to run a command:

| Key | Command |
|---|---|
| **Match** | |
| `1` | Start a round, the same as `--start-round`: flag up in 3 s, sim pauses after 3:00 |
| `2` | Lower the start flag (ends the round) |
| `3` | Reset match, the same as `--reset`: robot back to the start box and stopped, flag down, round ended. The field stays the same. |
| `4` | New match: same as reset, plus a new random field |
| **Field** | |
| `5` | New random field |
| `6` | New field from a seed you type, for repeatable tests |
| `7` | Start tag: `0` rolls a die, or `1` / `2` / `3` always uses that tag |
| `8` | Number of obstacles, 0–4 |
| `9` | Tolerance jitter on/off (tags shift up to ±1 in) |
| **Robot / sim** | |
| `d` | Drive the robot. W/S speed up and slow down, A/D steer (only while moving; it's a car), Space stops, Q or Esc leaves drive mode. |
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

Runs your robot code once per random field, each in a fresh sim with no window (so `/odom` starts at zero every time). For each run it starts the sim, starts your command, starts a round (the same one as [`--start-round`](#--start-round-and---reset-run-a-round): flag up after 3 sim seconds, sim paused after the 3:00 match), watches until the round is over or your command exits, then shuts everything down and moves on.

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
| `--duration S` | `180` | Sim seconds from the flag until the round ends (the sim pauses) |
| `--flag-delay S` | `3` | Sim seconds between your code starting and the flag |
| `--obstacles N`, `--start-tag N`, `--start-turn DEG`, `--tolerance` | | Same as the launch options |
| `--gui` | off | Show the Gazebo window (slower) |
| `--out DIR` | `simulation/test_results/<date-time>` | Where to write results |
| `key:=value ...` | | Any other [launch option](#launch-options), for every run. Put them after the batch options. |

For example, to test your code with a different camera mount:

```bash
./simulation/run_sim.sh --batch --runs 5 --cmd "..." cam_x:=-0.11 cam_yaw:=15
```

The launch options are listed in `summary.md` and `results.json`. Options the batch sets itself (`seed`, `gui`, `obstacles`, `start_tag`, `start_turn`, `tolerance`) are refused with a pointer to the batch option to use instead.

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
| `start_turn:=DEG` | `0` | Robot start heading, degrees left of facing the racecourse (rule G08 allows any heading inside the start cube) |
| `gui:=false` | `true` | No window, like `--no-gui`, but the sim stays in the foreground of this terminal |
| `cam_x:=M` | `-0.10` | OAK-D Lite mount: meters forward of the body center |
| `cam_y:=M` | `0` | ... meters left of the centerline |
| `cam_z:=M` | `0.185` | ... lens height above the floor, meters |
| `cam_pitch:=DEG` | `0` | ... tilt, degrees down |
| `cam_yaw:=DEG` | `0` | ... turn, degrees left |

The `cam_*` options move the camera without editing files; see [Camera placement](#camera-placement).

## Topics

| Topic | Type | Use |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Drive the robot: `linear.x` speed (m/s), `angular.z` yaw rate (rad/s). It's a car: no turning in place. |
| `/camera/image_raw` | `sensor_msgs/Image` | OAK-D Lite color image |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera calibration (color and depth share it) |
| `/camera/depth_image` | `sensor_msgs/Image` | Depth, 32FC1 meters, aligned to the color image |
| `/camera/points` | `sensor_msgs/PointCloud2` | Colored point cloud, in `camera_optical_link`, built from the depth image by `depth_image_proc` |
| `/camera/imu` | `sensor_msgs/Imu` | OAK-D Lite IMU, 200 Hz |
| `/odom` | `nav_msgs/Odometry` | Robot position estimate (perfect wheel odometry, no slip), frame `odom` → `base_link` (body center) |
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

# Raise the start flag by hand, or lower it (for a timed round, use --start-round)
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

## Robot

`models/robot/robot.urdf.xacro`: the stock-car body (red, racing stripes, spoiler), driven like the real chassis. The rear wheels drive and the front wheels steer with one servo (Ackermann).

| | Value |
|---|---|
| Body | 230 x 190 mm (about 260 x 235 mm with splitter, spoiler and wheels) |
| Wheel diameter | 70 mm |
| Wheelbase (rear to front axle) | 135 mm |
| Track (wheel centers) | 210 mm |
| Steering lock | 30 deg, inner wheel (**assumed**) |
| Tightest turn | about 0.33–0.38 m radius (measured in the sim) |
| Top speed / acceleration | 0.8 m/s, 1.5 m/s² (**assumed**) |
| Mass | ~1.6 kg |

`base_link` is the body center at axle height. The rear axle is 0.06 m behind it, the front axle 0.075 m ahead. Change the dimensions and assumed values at the top of the xacro file to match the real robot.

The real chassis (from the team CAD) has a longer 220 mm wheelbase and turns wider, about 0.45 m at 30 degrees of lock. That's right at the limit for the U-turns at each end of the track, so give the real servo linkage **at least 30 degrees of lock**.

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

The camera is fixed (no pan or tilt). By default it sits on a mast on the rear deck, 0.185 m up, level and facing forward. Move it with the `cam_*` launch options; see [Camera placement](#camera-placement).

The real sensor is 13 MP. The sim uses 640 x 480 so it runs in real time; raise `width`/`height` in the URDF to match your DepthAI pipeline if you need more detail.

### IMU

`/camera/imu` simulates the OAK-D Lite's [BMI270](https://bosch-sensortec.com/products/motion-sensors/imus/bmi270.html), in frame `imu_link` (x forward, y left, z up):

- **200 Hz.** DepthAI tops out around 250 Hz on this chip.
- **Noise from the Bosch datasheet:** gyro 0.007 °/s/√Hz and accelerometer 160 µg/√Hz, which is about 0.0017 rad/s and 0.022 m/s² per axis at 200 Hz. Each run also gets a small random bias, as on a calibrated sensor.
- **Gravity is included.** At rest, accel z reads about +9.81 m/s², like the real chip.
- **No orientation.** Like the real BMI270, it gives raw accel and gyro only. The orientation field in the ROS message is all zeros; ignore it, and integrate the gyro or fuse it yourself.

On the real camera the chip's axes differ from `imu_link`; the DepthAI ROS driver rotates them. Check your driver's frame before reusing sim-tuned code. Early Kickstarter OAK-D Lite units have no IMU at all.

## Camera placement

The camera mount is set by five launch options, so you can try a mount without editing files:

| Option | Meaning | Default |
|---|---|---|
| `cam_x:=M` | Meters forward of `base_link` (the body center) | `-0.10` (rear deck) |
| `cam_y:=M` | Meters left of the centerline | `0` |
| `cam_z:=M` | Lens height above the floor, meters | `0.185` |
| `cam_pitch:=DEG` | Tilt, degrees down | `0` |
| `cam_yaw:=DEG` | Turn, degrees left | `0` |

```bash
./simulation/run_sim.sh cam_x:=-0.11 cam_z:=0.22 cam_yaw:=15          # look at it in the sim
./simulation/run_sim.sh --batch --runs 5 --cmd "..." cam_x:=-0.11 cam_yaw:=15   # test your code with it
```

Keep the robot's whole camera mast inside the 12 in start cube: the top of the camera body is about 14 mm above `cam_z`.

The start flag tower's position is an assumption until the game manual's flag appendix is out (see [Known gaps](#known-gaps)), and the raised flag sits near the edge of the camera's view from the start box. Re-check the mount once the real position is known.

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
| `scripts/match.py` | Starts a round or resets the match on the running sim (`--start-round`, `--reset`) |
| `scripts/wait_ready.py` | Waits until a sim is ready (used by `--no-gui`, or your own scripts) |
| `worlds/track.sdf` | The field (generated, do not edit by hand) |
| `models/apriltag_41h12_<id>/` | Tag card models (generated) |
| `models/plush_car_<color>/` | Obstacle car models (generated) |
| `models/robot/robot.urdf.xacro` | The robot (stock car with Ackermann steering), its OAK-D Lite camera and IMU |
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
- **`[ERROR] [gazebo-1]: process has died ... exit code -2` in the log after stopping.** That's normal. Exit code -2 means Gazebo stopped on the Ctrl+C (SIGINT) from you or `--stop`, and `ros2 launch` reports every stop of a required process as an error. It's only a real crash if the sim stopped on its own, with another exit code (for example -11, a segfault).
- **A batch test was interrupted.** Ctrl+C or `--stop` both shut down the current run's sim and your robot command cleanly, and the summary keeps the runs that finished.

### WSL

- **Clone the repo inside WSL** (for example `~/SECON27`), not under `/mnt/c/...`. Windows folders are much slower from WSL and can lose the scripts' executable permission. Folder names with spaces do work.
- **If you clone with Git for Windows**, `simulation/.gitattributes` keeps the sim's files on Unix line endings. If you still see `bash\r: No such file or directory`, your copy has Windows line endings; re-clone from inside WSL.
- **Graphics.** `run_sim.sh` detects WSL and keeps `LIBGL_ALWAYS_SOFTWARE` and `QT_QUICK_BACKEND`, which WSL needs (the batch test does the same). On native Linux it removes them, because they crash Gazebo there. WSL needs WSLg (Windows 11, or Windows 10 with WSLg installed) for the Gazebo window and cameras.

## Known gaps

Values marked `ASSUMPTION` in `scripts/field_spec.py` are not pinned down by the documents. Check them on the Discord Q&A:

- **Pit box numbering.** Pit 1 is assumed to be the box next to the start box.
- **Start flag.** Appendix F says details come in Version 3.1. The flag is modeled as a tower on the start-side end of the divider, with an arm that swings up. The camera placement depends on this; see [Camera placement](#camera-placement).
- **Octagon lane line.** The chamfer angle is not dimensioned. The model fits the drawn 9 3/4 in chamfer edges to within the ±1 in tolerance.
- **Obstacle count.** The Game Manual says 3 obstacles. The Build Guide says at least 1 per lane, and `docs/field.md` says 2. The sim defaults to 3; use `obstacles:=2` to match the older docs.
- **Car mass and friction.** Not specified.

Not modeled yet, on purpose:

- **Robot chassis.** The sim uses the stock-car body, which is shorter than the real chassis (135 vs 220 mm wheelbase). Steering lock, top speed and acceleration are assumed. Update them at the top of `models/robot/robot.urdf.xacro` to match the real robot.
- **Pit stop tires and school flag.** These wait until the mechanisms are designed.
