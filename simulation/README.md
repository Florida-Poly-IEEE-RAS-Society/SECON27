# Simulation

Gazebo simulation of the race field and robot. ROS 2 nodes in `ros2/src` control the robot.

## Setup

Needs Ubuntu 24.04 with ROS 2 Jazzy. Install the Gazebo packages once:

```bash
sudo apt install ros-jazzy-ros-gz
```

## Run

```bash
./simulation/run_sim_local.sh   # native Linux or a VM
./simulation/run_sim_wsl.sh     # WSL
```

The local script unsets `QT_QUICK_BACKEND` and `LIBGL_ALWAYS_SOFTWARE`, which crash Gazebo outside WSL. The WSL script leaves your environment as is.

Gazebo opens with the field. The robot spawns at the same spot, facing the same way, every time.

## Topics

| Topic | Use |
|---|---|
| `/cmd_vel` | Drive the robot |
| `/camera/image_raw` | Camera image |
| `/camera/camera_info` | Camera calibration |
| `/odom` | Robot position estimate |

Set `use_sim_time:=true` on every ROS 2 node that talks to the simulation.

## Files

| File | Contents |
|---|---|
| `run_sim_local.sh` | Starts everything (native Linux / VM) |
| `run_sim_wsl.sh` | Starts everything (WSL) |
| `launch/sim.launch.py` | Starts Gazebo, spawns the robot, connects topics to ROS 2 |
| `worlds/track.sdf` | The field |
| `models/robot/robot.urdf` | The robot and its camera |
| `config/bridge.yaml` | List of topics shared with ROS 2 |

## Known gaps

- Field values marked `GUESS` in `track.sdf` are placeholders until the official field drawing is available.
- The AprilTag is a plain black square for now.
