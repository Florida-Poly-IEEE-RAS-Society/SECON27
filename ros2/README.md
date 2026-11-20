# The ROS2 Workspace

```
.
└── src
    ├── camera_driver
    ├── perception
    │   ├── apriltag_detection
    │   ├── lane_detection
    │   ├── start_flag_detection
    │   └── object_detection
    ├── pitstop
    │   ├── tire_drop
    │   ├── tire_pick
    │   └── pit_stop_pick
    ├── race_manager
    │   ├── map_planner
    │   └── motor_controller
    ├── race_planner
    ├── robot_bringup
    └── robot_description
```

## camera_driver

This is where the hardware drivers will be for the OAK-D lite that will map the camera to specific topics as you see in the simulator.

Examples:

- `camera/depth`
- `camera/image`


## perception

This is where we will have our yolo models running to detect the lane and object. Apriltags will just be decoding information so it will not need a yolo model. 

### Apriltag Detection

Uses the [AprilRobotics/apriltag](https://github.com/AprilRobotics/apriltag) C library and its Python wrapper (git submodule in `src/third_party/apriltag`). Family `tagStandard41h12`, competition ids 1, 2, 3. The `apriltag_detector` node decodes tags from the camera and publishes the id, which is passed onto `robot_planner`.

```bash
git submodule update --init   # once, to fetch the apriltag library
cd ros2                       # build from here so colcon.meta is used
source /opt/ros/jazzy/setup.bash
colcon build --packages-select apriltag perception && source install/setup.bash
ros2 run perception apriltag_node
```

`tag_size` defaults to 0.0378 m: the detector corners lie on the black border, which is 5/9 of the 0.068 m printed tag.

| Topic | Type | Description |
| --- | --- | --- |
| `/apriltag/id` | `std_msgs/Int32` | Best tag id, published every frame a valid tag is seen |
| `/apriltag/detections` | `std_msgs/String` | JSON per frame: id, hamming, margin, center, corners, pose |
| `/apriltag/pit_spot` | `std_msgs/Int32` | Latched; tag id confirmed over `confirm_frames` consecutive frames. Locked after the first confirmation (`lock_on_confirm`) so pit tags seen during the race can't change it |
| `/apriltag/debug_image` | `sensor_msgs/Image` | Annotated image, only if `publish_debug_image` is true |

Subscribes to `/camera/image_raw` and `/camera/camera_info`. Service `/apriltag/reset` (`std_srvs/Trigger`) clears the locked pit spot.

The tag is read before the round: the technician turns the robot in the start box to face the tag, waits for `/apriltag/pit_spot` (node logs `Pit spot confirmed`), then sets it for the start. In the sim: `./simulation/run_sim.sh start_turn:=-90`. The default sim camera mount (rear deck, 0.25 m up, 20° down) is chosen so the tag is visible over the hood; at 0.185 m and level the hood hides the bottom of the tag and it is not detected.

### Lane Detection

We may need a yolo model or not we will have to do research

### Start Flag Detection

We need to detect a green flag dropping

### Object Detection

We need to be able to detect cars objects


