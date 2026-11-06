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

Find a Apriltag library that we can use in ROS2 or find something that is already built. We just need to be able to decode the apriltag here and return its value, this will be passed onto `robot_planner`

### Lane Detection

We may need a yolo model or not we will have to do research

### Start Flag Detection

We need to detect a green flag dropping

### Object Detection

We need to be able to detect cars objects


