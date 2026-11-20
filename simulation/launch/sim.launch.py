"""Start Gazebo with the race field, spawn the camera robot, randomize the field.

Launch arguments (all optional):
    randomize:=true|false   random layout at startup (false keeps the world
                            file's default layout; you can still randomize
                            later through /field/randomize)
    seed:=N                 layout seed, -1 = random (default)
    obstacles:=N            plush cars on the course, 0-4 (default 3)
    start_tag:=N            force the start AprilTag ID 1-3, 0 = die roll
    tolerance:=true|false   shift tags up to +/- 1 in (field tolerance)
    interval:=S             new layout every S seconds, 0 = off (default)
    start_turn:=DEG         robot start heading, degrees left of facing the
                            racecourse (default 0). See README, "Seeing the
                            start flag".
    gui:=true|false         false runs only the Gazebo server, no window
                            (faster; cameras still work). Used by the batch
                            test runner.
    cam_x:=M  cam_y:=M  cam_z:=M  cam_pitch:=DEG  cam_yaw:=DEG
                            OAK-D Lite mount: m forward of the body center,
                            m left, m above the floor, deg tilted down,
                            deg turned left. Default: rear deck, level,
                            facing forward (-0.10, 0, 0.185, 0, 0).

Example:
    ros2 launch simulation/launch/sim.launch.py seed:=42 obstacles:=2
"""

import os
import shlex
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

SIM_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
SCRIPTS_DIR = os.path.join(SIM_DIR, 'scripts')
MODELS_DIR = os.path.join(SIM_DIR, 'models')
XACRO_PATH = os.path.join(MODELS_DIR, 'robot', 'robot.urdf.xacro')
WORLD_PATH = os.path.join(SIM_DIR, 'worlds', 'track.sdf')
BRIDGE_PATH = os.path.join(SIM_DIR, 'config', 'bridge.yaml')
FIELD_MANAGER = os.path.join(SCRIPTS_DIR, 'field_manager.py')

sys.path.insert(0, SCRIPTS_DIR)
import field_spec  # noqa: E402


def _truthy(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def field_manager_action(context):
    cfg = {k: LaunchConfiguration(k).perform(context) for k in
           ('randomize', 'seed', 'obstacles', 'start_tag', 'tolerance', 'interval',
            'start_turn')}
    cmd = [sys.executable, '-B', FIELD_MANAGER,
           '--start-turn', cfg['start_turn'],
           '--seed', cfg['seed'],
           '--obstacles', cfg['obstacles'],
           '--start-tag', cfg['start_tag'],
           '--interval', cfg['interval']]
    if not _truthy(cfg['randomize']):
        cmd.append('--keep')
    if _truthy(cfg['tolerance']):
        cmd.append('--tolerance')
    return [ExecuteProcess(cmd=cmd, name='field_manager', output='screen')]


def gazebo_action(context):
    """Gazebo with or without its window (gui:=false)."""
    gz_args = '-r '
    if not _truthy(LaunchConfiguration('gui').perform(context)):
        gz_args += '-s '
        # Cameras need a renderer. Without a display, use EGL offscreen
        # rendering; with one (desktop, xrdp, VNC), use the display.
        if not os.environ.get('DISPLAY'):
            gz_args += '--headless-rendering '
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py')),
        launch_arguments={
            # ros_gz_sim runs gz through a shell: quote the path so a repo
            # folder with spaces (common on WSL under /mnt/c) still works.
            'gz_args': gz_args + shlex.quote(WORLD_PATH),
            'on_exit_shutdown': 'true',
        }.items(),
    )]


def spawn_robot_action(context):
    """Spawn the robot centered in the start box, turned start_turn deg left."""
    turn = float(LaunchConfiguration('start_turn').perform(context))
    x, y, z, yaw = (str(round(v, 4)) for v in field_spec.robot_spawn(turn))
    return [Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'race_robot',
            '-x', x, '-y', y, '-z', z, '-Y', yaw,
        ],
        output='screen',
    )]


# Default mount: rear deck, level, facing forward (README, "Camera placement").
CAM_ARGS = {'cam_x': '-0.10', 'cam_y': '0.0', 'cam_z': '0.185',
            'cam_pitch': '0.0', 'cam_yaw': '0.0'}


def robot_action(context):
    """Build the robot from xacro (camera mount from launch options),
    publish its description, and spawn it in the start box."""
    mappings = {k: LaunchConfiguration(k).perform(context) for k in CAM_ARGS}
    robot_description = xacro.process_file(XACRO_PATH, mappings=mappings).toxml()
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )
    return [rsp] + spawn_robot_action(context)


def generate_launch_description():
    args = [
        DeclareLaunchArgument('randomize', default_value='true'),
        DeclareLaunchArgument('seed', default_value='-1'),
        DeclareLaunchArgument('obstacles', default_value=str(field_spec.DEFAULT_NUM_OBSTACLES)),
        DeclareLaunchArgument('start_tag', default_value='0'),
        DeclareLaunchArgument('tolerance', default_value='false'),
        DeclareLaunchArgument('interval', default_value='0'),
        DeclareLaunchArgument('start_turn', default_value='0'),
        DeclareLaunchArgument('gui', default_value='true'),
    ] + [DeclareLaunchArgument(k, default_value=v) for k, v in CAM_ARGS.items()]

    # Lets the world find model://apriltag_41h12_<id> and model://plush_car_<color>.
    resource_path = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', MODELS_DIR)

    gz_sim = OpaqueFunction(function=gazebo_action)

    robot = OpaqueFunction(function=robot_action)

    # Point cloud. Gazebo's own RGB-D cloud is labeled camera_optical_link but
    # filled in body axes (x forward), so it shows up on its side in RViz.
    # Build it from the depth image instead, in the optical frame, like the
    # real OAK-D Lite ROS driver does.
    point_cloud = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        name='point_cloud_xyzrgb',
        remappings=[
            ('rgb/image_rect_color', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth_registered/image_rect', '/camera/depth_image'),
            ('points', '/camera/points'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': BRIDGE_PATH,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # field_manager waits for the world itself; the delay only keeps its
    # output after Gazebo's startup messages.
    field_manager = TimerAction(period=2.0, actions=[OpaqueFunction(function=field_manager_action)])

    return LaunchDescription(args + [
        resource_path,
        gz_sim,
        robot,
        bridge,
        point_cloud,
        field_manager,
    ])
