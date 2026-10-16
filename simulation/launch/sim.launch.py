"""Start Gazebo with the race track world and spawn the camera robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

SIM_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
URDF_PATH = os.path.join(SIM_DIR, 'models', 'robot', 'robot.urdf')
WORLD_PATH = os.path.join(SIM_DIR, 'worlds', 'track.sdf')
BRIDGE_PATH = os.path.join(SIM_DIR, 'config', 'bridge.yaml')

# Spawn pose: inner lane of the east straightaway, facing +y.
SPAWN_X = '0.1556'
SPAWN_Y = '-0.30'
SPAWN_Z = '0.05'
SPAWN_YAW = '1.5708'


def generate_launch_description():
    with open(URDF_PATH) as f:
        robot_description = f.read()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r ' + WORLD_PATH,
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'race_robot',
            '-x', SPAWN_X, '-y', SPAWN_Y, '-z', SPAWN_Z, '-Y', SPAWN_YAW,
        ],
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

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
