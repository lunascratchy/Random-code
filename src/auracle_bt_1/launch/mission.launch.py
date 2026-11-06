import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Mission on the Pi, AFTER launch_robot.launch.py:

        ros2 launch auracle_bt_1 mission.launch.py map:=<map.yaml>

    Starts Nav2 + AMCL on the map (start_nav:=true) and the mission runner. The runner waits for
    Nav2, asks for a 2D Pose Estimate, drives to load region N, pauses, drives to offload region N
    and exits. Pick N with route:=1|2|3. Regions are in config/regions.yaml.
    No RViz here: on the laptop run  ros2 launch auracle_bt_1 mission_rviz.launch.py

    auto_pose:=true: no click at all. Park the robot where you started mapping, facing the same
    way, and it sets (0,0,0) as the start pose itself, then drives.

    First time: mark:=true. Click the centre of LOAD 1,2,3 then OFFLOAD 1,2,3 with the RViz
    Publish Point tool; the runner writes regions.yaml and exits. The robot does not move.
    Click-your-goals mode: tree_file:=goal_tree.xml"""
    bringup = get_package_share_directory('auracle_bringup')
    ns = LaunchConfiguration('namespace')

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'localization_nav_rviz.launch.py')),
        launch_arguments={
            'namespace': ns,
            'use_rviz': 'false',
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_nav')),
    )

    runner = Node(
        package='auracle_bt_1',
        executable='bt_runner',
        name='bt_runner',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'route': ParameterValue(LaunchConfiguration('route'), value_type=int),
            'mark': ParameterValue(LaunchConfiguration('mark'), value_type=bool),
            'auto_pose': ParameterValue(LaunchConfiguration('auto_pose'), value_type=bool),
            'route_file': LaunchConfiguration('route_file'),
            'tree_file': LaunchConfiguration('tree_file'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('route', default_value='1',
                              description='1, 2 or 3: drive to load_N then offload_N'),
        DeclareLaunchArgument('auto_pose', default_value='false',
                              description='true: robot is parked at the mapping start (0,0,0), no 2D Pose Estimate click'),
        DeclareLaunchArgument('mark', default_value='false',
                              description='true: record the six regions from Publish Point clicks'),
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/maps/trial_6a.yaml')),
        DeclareLaunchArgument('start_nav', default_value='true',
                              description='false if Nav2 is already running'),
        DeclareLaunchArgument('namespace', default_value='auracle'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('route_file', default_value='',
                              description='empty = share/auracle_bt_1/config/regions.yaml'),
        DeclareLaunchArgument('tree_file', default_value='',
                              description='empty = region_tree.xml (or mark_tree.xml when mark:=true). Name or full path'),
        nav,
        runner,
    ])