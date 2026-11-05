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

        ros2 launch auracle_bt_1 mission.launch.py route:=2

    Starts Nav2 + AMCL on the map (start_nav:=true) and the mission runner, which waits for them,
    sets the start pose, then drives route_<route> from config/route.yaml. No RViz here (laptop)."""
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
            'route_file': LaunchConfiguration('route_file'),
            'tree_file': LaunchConfiguration('tree_file'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('route', default_value='1',
                              description='1, 2 or 3: which route_N in route.yaml to drive'),
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/maps/trial_6a.yaml')),
        DeclareLaunchArgument('start_nav', default_value='true',
                              description='false if Nav2 is already running'),
        DeclareLaunchArgument('namespace', default_value='auracle'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('route_file', default_value='',
                              description='empty = share/auracle_bt_1/config/route.yaml'),
        DeclareLaunchArgument('tree_file', default_value='',
                              description='empty = share/auracle_bt_1/behavior_trees/region_tree.xml'),
        nav,
        runner,
    ])