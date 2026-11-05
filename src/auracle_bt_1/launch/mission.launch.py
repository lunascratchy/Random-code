from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start the mission runner. Run it on the Pi, after (or together with) the robot and
    localization_nav_rviz.launch.py (use_rviz:=false). It waits for Nav2 by itself."""
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='auracle'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('route_file', default_value='',
                              description='empty = share/auracle_bt/config/route.yaml'),
        DeclareLaunchArgument('tree_file', default_value='',
                              description='empty = share/auracle_bt/behavior_trees/region_tree.xml'),
        Node(
            package='auracle_bt',
            executable='bt_runner',
            name='bt_runner',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'route_file': LaunchConfiguration('route_file'),
                'tree_file': LaunchConfiguration('tree_file'),
            }],
        ),
    ])