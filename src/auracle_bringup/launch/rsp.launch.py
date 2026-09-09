import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # 1. Check if we are told to use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 2. Point to the description package where the URDF lives
    pkg_path = os.path.join(get_package_share_directory('auracle_description'))
    
    # 3. Point to the specific xacro file
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    # 4. Process the xacro file into a URDF string
    robot_description_config = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' sim_mode:=', use_sim_time
        ]),
        value_type=str
    )
    
    # 5. Create the robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 6. Return the LaunchDescription
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false', # Set to false by default for the real robot
            description='Use sim time if true'),

        node_robot_state_publisher
    ])