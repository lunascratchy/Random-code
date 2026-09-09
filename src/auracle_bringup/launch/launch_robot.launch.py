import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction, DeclareLaunchArgument,
    GroupAction, RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    bringup_pkg = get_package_share_directory('auracle_bringup')
    description_pkg = get_package_share_directory('auracle_description')

    # Single source of truth for sim time. This is the REAL ROBOT launch file, so it
    # defaults to false. Every node below reads this same value instead of hardcoding
    # its own True/False, so they can't drift out of sync with each other again.
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    namespace = LaunchConfiguration('namespace')
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace applied to every node/topic below'
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_pkg, 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': use_sim_time, 'namespace': namespace}.items()
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_pkg, 'launch', 'joystick.launch.py'
        )]),
        launch_arguments={'use_sim_time': use_sim_time, 'namespace': namespace}.items()
    )

    # Real hardware only - launch_sim.launch.py gets its /scan from the
    # Gazebo lidar plugin + ros_gz_bridge instead. Without this, running
    # launch_robot.launch.py on the actual robot gave SLAM/Nav2 no scan data
    # at all - the lidar driver was never started anywhere.
    lidar_port = LaunchConfiguration('lidar_port')
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/rplidar',
        description=("Serial device for the RPLidar. Prefer a stable udev symlink "
                     "(e.g. create /dev/rplidar via udev rule) over /dev/ttyUSBx, "
                     "which can renumber across reboots/replugs.")
    )
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_pkg, 'launch', 'rplidar.launch.py'
        )]),
        launch_arguments={'namespace': namespace, 'serial_port': lidar_port}.items()
    )

    twist_mux_params = os.path.join(bringup_pkg, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        # Relative names: under a namespace these resolve to
        # <namespace>/cmd_vel_out etc; with namespace:='' behavior is
        # unchanged from before.
        remappings=[('cmd_vel_out', 'cmd_vel_unstamped')]
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel_in', 'cmd_vel_unstamped'),
                    ('cmd_vel_out', 'diff_cont/cmd_vel')]
    )

    # Built directly from the same xacro used by rsp.launch.py instead of the old
    # `ros2 param get --hide-type /robot_state_publisher robot_description` hack.
    # That hack raced robot_state_publisher's startup (it could run before RSP had
    # published anything, and it hardcoded the UNNAMESPACED node name /robot_state_publisher,
    # which breaks the moment namespace is anything other than ''). Generating the
    # description directly here has neither problem.
    xacro_file = os.path.join(description_pkg, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim_mode:=', use_sim_time]),
        value_type=str
    )

    controller_params_file = os.path.join(bringup_pkg, 'config', 'my_controllers.yaml')

    # NOTE on namespace: controller_manager and the three spawners below are
    # all started indirectly (TimerAction / RegisterEventHandler), and
    # actions started that way do NOT inherit a PushRosNamespace from an
    # enclosing GroupAction - the push is already popped by the time the
    # timer/event fires (confirmed upstream: ros2/launch#743). So each one
    # gets an explicit namespace= here instead of relying on the
    # GroupAction it's still listed in below (kept for the nodes that DO
    # start synchronously and so still benefit from the push).
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time},
                    controller_params_file],
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["imu_broadcaster"],
    )

    delayed_imu_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[imu_broadcaster_spawner],
        )
    )

    # Fuses /diff_cont/odom (wheel encoders) with /imu_broadcaster/imu (MPU6050
    # gyro) and publishes the odom->base_link transform that slam_toolbox
    # actually uses. diff_cont.enable_odom_tf is false in my_controllers.yaml
    # so this is the ONLY thing publishing that transform now.
    ekf_params_file = os.path.join(bringup_pkg, 'config', 'ekf.yaml')
    ekf_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
    )

    # Everything except rsp (which handles its own namespacing/frame_prefix
    # internally) is namespaced here so relative topic names above resolve
    # under <namespace>/... . With namespace:='' (default) this is a no-op
    # and every topic name is identical to before.
    namespaced_nodes = GroupAction([
        PushRosNamespace(namespace),
        twist_mux,
        twist_stamper,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        delayed_imu_broadcaster_spawner,
        ekf_localization,
    ])

    return LaunchDescription([
        declare_use_sim_time,
        declare_namespace,
        declare_lidar_port,
        rsp,
        joystick,
        rplidar,
        namespaced_nodes,
    ])
