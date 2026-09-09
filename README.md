# auracle_description
## urdf
- robot_core.xacro: describes the general shape and dimensions of the robot; the chassis. Most joints are fixed, so suspension won't rock. To have a functional suspension, the left_wheel_rocker_joint and right_wheel_rocker_joint should be type="revolute". They should have <limit> tags to avoid collapsing
- robot.urdf.xacro: includes all my related xacro files and makes them urdf's
- inertial_macros.xacro: includes the inertial properties for the urdf. Standard inertial formulas
- gazebo_control.xacro: includes gazebo plugins, such as the diff_drive plugin for 'motor' control
face.xacro: It adds a "front" to the robot. In Rviz, a box looks the same from all sides. Adding a "face" or a "nose" link helps you visually confirm which way is "forward" when you command it to drive. 
- lidar.xacro: Provides the Scan data (the laser dots). It tells the robot where walls are.
- imu.xacro: Provides Inertial data (rotation speed). It helps the robot stay precise when it turns.
note: In simulation the Gazebo plugin calculates how much the wheels turned and "fakes" the odometry. On the real robot Arduino reads the Encoders, and your code converts those clicks into odometry.
- ros2_control.xacro: 

# auracle_bringup
rsp.launch.py: handles the "Robot State Publisher," which takes your Xacro files, converts them to URDF, and publishes the tf (transform) tree that ROS needs to know where the Lidar is relative to the wheels.

- Baud rate: speed of communication (bits per second) over the Serial cable. For an Arduino Nano to a Pi 5, 115,200 is the industry standard. You can go higher (up to 1M), but you risk "bit flipping" (data corruption) if your wires are long or have electrical noise from your motors.

- Using ros2_control as a plugin that the controller_manager loads into memory. Faster and more stable vs writing a custom node that subscribes to /cmd_vel, calculates, sends and receives serial comms and publishes /odom. If sim_mode = true, the gz_ros2_control/GazeboSimSystem plugin is loaded, if false, it loads 

- auracle_hardware.xml: his is a Plugin Export file. It tells the ROS 2 pluginlib: "If someone asks for 'AuracleHardwareInterface', give them the C++ class found in this library.

- what are all those numbers in my_controllers.yaml...

- a Watchdog Timer (WDT) is essentially a "Dead Man's Switch." This dog has a timer that counts down from 2 seconds to 0. If the timer ever reaches 0, the dog "bites" the Arduino, which forces the entire microcontroller to hard-reset (reboot). To keep the dog from biting, your code must "pet" the dog (reset the timer back to 2 seconds) every single time the main loop finishes. Imagine your robot is driving toward a wall at full speed. Suddenly, a bit of electrical noise from the motors causes the Arduino to "freeze" or enter an infinite loop. Without a Watchdog: The Arduino is frozen, but the L298N motor driver is still receiving the last signal ("Full Speed Forward"). The robot crashes into the wall. With a watchdog, the code freezes, the dog isn't petted, the timer hits 0, the arduino reboots, the pins go LOW, the robot stops