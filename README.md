# building
cd ~/your_ros2_ws          # wherever you put the src/ folder from the zip
colcon build --symlink-install
source install/setup.bash
# Simulation (Gazebo + RViz)
This launches the robot in Gazebo with use_sim_time:=true baked in

ros2 launch auracle_bringup launch_sim.launch.py
# Real robot
Flash the Arduino first, plug it into the Pi, confirm the port
ls /dev/ttyUSB* /dev/ttyACM*
On the pi
ros2 launch auracle_bringup launch_robot.launch.py use_sim_time:=false
Second terminal, same SLAM/Nav2/RViz launch, just flip the flag
ros2 launch auracle_bringup slam_nav_rviz.launch.py use_sim_time:=false

That's the whole point of the use_sim_time argument — same commands, same nodes, only the clock source changes.

# Namespacing (multi-robot, or just isolating topics)
Add namespace:=<name> to every launch call, consistently, across all three commands (bringup, SLAM/Nav2, and lidar if launched separately)

ros2 launch auracle_bringup launch_robot.launch.py use_sim_time:=false namespace:=robot1
ros2 launch auracle_bringup slam_nav_rviz.launch.py use_sim_time:=false namespace:=robot1

# flashing arduino nano
PlatformIO (from firmware/auracle_firmware/, where platformio.ini lives)
pio run -t upload

# Joystick (optional, either mode)
ros2 launch auracle_bringup joystick.launch.py use_sim_time:=<true|false> namespace:=<same as above>