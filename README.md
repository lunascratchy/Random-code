# Building - either
```
cd ~/Documents/auracle_ws
rm -rf build install log
colcon build --symlink-install && source install/setup.bash
```

# Simulation 
## Terminal 1 (Gazebo + robot)8
This launches the robot in Gazebo with use_sim_time:=true baked in
```
source install/setup.bash && ros2 launch auracle_bringup launch_sim.launch.py
```
Wait for it to settle — Gazebo window opens, spawner sequence finishes diff_cont → joint_broad → imu_broadcaster, one after another now

To confirm in a separate terminal
```
ros2 control list_controllers          # diff_cont must show "active". If any inactive, launch with ros2 run controller_manager spawner diff_cont
```
## Terminal 2 (SLAM, NAV2, Rviz)
```
source install/setup.bash && ros2 launch auracle_bringup slam_nav_rviz.launch.py use_sim_time:=true
```
## Terminal 3 (Teleop)
```
source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_joy
```

# Real-time
## Flashing the arduino nano
```
#using arduino ide
# plug it into the Pi, confirm the port, upload
ls /dev/ttyUSB* /dev/ttyACM*

#using platformio
PlatformIO (from firmware/auracle_firmware/, where platformio.ini lives)
pio run -t upload
```

## Terminal 1 (hardware interface + lidar)
```
source install/setup.bash && ros2 launch auracle_bringup launch_robot.launch.py use_sim_time:=false
# if lidar port not at /dev/rplidar, override
source install/setup.bash && ros2 launch auracle_bringup launch_robot.launch.py use_sim_time:=false lidar_port:=/dev/ttyUSB0
```
## Terminal 2 (SLAM + Nav2 + RViz)
```
source install/setup.bash && ros2 launch auracle_bringup slam_nav_rviz.launch.py use_sim_time:=false
```
## Terminal 3 (teleop)
```
source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_joy
```

### namespacing
Add namespace:=<name> to all three commands, and give teleop the matching __ns remap
```
ros2 launch auracle_bringup launch_sim.launch.py namespace:=robot1
ros2 launch auracle_bringup slam_nav_rviz.launch.py use_sim_time:=true namespace:=robot1
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_joy -r __ns:=/robot1
```
### Joystick (optional, either mode)
ros2 launch auracle_bringup joystick.launch.py use_sim_time:=<true|false> namespace:=<same as above>



