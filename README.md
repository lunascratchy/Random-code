# Building - either
```
cd ~/Documents/auracle_ws
rm -rf build install log && colcon build --symlink-install && source install/setup.bash
```

# Virtual Teleop
## Terminal 1 (Gazebo + robot)
This launches the robot in Gazebo with use_sim_time:=true baked in. Build and save a map
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

## Terminal 4 (Map saving)
This writes map_1.yaml + map_1.pgm into config/
```
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/Documents/auracle_ws/src/auracle_bringup/config/map_1
```
Add map_server block to nav2_params.yaml
```
map_server:
  ros__parameters:
    yaml_filename: ""
```

## Terminal 1 (simulation)
```
source install/setup.bash && ros2 launch auracle_bringup launch_sim.launch.py
```

## Terminal 2 (AMCL + Nav2 + RViz, against your saved map)
```
source install/setup.bash&& ros2 launch auracle_bringup localization_nav_rviz.launch.py use_sim_time:=true map:=$(pwd)/src/auracle_bringup/config/map_1.yaml
```
Click the + icon at the end of the toolbar (next to Nav2 Goal).
In the tool picker, search for and add "2D Pose Estimate" (this is the standard rviz tool, distinct from Nav2 Goal).
Click that new tool, then click-and-drag on the map at the robot's actual position and heading in Gazebo — click sets the position, the drag direction sets which way it's facing.
Click "2D Pose Estimate", then click-drag on the map at the robot's actual position/heading in Gazebo. Don't send Nav2 goals and teleop at the same time (they'll f]
ight over cmd_vel).


# Real robot (Pi 5 + Arduino Nano)

## Flashing the Nano from WSL
PowerShell as admin, once per device:
```
usbipd list                      # Nano = FTDI 0403:6001, e.g. BUSID 1-2
usbipd bind --busid=1-2
```
Every time you replug:
```
usbipd attach --wsl --busid=1-2
```
In WSL:
```
ls /dev/ttyUSB*                  # FTDI shows up as /dev/ttyUSB0
cd firmware/auracle_firmware
pio run -t upload --upload-port /dev/ttyUSB0
```
To flash from the Pi instead, use the same command with `--upload-port /dev/arduino`. Close ROS first, because only one program can hold the port.

## Installations
```
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-robot-localization   ros-jazzy-twist-mux ros-jazzy-twist-stamper ros-jazzy-rplidar-ros ros-jazzy-slam-toolbox   ros-jazzy-teleop-twist-keyboard ros-jazzy-joint-state-publisher libserial-dev
sudo usermod -aG dialout $USER             # then log out/in
sudo cp src/auracle_bringup/config/99-auracle.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/arduino /dev/rplidar
```
Building on a 4 GB Pi 5: limit parallel jobs so the C++ builds don't run out of RAM, and skip the sim-only packages:
```
MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2   --cmake-args -DCMAKE_BUILD_TYPE=Release   --packages-skip auracle_slam_tuning auracle_vision
```
Run RViz on your laptop (same `ROS_DOMAIN_ID`, same network) rather than on the Pi.

## Adding udev rules
```
#check if rules file exists
ls /etc/udev/rules.d/99-auracle.rules
#if missing, copy from workspace
sudo cp src/auracle_bringup/config/99-auracle.rules /etc/udev/rules.d/
Reload and trigger the udev rules to generate the symlinks
sudo udevadm control --reload-rules
sudo udevadm trigger
#Verify the symlinks now exist and point to the correct ttyUSB ports
ls -l /dev/arduino /dev/rplidar
#ensure your user account is in the dialout group to allow serial access:
udo usermod -aG dialout $USER
#log out and log back into your Ubuntu session for the group changes to take effect.
sudo reboot
```

# Teleop on the real robot (copy-paste)
Every terminal below is on the Pi. Keep the robot still while Terminal 1 starts: opening the port resets the Nano, and it spends about 0.5 s calibrating the gyro.

**Terminal 1: robot (hardware interface, controllers, lidar, EKF)**
```bash
source install/setup.bash && ros2 launch auracle_bringup launch_robot.launch.py
```
If there are no udev rules, add `arduino_port:=/dev/ttyUSB0 lidar_port:=/dev/ttyUSB1`.

No lidar plugged in yet? Teleop still works; just skip it:
```bash
ource install/setup.bash && ros2 launch auracle_bringup launch_robot.launch.py use_lidar:=false
```

*Terminal 2: check that the controllers are up** (`diff_cont`, `joint_broad`, `imu_broadcaster` should all be `active`)
```bash
source install/setup.bash && ros2 control list_controllers
```

*Terminal 3: keyboard teleop**
```bash
source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_joy
```
The real robot is limited to 0.25 m/s and 1.5 rad/s (`config/my_controllers_real.yaml`). Teleop starts at 0.5 m/s, and the controller clamps it.

**With a namespace** (for example `robot1`), pass the same name to both commands:
```bash
source install/setup.bash && ros2 launch auracle_bringup launch_robot.launch.py namespace:=robot1
```
```bash
source install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_joy -r __ns:=/robot1
```

**Optional: SLAM + Nav2 + RViz** (better run on the laptop, same `ROS_DOMAIN_ID`)
```bash
source install/setup.bash && ros2 launch auracle_bringup slam_nav_rviz.launch.py use_sim_time:=false
```
Nav2's `vx_max: 0.5` in `nav2_params.yaml` is higher than the real robot's limit of about 0.25 m/s. Lower it before navigating on hardware.

## Calibration (in this order)
1. **Wheel direction.** Prop the robot up and teleop forward. `ros2 topic echo /joint_states_hw` should show both rear wheel positions increasing. If one side decreases, set that side's `*_direction_sign` to `-1.0` in `ros2_control.xacro`.
2. **Ticks per rev.** With ROS stopped, open `pio device monitor`. Mark a rear wheel and turn it exactly 10 turns by hand, then read the change in the `e` line counts and divide by 10. Put the results in `left/right_enc_counts_per_rev` in `ros2_control.xacro`. Note that 730/655 came from the old code and may not match 1x decoding.
3. **Wheel radius.** `ros2 run auracle_bringup drive_test.py --distance 1.0`, then tape-measure the real distance and type it in. Put the printed `wheel_radius` into `my_controllers_real.yaml`.
4. **Wheel separation.** `ros2 run auracle_bringup drive_test.py --angle 360`, then enter the real rotation. Put the printed `wheel_separation` into `my_controllers_real.yaml`. On a skid-steer robot this ends up larger than the measured track width.
5. **5 cm check.** `ros2 run auracle_bringup drive_test.py --distance 0.05`. Once steps 2-3 are done, it should land within a few mm. Most of the remaining error is coast after the stop command.

## PID deadzone
Speed resolution is 1 tick per 50 ms window, which is **20 ticks/s**. A wheel holding a steady speed reads N or N+1 ticks per window, so the measured error jumps by ±20 even when it's on target. `PID_DEADZONE_TICKS_S = 25` is just above one step, so the PID ignores that jitter.

To check it, hold a constant speed (teleop, prop the robot up) and plot `/joint_states_hw` rear velocities in `rqt_plot`. One step is 20 / ticks_per_rev × 2π ≈ 0.17 rad/s. If the steady-state trace jumps by about 2 steps, raise the deadzone to about 45.

## Front-offset test (optional)
Prop the robot up and stop ROS. This drives both sides open-loop at rear PWM 150 for 10 s. It resends the command every 0.2 s so the 500 ms watchdog doesn't stop the motors:
```
stty -F /dev/arduino 115200 raw -echo -hupcl
(sleep 2; for i in $(seq 50); do echo "o 150 150"; sleep 0.2; done; echo s) > /dev/arduino
```
Count front vs rear wheel turns while it runs. Set `LF_OFFSET = rear_turns / front_turns` (same for the right side) in `config.h` and reflash.

### namespacing
Add namespace:=<name> to all three commands, and give teleop the matching __ns remap
```
ros2 launch auracle_bringup launch_sim.launch.py namespace:=robot1
ros2 launch auracle_bringup slam_nav_rviz.launch.py use_sim_time:=true namespace:=robot1
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_joy -r __ns:=/robot1
```
### Joystick (optional, either mode)
ros2 launch auracle_bringup joystick.launch.py use_sim_time:=<true|false> namespace:=<same as above>