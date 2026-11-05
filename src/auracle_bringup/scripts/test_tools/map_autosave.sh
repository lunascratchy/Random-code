#!/bin/bash
# Saves /auracle/map every 20 s while mapping, alternating between two slots so a power cut
# mid-write can never destroy the last good copy. Newest good copy: ~/maps/auto_latest.{yaml,pgm}
#   ~/auracle_tools/map_autosave.sh        (Ctrl+C to stop)
source /opt/ros/jazzy/setup.bash
mkdir -p ~/maps
slot=a
while true; do
  if timeout 15 ros2 run nav2_map_server map_saver_cli -f ~/maps/auto_$slot \
       --ros-args -r map:=/auracle/map -p save_map_timeout:=10.0 >/dev/null 2>&1; then
    sync
    ln -sf ~/maps/auto_$slot.yaml ~/maps/auto_latest.yaml
    ln -sf ~/maps/auto_$slot.pgm ~/maps/auto_latest.pgm
    echo "$(date +%H:%M:%S) saved ~/maps/auto_$slot"
    [ $slot = a ] && slot=b || slot=a
  else
    echo "$(date +%H:%M:%S) no map yet"
  fi
  sleep 20
done
