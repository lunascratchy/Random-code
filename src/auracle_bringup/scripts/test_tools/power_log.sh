#!/bin/bash
# Logs the Pi 5's 5 V input, core current and throttle flags every 0.5 s, synced line by line
# so the last readings before a power cut survive. Output: ~/auracle_tools/runs/power_<boot time>.log
OUT=~/auracle_tools/runs/power_$(date +%H%M%S).log
echo "time ext5v_V vdd_core_A throttled" > $OUT
while true; do
  v=$(vcgencmd pmic_read_adc EXT5V_V | sed -E 's/.*=([0-9.]+)V/\1/')
  a=$(vcgencmd pmic_read_adc VDD_CORE_A | sed -E 's/.*=([0-9.]+)A/\1/')
  t=$(vcgencmd get_throttled | cut -d= -f2)
  echo "$(date +%H:%M:%S.%N | cut -c1-12) $v $a $t" >> $OUT
  sync $OUT
  sleep 0.5
done
