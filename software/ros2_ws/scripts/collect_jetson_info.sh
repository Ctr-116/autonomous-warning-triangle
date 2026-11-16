#!/usr/bin/env bash
# Read-only collection. Does not open serial ports or send motor commands.
set -u
printf '\nJetson model:\n'
if [ -r /proc/device-tree/model ]; then tr -d '\0' < /proc/device-tree/model; printf '\n'; fi
printf '\nOS:\n'
cat /etc/os-release
printf '\nJetPack/L4T:\n'
if [ -r /etc/nv_tegra_release ]; then cat /etc/nv_tegra_release; fi
printf '\nROS installations:\n'
ls -1 /opt/ros 2>/dev/null || true
printf '\nSerial devices:\n'
ls -l /dev/serial/by-id /dev/rosmaster /dev/myserial /dev/lora 2>/dev/null || true
printf '\nExisting SDK path:\n'
python3 -c 'import Rosmaster_Lib; print(Rosmaster_Lib.__file__)' 2>&1 || true
printf '\nExisting ROS processes:\n'
ps -eo pid,args | grep -E 'rosmaster|Mcnamu|base_node|roslaunch|ros2 launch' || true
