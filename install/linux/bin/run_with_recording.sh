#!/bin/bash
###
 # @Author: richie.li
 # @Date: 2024-11-18 17:03:42
 # @LastEditors: richie.li
 # @LastEditTime: 2024-11-18 17:12:12
### 

if [ -f ./install/share/ros2_plugin_proto/local_setup.bash ]; then
    source ./install/share/ros2_plugin_proto/local_setup.bash
elif [ -f ../share/ros2_plugin_proto/local_setup.bash ]; then
    source ../share/ros2_plugin_proto/local_setup.bash
fi

sudo setcap cap_net_raw=ep ./aimrt_main

set -m

bash record.sh &
record_pid=$!
pgid=$(ps -o pgid= -p $record_pid | tr -d ' ')
# echo "Child script PGID: $pgid"

./aimrt_main --cfg_file_path=./cfg/x1_cfg.yaml

kill -TERM -$pgid 2>/dev/null
