#!/bin/bash
###
 # @Author: richie.li
 # @Date: 2024-11-18 14:41:20
 # @LastEditors: richie.li
 # @LastEditTime: 2024-11-18 17:48:16
### 

if [ -f ./build/install/share/ros2_plugin_proto/local_setup.bash ]; then
    source ./build/install/share/ros2_plugin_proto/local_setup.bash
elif [ -f ./install/share/ros2_plugin_proto/local_setup.bash ]; then
    source ./install/share/ros2_plugin_proto/local_setup.bash
elif [ -f ../share/ros2_plugin_proto/local_setup.bash ]; then
    source ../share/ros2_plugin_proto/local_setup.bash
fi

current_time=$(date "+%Y-%m-%d_%H:%M:%S")

ros2 launch ros2_utils record.launch.py dir:=./log/ros_bags/$current_time
