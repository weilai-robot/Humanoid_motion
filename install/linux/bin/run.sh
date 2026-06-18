#!/bin/bash

if [ -f ./install/share/ros2_plugin_proto/local_setup.bash ]; then
    source ./install/share/ros2_plugin_proto/local_setup.bash
elif [ -f ../share/ros2_plugin_proto/local_setup.bash ]; then
    source ../share/ros2_plugin_proto/local_setup.bash
fi

sudo setcap cap_net_raw=ep ./aimrt_main
./aimrt_main --cfg_file_path=./cfg/x1_cfg.yaml
