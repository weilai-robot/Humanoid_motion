#!/bin/bash

if [ -f ./build/install/share/ros2_plugin_proto/local_setup.bash ]; then
    source ./build/install/share/ros2_plugin_proto/local_setup.bash
elif [ -f ./install/share/ros2_plugin_proto/local_setup.bash ]; then
    source ./install/share/ros2_plugin_proto/local_setup.bash
elif [ -f ../share/ros2_plugin_proto/local_setup.bash ]; then
    source ../share/ros2_plugin_proto/local_setup.bash
fi
