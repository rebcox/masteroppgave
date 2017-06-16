#!/bin/bash
# Run master script

echo "Starting master"
source ./devel/setup.bash
export OGRE_RTT_MODE=Copy
roslaunch master master_gazebo2.launch