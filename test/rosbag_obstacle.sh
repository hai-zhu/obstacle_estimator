#! /bin/bash

clear 
source /opt/ros/kinetic/setup.bash

echo "Initialising rosbag of obstacle measurement topics"
echo 

rosbag record -e -q "/Target1/(.*)"

rosbag record -e -q "/Target2/(.*)"

rosbag record -e -q "/Target3/(.*)"
