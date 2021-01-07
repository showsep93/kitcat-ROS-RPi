#!/bin/bash

#-------CRONTAB-------------
# m h  dom mon dow   command
#@reboot (. ~/.profile; /usr/bin/screen -dmS Kit-Cat /home/ubuntu/catkin_ws/src/Kit-Cat.sh) >> /dev/null 2>&1
#-------CRONTAB-------------

source /opt/ros/kinetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash
sh -c "roslaunch kitcat_llc kitcat.launch"
