#!/usr/bin/env zsh
source ~/unitree_ros2/setup_local.sh
source ~/unitree_ros2/install/setup.sh
source timed_topics/install/setup.sh

export ROS_DOMAIN_ID=1  

ros2 run timed_topics republish_node
