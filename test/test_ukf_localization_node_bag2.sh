#!/bin/bash

#************************ test_ukf_localization_node_bag2 ***************************
$PWD = `pwd`
echo "Current Working Directory = $PWD"
ROS1_DISTRO=melodic
ROS2_DISTRO=bouncy
echo "ROS1_DISTRO = $ROS1_DISTRO"
echo "ROS2_DISTRO = $ROS2_DISTRO"

#Command to run roscore
cmd1="source /opt/ros/$ROS1_DISTRO/setup.sh; roscore; exec /bin/bash"

#Command torRun ros1_bridge
cmd2="source ~/ros2_ws/install/setup.bash; source /opt/ros/$ROS1_DISTRO/setup.bash; ros2 run ros1_bridge dynamic_bridge --bridge-all-topics; exec /bin/bash"

#Command to play .bag from ROS1
cmd3="source /opt/ros/$ROS1_DISTRO/setup.bash; rosparam set use_sim_time true; rosbag play $PWD/src/robot_localization/test/test2.bag --clock -d 5; exec /bin/bash"

#Command to launch TestCase launch.py
cmd4="source /opt/ros/$ROS2_DISTRO/setup.bash; source $PWD/install/setup.bash; ros2 launch robot_localization test_ukf_localization_node_bag2.launch.py; exec /bin/bash"

gnome-terminal --tab -t "roscore" -- /bin/bash -c "$cmd1" 
sleep 1
gnome-terminal --tab -t "bag" -- /bin/bash -c "$cmd3" 
gnome-terminal --tab -t "TestCase_launch" -- /bin/bash -c "$cmd4"
gnome-terminal --tab -t "ros1_bridge" -- /bin/bash -c "$cmd2"


