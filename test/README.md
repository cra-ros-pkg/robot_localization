
**********This file describes the work done and steps to perform test on the robot localization package.**********

"https://github.com/Rohita83/robot_localization"

Work Done by referring ROS1 kinetic branch robot localization test folder.
1)Migrated all test.cpp into ROS2 style
2)Migrated yaml files into ROS2 style
3)Migrated .test files into launch.py in ROS2 style

NOTE:- 
1)Consider master branch of launch package to test .launch.py files.
2)colcon test does not work as launch.py files can not be executed/added with CMakeLists.txt as of now.
3)Each test/launch.py files have been tested independently.
4)Use of ros1 bridge to play rosbag files as rosbag is not available in ros2.

Pre-requisites: 
1. System should have checkout ros2 robot_localization pkg & build (Refer Steps below).
2. User should checkout launch (master) pkg & build (Refer steps below).
3. ros1_bridge pkg should be available in ros2 workspace.
4. System should have Both ROS1 & ROS2. Note: We have verified on ROS1(i.e. melodic) & ROS2 (i.e. Bouncy).

Future Work: 
1. All Test launch.py files should be execute using "colcon test" automatially. 
2. Remove the dependency with ros1_bridge to play bag file from ROS1 using rosbag.

==========================================================
****Steps to checkout robot_localization pkg & build****

mkdir -p rl_ws/src
cd ~/rl_ws/src

git clone git@github.com:Rohita83/robot_localization.git
#git branch -ra //This command will show different branches of main git on #kinetic
cd robot_localization/
git checkout -b ros2 remotes/origin/ros2
cd ~/rl_ws
source /opt/ros/bouncy/setup.bash
colcon build

=========================================================
*****Steps to checkout launch pkg (master) & build****
cd ~/rl_ws/src
git clone git@github.com:ros2/launch.git
cd ~/rl_ws
source /opt/ros/bouncy/setup.bash
colcon build

=========================================================

*******************TESTING:***************************

There are two ways to run the test cases:
1) Using automated scripts
The script files are placed in the /test folder of robot localization.
Go to the test folder path and run the bash scripts provided that you have downloaded the git source code explained above.

2) Using launch files independently
Below are the steps to run the test case independently using launch files.
In order to run the UKF related test cases, follow same steps as of EKF. Just replace ekf with ukf.
e.g. test_ukf_localization_node_bag1.launch.py

1)*******test_ekf_localization_node_bag1.launch.py**********

Terminal1:-
source /opt/ros/melodic/setup.bash
roscore

Termianl2:- (Run ros1_bridge):-
source ~/ros2_ws/install/setup.bash
source /opt/ros/melodic/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics	

Termianl3:- (Play .bag from ROS1):-
source /opt/ros/melodic/setup.bash
rosparam set /use_sim_time true
rosbag play ~/rl_ws/src/robot_localization/test/test1.bag --clock -d 5

Termianl4:- (Launch TestCase launch.py):-
source /opt/ros/bouncy/setup.bash
source ~/rl_ws/install/setup.bash
ros2 launch robot_localization test_ekf_localization_node_bag1.launch.py

Termianl5:- (Run static_transform_publisher):-
source /opt/ros/bouncy/setup.bash
ros2 run tf2_ros static_transform_publisher 0 -0.3 0.52 -1.570796327 0 1.570796327 base_link imu_link


2)*******test_ekf_localization_node_bag2.launch.py**********

Terminal1:-
source /opt/ros/melodic/setup.bash
roscore

Termianl2:- (Run ros1_bridge):-
source ~/ros2_ws/install/setup.bash
source /opt/ros/melodic/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics	

Termianl3:- (Play .bag from ROS1):-
source /opt/ros/melodic/setup.bash
rosparam set /use_sim_time true
rosbag play ~/rl_ws/src/robot_localization/test/test2.bag --clock -d 5

Termianl4:- (Launch TestCase launch.py):-
source /opt/ros/bouncy/setup.bash
source ~/rl_ws/install/setup.bash
ros2 launch robot_localization test_ekf_localization_node_bag2.launch.py

3)*******test_ekf_localization_node_bag3.launch.py**********

Terminal1:-
source /opt/ros/melodic/setup.bash
roscore

Termianl2:- (Run ros1_bridge):-
source ~/ros2_ws/install/setup.bash
source /opt/ros/melodic/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics	

Termianl3:- (Play .bag from ROS1):-
source /opt/ros/melodic/setup.bash
rosparam set /use_sim_time true
rosbag play ~/rl_ws/src/robot_localization/test/test3.bag --clock -d 5

Termianl4:- (Launch TestCase launch.py):-
source /opt/ros/bouncy/setup.bash
source ~/rl_ws/install/setup.bash
ros2 launch robot_localization test_ekf_localization_node_bag3.launch.py

4)*******test_ekf_localization_node_interfaces.launch.py**********

Terminal1:- (Launch TestCase .launch.py)
source /opt/ros/bouncy/setup.bash
source ~/rl_ws/install/setup.bash
ros2 launch robot_localization test_ekf_localization_node_interfaces.launch.py

5)*******test_ekf.launch.py**********

Terminal1:- (Launch TestCase .launch.py)
source /opt/ros/bouncy/setup.bash
source ~/rl_ws/install/setup.bash
ros2 launch robot_localization test_ekf.launch.py

