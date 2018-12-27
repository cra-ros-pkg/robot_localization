
**This file describes steps to perform test on the robot localization package for ROS2 (bouncy) and other required stuff.**

**Overview:**

	robot_localization is a package of nonlinear state estimation nodes.

**List of changes from ROS to ROS2:**

	The changes has been done by following the Migration guide for ROS2( https://index.ros.org/doc/ros2/Migration-Guide/ ).

	1. Migrated all test.cpp into ROS2 style
	2. Migrated yaml files into ROS2 style
	3. Migrated .test files into launch.py in ROS2 style
	4. Migrated CMakeLists.txt and package.xml
	5. Created Automated scripts in order to run test cases except bag related test cases.
	6. Added .travis.yml file in order to check for CI build
	7. Followed cosmetics rules and updated the files with cppcheck, cpplint and uncrustify rules.

**Pre-requisites:**

	1. System should have installed bouncy distro. Check out installation instructions and tutorials https://index.ros.org/doc/ros2/.
	AGAIN PLEASE NOTE THAT IN ORDER TO USE THE AUTOMATED SCRIPTS, YOU HAVE TO FOLLOW THE SAME DIRECTORY STRUCTURE PROVIDED IN THIS README.
	2. User should checkout launch pkg & build (Refer steps below).

		Steps to checkout launch pkg (master) & build:

			1. mkdir -p rl_ws/src
			2. cd ~/rl_ws/src
			3. git clone git@github.com:ros2/launch.git
			4. cd ~/rl_ws 
			5. source /opt/ros/bouncy/setup.bash
			6. colcon build

	3. System should have checkout ros2 robot_localization pkg & build (Refer Steps below).

		Steps to checkout robot_localization pkg & build:

			1. cd ~/rl_ws/src
			2. git clone git@github.com:Rohita83/robot_localization.git
			#git branch -ra //This command will show different branches of main git on #kinetic
			3. cd robot_localization/
			4. git checkout -b ros2 remotes/origin/ros2
			5. cd ~/rl_ws
			6. source /opt/ros/bouncy/setup.bash
			7. colcon build

	4. System should have both ROS1 & ROS2. Note: We have verified on ROS1(i.e. melodic) & ROS2 (i.e. bouncy).
	5. ros1_bridge pkg should be available in ros2 workspace.

**Limitations:-** 

	1. colcon test does not work as launch.py files can not be executed/added with CMakeLists.txt as of now.
	2. Each test/launch.py files have been tested independently.
	3. Use of ros1 bridge to play rosbag files as rosbag is not available in ros2.
	4. Bag related testcases launch files will not terminate automatically i.e. user will have to do manually ctrl+C after bag duration finish.

**Future Work:-**

	1. All Test launch.py files should be executed using "colcon test" automatically. Currently we have built automated script "robot_localization_test_cases.sh" which performs all the test cases except bag related test cases.
	2. Remove the dependency with ros1_bridge to play bag file from ROS1 using rosbag.

**TESTING**

There are two ways to run the test cases:

	1. Using automated scripts
	The script files are placed in the /test folder of robot localization.
	Go to the test folder path and run the shell scripts provided that you have downloaded the git source code explained above.
	Do not forget to source the install "source install/setup.bash"
	For example:-
	./robot_localization_test_cases.sh -> to run all other test cases except bag related test cases
	./test_ekf_localization_node_bag1.sh -> to run the bag1 ekf test case
	With bag related scripts, multiple terminals will be opened, check the terminal on which bag is playing.Once is bag stoped, chekck the terminal on which launch.py is running, you will see, test cases are passing after pressing ctrl+c.

	2. Using launch files independently
	Below are the steps to run the test case independently using launch files.
	In order to run the UKF related test cases, follow same steps as of EKF. Just replace ekf with ukf.
	e.g. test_ukf_localization_node_bag1.launch.py

1)**test_ekf_localization_node_bag1.launch.py**

	Terminal1:-
	source /opt/ros/melodic/setup.bash
	roscore

	Terminal2:- (Run ros1_bridge):-
	source ~/ros2_ws/install/setup.bash
	source /opt/ros/melodic/setup.bash
	ros2 run ros1_bridge dynamic_bridge --bridge-all-topics	

	Terminal3:- (Play .bag from ROS1):-
	source /opt/ros/melodic/setup.bash
	rosparam set /use_sim_time true
	rosbag play ~/rl_ws/src/robot_localization/test/test1.bag --clock -d 5

	Terminal4:- (Launch TestCase launch.py):-
	source /opt/ros/bouncy/setup.bash
	source ~/rl_ws/install/setup.bash
	ros2 launch robot_localization test_ekf_localization_node_bag1.launch.py

	Terminal5:- (Run static_transform_publisher):-
	source /opt/ros/bouncy/setup.bash
	ros2 run tf2_ros static_transform_publisher 0 -0.3 0.52 -1.570796327 0 1.570796327 base_link imu_link


2)**test_ekf_localization_node_bag2.launch.py**

	Terminal1:-
	source /opt/ros/melodic/setup.bash
	roscore

	Terminal2:- (Run ros1_bridge):-
	source ~/ros2_ws/install/setup.bash
	source /opt/ros/melodic/setup.bash
	ros2 run ros1_bridge dynamic_bridge --bridge-all-topics	

	Terminal3:- (Play .bag from ROS1):-
	source /opt/ros/melodic/setup.bash
	rosparam set /use_sim_time true
	rosbag play ~/rl_ws/src/robot_localization/test/test2.bag --clock -d 5

	Terminal4:- (Launch TestCase launch.py):-
	source /opt/ros/bouncy/setup.bash
	source ~/rl_ws/install/setup.bash
	ros2 launch robot_localization test_ekf_localization_node_bag2.launch.py

3)**test_ekf_localization_node_bag3.launch.py**

	Terminal1:-
	source /opt/ros/melodic/setup.bash
	roscore

	Terminal2:- (Run ros1_bridge):-
	source ~/ros2_ws/install/setup.bash
	source /opt/ros/melodic/setup.bash
	ros2 run ros1_bridge dynamic_bridge --bridge-all-topics	

	Terminal3:- (Play .bag from ROS1):-
	source /opt/ros/melodic/setup.bash
	rosparam set /use_sim_time true
	rosbag play ~/rl_ws/src/robot_localization/test/test3.bag --clock -d 5

	Terminal4:- (Launch TestCase launch.py):-
	source /opt/ros/bouncy/setup.bash
	source ~/rl_ws/install/setup.bash
	ros2 launch robot_localization test_ekf_localization_node_bag3.launch.py

4)**test_ekf_localization_node_interfaces.launch.py**

	Terminal1:- (Launch TestCase .launch.py)
	source /opt/ros/bouncy/setup.bash
	source ~/rl_ws/install/setup.bash
	ros2 launch robot_localization test_ekf_localization_node_interfaces.launch.py

5)**test_ekf.launch.py**

	Terminal1:- (Launch TestCase .launch.py)
	source /opt/ros/bouncy/setup.bash
	source ~/rl_ws/install/setup.bash
	ros2 launch robot_localization test_ekf.launch.py

