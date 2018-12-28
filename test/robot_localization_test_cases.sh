
# This script is calling robot_localization_test_cases.py to run all other test cases except bag related test cases in one shot.

PWD=`pwd`
source $PWD/install/setup.bash

/usr/bin/python3 $PWD/src/robot_localization/test/robot_localization_test_cases.py
