#!/bin/bash

./stat_recorder.sh test_ekf_localization_node_bag1.test $1/ekf1.txt
./stat_recorder.sh test_ekf_localization_node_bag2.test $1/ekf2.txt
./stat_recorder.sh test_ekf_localization_node_bag3.test $1/ekf3.txt

./stat_recorder.sh test_ukf_localization_node_bag1.test $1/ukf1.txt
./stat_recorder.sh test_ukf_localization_node_bag2.test $1/ukf2.txt
./stat_recorder.sh test_ukf_localization_node_bag3.test $1/ukf3.txt
