#include "robot_localization/ekf.h"

#include <gtest/gtest.h>

TEST (EkfTest, MeasurementStruct) {
    RobotLocalization::Ekf ekf;
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
