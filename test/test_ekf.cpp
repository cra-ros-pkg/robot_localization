#include "robot_localization/ekf.h"

#include <gtest/gtest.h>

TEST (EkfTest, Measurements)
{
  RobotLocalization::Ekf ekf;

  Eigen::VectorXd measurement(12);
  for(size_t i = 0; i < 12; ++i)
  {
    measurement[i] = i;
  }

  Eigen::MatrixXd measurementCovariance(12, 12);
  for(size_t i = 0; i < 12; ++i)
  {
    measurementCovariance(i, i) = 0.5;
  }

  std::vector<int> updateVector(12, true);

  // Ensure that measurements are being placed in the queue correctly
  ekf.enqueueMeasurement("odom0",
                         measurement,
                         measurementCovariance,
                         updateVector,
                         1000);

  std::map<std::string, Eigen::VectorXd> postUpdateStates;

  ekf.integrateMeasurements(1001,
                            postUpdateStates);

  EXPECT_EQ(ekf.getState(), measurement);
  EXPECT_EQ(ekf.getEstimateErrorCovariance(), measurementCovariance);

  // Now fuse another measurement and check the output.
  // We know what the filter's state should be when
  // this is complete, so we'll check the difference and
  // make sure it's suitably small.
  Eigen::VectorXd measurement2 = measurement;

  measurement2 *= 2.0;

  ekf.enqueueMeasurement("odom0",
                         measurement,
                         measurementCovariance,
                         updateVector,
                         1002);

  ekf.integrateMeasurements(1003,
                            postUpdateStates);

  measurement[0] = -2.8975;
  measurement[1] = -0.42068;
  measurement[2] = 5.5751;
  measurement[3] = 2.7582;
  measurement[4] = -2.0858;
  measurement[5] = -2.0859;
  measurement[6] = 3.7596;
  measurement[7] = 4.3694;
  measurement[8] = 5.1206;
  measurement[9] = 9.2408;
  measurement[10] = 9.8034;
  measurement[11] = 11.796;

  measurement = measurement.eval() - ekf.getState();

  for(size_t i = 0; i < 12; ++i)
  {
    EXPECT_LT(::fabs(measurement[i]), 0.001);
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
