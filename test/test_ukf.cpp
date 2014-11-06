#include "robot_localization/ukf.h"
#include "robot_localization/filter_common.h"

#include <gtest/gtest.h>

using namespace RobotLocalization;

TEST (UkfTest, Measurements)
{
  std::vector<double> args;
  args.push_back(0.001);
  args.push_back(0);
  args.push_back(2);
  Ukf ukf(args);

  Eigen::VectorXd measurement(STATE_SIZE);
  for(size_t i = 0; i < STATE_SIZE; ++i)
  {
    measurement[i] = i;
  }

  Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);
  for(size_t i = 0; i < STATE_SIZE; ++i)
  {
    measurementCovariance(i, i) = 0.5;
  }

  std::vector<int> updateVector(STATE_SIZE, true);

  // Ensure that measurements are being placed in the queue correctly
  ukf.enqueueMeasurement("odom0",
                         measurement,
                         measurementCovariance,
                         updateVector,
                         1000);

  std::map<std::string, Eigen::VectorXd> postUpdateStates;

  ukf.integrateMeasurements(1001,
                            postUpdateStates);

  EXPECT_EQ(ukf.getState(), measurement);
  EXPECT_EQ(ukf.getEstimateErrorCovariance(), measurementCovariance);

  // Now fuse another measurement and check the output.
  // We know what the filter's state should be when
  // this is complete, so we'll check the difference and
  // make sure it's suitably small.
  Eigen::VectorXd measurement2 = measurement;

  measurement2 *= 2.0;

  ukf.enqueueMeasurement("odom0",
                         measurement2,
                         measurementCovariance,
                         updateVector,
                         1002);

  ukf.integrateMeasurements(1003,
                            postUpdateStates);

  measurement[0] = -5.5142;
  measurement[1] = -0.91698;
  measurement[2] = 10.304;
  measurement[3] = -2.1372;
  measurement[4] = 0.36284;
  measurement[5] = 2.8628;
  measurement[6] = 15.535;
  measurement[7] = 16.659;
  measurement[8] = 18.634;
  measurement[9] = 9.0708;
  measurement[10] = 10.071;
  measurement[11] = 11.071;
  measurement[12] = 10.5;
  measurement[13] = 11.5;
  measurement[14] = 12.5;

  measurement = measurement.eval() - ukf.getState();

  for(size_t i = 0; i < STATE_SIZE; ++i)
  {
    EXPECT_LT(::fabs(measurement[i]), 0.001);
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
