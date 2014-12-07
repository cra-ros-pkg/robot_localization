#include "robot_localization/ros_filter_types.h"
#include <limits>
#include <gtest/gtest.h>

using namespace RobotLocalization;

class RosEkfPassThrough : public RosEkf
{
  public:
    RosEkfPassThrough()
    {
    }

    Ekf getFilter()
    {
      return filter_;
    }
};

TEST (EkfTest, Measurements)
{
  RosEkfPassThrough ekf;

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
  ros::Time time;
  time.fromSec(1000);
  ekf.enqueueMeasurement("odom0",
                         measurement,
                         measurementCovariance,
                         updateVector,
                         std::numeric_limits<double>::max(),
                         time);

  std::map<std::string, Eigen::VectorXd> postUpdateStates;

  ekf.integrateMeasurements(1001);

  EXPECT_EQ(ekf.getFilter().getState(), measurement);
  EXPECT_EQ(ekf.getFilter().getEstimateErrorCovariance(), measurementCovariance);

  // Now fuse another measurement and check the output.
  // We know what the filter's state should be when
  // this is complete, so we'll check the difference and
  // make sure it's suitably small.
  Eigen::VectorXd measurement2 = measurement;

  measurement2 *= 2.0;

  time.fromSec(1002);
  ekf.enqueueMeasurement("odom0",
                         measurement2,
                         measurementCovariance,
                         updateVector,
                         std::numeric_limits<double>::max(),
                         time);

  ekf.integrateMeasurements(1003);

  measurement[0] = -4.5198;
  measurement[1] = 0.14655;
  measurement[2] = 9.4514;
  measurement[3] = -2.8688;
  measurement[4] = -2.2672;
  measurement[5] = 0.12861;
  measurement[6] = 15.481;
  measurement[7] = 17.517;
  measurement[8] = 19.587;
  measurement[9] = 9.8351;
  measurement[10] = 12.73;
  measurement[11] = 13.87;
  measurement[12] = 10.978;
  measurement[13] = 12.008;
  measurement[14] = 13.126;

  measurement = measurement.eval() - ekf.getFilter().getState();

  for(size_t i = 0; i < STATE_SIZE; ++i)
  {
    EXPECT_LT(::fabs(measurement[i]), 0.001);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ekf");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
