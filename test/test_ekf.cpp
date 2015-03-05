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

    Ekf &getFilter()
    {
      return filter_;
    }
};

TEST (EkfTest, Measurements)
{
  RosEkfPassThrough ekf;

  Eigen::MatrixXd initialCovar(15, 15);
  initialCovar.setIdentity();
  initialCovar *= 0.5;
  ekf.getFilter().setEstimateErrorCovariance(initialCovar);

  Eigen::VectorXd measurement(STATE_SIZE);
  for(size_t i = 0; i < STATE_SIZE; ++i)
  {
    measurement[i] = i * 0.01 * STATE_SIZE;
  }

  Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);
  for(size_t i = 0; i < STATE_SIZE; ++i)
  {
    measurementCovariance(i, i) = 1e9;
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

  ekf.integrateMeasurements(1001);

  EXPECT_EQ(ekf.getFilter().getState(), measurement);
  EXPECT_EQ(ekf.getFilter().getEstimateErrorCovariance(), measurementCovariance);

  // Now fuse another measurement and check the output.
  // We know what the filter's state should be when
  // this is complete, so we'll check the difference and
  // make sure it's suitably small.
  Eigen::VectorXd measurement2 = measurement;

  measurement2 *= 2.0;

  for(size_t i = 0; i < STATE_SIZE; ++i)
  {
    measurementCovariance(i, i) = 1e-9;
  }

  time.fromSec(1002);
  ekf.enqueueMeasurement("odom0",
                         measurement2,
                         measurementCovariance,
                         updateVector,
                         std::numeric_limits<double>::max(),
                         time);

  ekf.integrateMeasurements(1003);

  measurement = measurement2.eval() - ekf.getFilter().getState();
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
