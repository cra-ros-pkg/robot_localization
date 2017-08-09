/*
 * Copyright (c) 2014, 2015, 2016, Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "robot_localization/ros_filter_types.h"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

using RobotLocalization::Ekf;
using RobotLocalization::RosEkf;
using RobotLocalization::STATE_SIZE;

class RosEkfPassThrough : public RosEkf
{
  public:
    RosEkfPassThrough() : RosEkf(ros::NodeHandle(), ros::NodeHandle("~"))
    {
    }

    Ekf &getFilter()
    {
      return filter_;
    }
};

TEST(EkfTest, Measurements)
{
  RosEkfPassThrough ekf;

  Eigen::MatrixXd initialCovar(15, 15);
  initialCovar.setIdentity();
  initialCovar *= 0.5;
  ekf.getFilter().setEstimateErrorCovariance(initialCovar);

  Eigen::VectorXd measurement(STATE_SIZE);
  measurement.setIdentity();
  for (size_t i = 0; i < STATE_SIZE; ++i)
  {
    measurement[i] = i * 0.01 * STATE_SIZE;
  }

  Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);
  measurementCovariance.setIdentity();
  for (size_t i = 0; i < STATE_SIZE; ++i)
  {
    measurementCovariance(i, i) = 1e-9;
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

  ekf.integrateMeasurements(ros::Time(1001));

  EXPECT_EQ(ekf.getFilter().getState(), measurement);
  EXPECT_EQ(ekf.getFilter().getEstimateErrorCovariance(), measurementCovariance);

  ekf.getFilter().setEstimateErrorCovariance(initialCovar);

  // Now fuse another measurement and check the output.
  // We know what the filter's state should be when
  // this is complete, so we'll check the difference and
  // make sure it's suitably small.
  Eigen::VectorXd measurement2 = measurement;

  measurement2 *= 2.0;

  for (size_t i = 0; i < STATE_SIZE; ++i)
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

  ekf.integrateMeasurements(ros::Time(1003));

  measurement = measurement2.eval() - ekf.getFilter().getState();
  for (size_t i = 0; i < STATE_SIZE; ++i)
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
