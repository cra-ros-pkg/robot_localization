/*
 * Copyright (c) 2014, 2015, 2016 Charles River Analytics, Inc.
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

#include <Eigen/Dense>

#include <gtest/gtest.h>

#include <iostream>
#include <queue>
#include <string>

#include "robot_localization/filter_common.hpp"
#include "robot_localization/filter_utilities.hpp"
#include "robot_localization/filter_base.hpp"
#include "robot_localization/measurement.hpp"

using robot_localization::Measurement;
using robot_localization::STATE_SIZE;

namespace robot_localization
{

class FilterDerived : public FilterBase
{
public:
  rclcpp::Time val;

  FilterDerived()
  : val(0) {}

  void correct(const Measurement & measurement)
  {
    EXPECT_EQ(val, measurement.time_);
    EXPECT_EQ(measurement.topic_name_, "topic");

    EXPECT_EQ(measurement.update_vector_.size(), 10u);
    for (size_t i = 0; i < measurement.update_vector_.size(); ++i) {
      EXPECT_EQ(measurement.update_vector_[i], true);
    }
  }
  void predict(
    const rclcpp::Time & /*reference_time*/,
    const rclcpp::Duration & /*delta*/) {}
};

class FilterDerived2 : public FilterBase
{
public:
  FilterDerived2() {}

  void correct(const Measurement & /*measurement*/) {}

  void predict(
    const rclcpp::Time & /*reference_time*/,
    const rclcpp::Duration & /*delta*/) {}

  void processMeasurement(const Measurement & measurement)
  {
    FilterBase::processMeasurement(measurement);
  }
};

}  // namespace robot_localization

TEST(FilterBaseTest, MeasurementStruct) {
  robot_localization::Measurement meas1;
  robot_localization::Measurement meas2;

  EXPECT_EQ(meas1.topic_name_, std::string(""));
  EXPECT_EQ(meas1.time_, rclcpp::Time(0));
  EXPECT_EQ(meas2.time_, rclcpp::Time(0));

  // Comparison test is true if the first
  // argument is > the second, so should
  // be false if they're equal.
  EXPECT_EQ(meas1(meas1, meas2), false);
  EXPECT_EQ(meas2(meas2, meas1), false);

  builtin_interfaces::msg::Time msg1;
  msg1.sec = 0;
  msg1.nanosec = 100;

  builtin_interfaces::msg::Time msg2;
  msg2.sec = 0;
  msg2.nanosec = 200;

  meas1.time_ = msg1;
  meas2.time_ = msg2;

  EXPECT_EQ(meas1(meas1, meas2), false);
  EXPECT_EQ(meas1(meas2, meas1), true);
  EXPECT_EQ(meas2(meas1, meas2), false);
  EXPECT_EQ(meas2(meas2, meas1), true);
}

TEST(FilterBaseTest, DerivedFilterGetSet) {
  robot_localization::FilterDerived derived;

  // With the ostream argument as NULL,
  // the debug flag will remain false.
  derived.setDebug(true);

  EXPECT_FALSE(derived.getDebug());

  // Now set the stream and do it again
  std::stringstream os;
  derived.setDebug(true, &os);

  EXPECT_TRUE(derived.getDebug());

  // Simple get/set checks
  double timeout = 7.4;
  derived.setSensorTimeout(rclcpp::Duration::from_seconds(timeout));
  EXPECT_EQ(derived.getSensorTimeout(), rclcpp::Duration::from_seconds(timeout));

  double lastMeasTime = 3.83;
  derived.setLastMeasurementTime(rclcpp::Time(lastMeasTime));
  EXPECT_EQ(derived.getLastMeasurementTime(), rclcpp::Time(lastMeasTime));

  Eigen::MatrixXd pnCovar(STATE_SIZE, STATE_SIZE);
  for (size_t i = 0; i < STATE_SIZE; ++i) {
    for (size_t j = 0; j < STATE_SIZE; ++j) {
      pnCovar(i, j) = static_cast<double>(i * j);
    }
  }
  derived.setProcessNoiseCovariance(pnCovar);
  EXPECT_EQ(derived.getProcessNoiseCovariance(), pnCovar);

  Eigen::VectorXd state(STATE_SIZE);
  state.setZero();
  derived.setState(state);
  EXPECT_EQ(derived.getState(), state);

  EXPECT_EQ(derived.getInitializedStatus(), false);
}

TEST(FilterBaseTest, MeasurementProcessing) {
  robot_localization::FilterDerived2 derived;

  Measurement meas;

  Eigen::VectorXd measurement(STATE_SIZE);
  for (size_t i = 0; i < STATE_SIZE; ++i) {
    measurement[i] = 0.1 * static_cast<double>(i);
  }

  Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);
  for (size_t i = 0; i < STATE_SIZE; ++i) {
    for (size_t j = 0; j < STATE_SIZE; ++j) {
      measurementCovariance(i, j) = 0.1 * static_cast<double>(i * j);
    }
  }

  builtin_interfaces::msg::Time msg;
  msg.sec = 0;
  msg.nanosec = 1000;

  meas.topic_name_ = "odomTest";
  meas.measurement_ = measurement;
  meas.covariance_ = measurementCovariance;
  meas.update_vector_.resize(STATE_SIZE, true);
  meas.time_ = msg;

  // The filter shouldn't be initializedyet
  EXPECT_FALSE(derived.getInitializedStatus());

  derived.processMeasurement(meas);

  // Now it's initialized, and the entire filter state
  // should be equal to the first state
  EXPECT_TRUE(derived.getInitializedStatus());
  EXPECT_EQ(derived.getState(), measurement);

  msg.nanosec = 1002;
  // Process a measurement and make sure it updates the
  // lastMeasurementTime variable
  meas.time_ = msg;
  derived.processMeasurement(meas);
  EXPECT_EQ(derived.getLastMeasurementTime(), meas.time_);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
