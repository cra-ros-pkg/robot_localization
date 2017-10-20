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

#include "robot_localization/filter_base.h"
#include "robot_localization/filter_common.h"

#include <Eigen/Dense>

#include <gtest/gtest.h>

#include <iostream>
#include <queue>
#include <string>

using RobotLocalization::STATE_SIZE;
using RobotLocalization::Measurement;

namespace RobotLocalization
{

class FilterDerived : public FilterBase
{
  public:
    double val;

    FilterDerived() : val(0) { }

    void correct(const Measurement &measurement)
    {
      EXPECT_EQ(val, measurement.time_);
      EXPECT_EQ(measurement.topicName_, "topic");

      EXPECT_EQ(measurement.updateVector_.size(), 10);
      for (size_t i = 0; i < measurement.updateVector_.size(); ++i)
      {
        EXPECT_EQ(measurement.updateVector_[i], true);
      }
    }

    void predict(const double refTime, const double delta)
    {
      val = delta;
    }
};

class FilterDerived2 : public FilterBase
{
  public:
    FilterDerived2() { }

    void correct(const Measurement &measurement)
    {
    }

    void predict(const double refTime, const double delta)
    {
    }

    void processMeasurement(const Measurement &measurement)
    {
      FilterBase::processMeasurement(measurement);
    }
};

}  // namespace RobotLocalization

TEST(FilterBaseTest, MeasurementStruct)
{
    RobotLocalization::Measurement meas1;
    RobotLocalization::Measurement meas2;

    EXPECT_EQ(meas1.topicName_, std::string(""));
    EXPECT_EQ(meas1.time_, 0);
    EXPECT_EQ(meas2.time_, 0);

    // Comparison test is true if the first
    // argument is > the second, so should
    // be false if they're equal.
    EXPECT_EQ(meas1(meas1, meas2), false);
    EXPECT_EQ(meas2(meas2, meas1), false);

    meas1.time_ = 100;
    meas2.time_ = 200;

    EXPECT_EQ(meas1(meas1, meas2), false);
    EXPECT_EQ(meas1(meas2, meas1), true);
    EXPECT_EQ(meas2(meas1, meas2), false);
    EXPECT_EQ(meas2(meas2, meas1), true);
}

TEST(FilterBaseTest, DerivedFilterGetSet)
{
    using RobotLocalization::FilterDerived;

    FilterDerived derived;

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
    derived.setSensorTimeout(timeout);
    EXPECT_EQ(derived.getSensorTimeout(), timeout);

    double lastMeasTime = 3.83;
    derived.setLastMeasurementTime(lastMeasTime);
    EXPECT_EQ(derived.getLastMeasurementTime(), lastMeasTime);

    Eigen::MatrixXd pnCovar(STATE_SIZE, STATE_SIZE);
    for (size_t i = 0; i < STATE_SIZE; ++i)
    {
      for (size_t j = 0; j < STATE_SIZE; ++j)
      {
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

TEST(FilterBaseTest, MeasurementProcessing)
{
  using RobotLocalization::FilterDerived2;

  FilterDerived2 derived;

  Measurement meas;

  Eigen::VectorXd measurement(STATE_SIZE);
  for (size_t i = 0; i < STATE_SIZE; ++i)
  {
    measurement[i] = 0.1 * static_cast<double>(i);
  }

  Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);
  for (size_t i = 0; i < STATE_SIZE; ++i)
  {
    for (size_t j = 0; j < STATE_SIZE; ++j)
    {
      measurementCovariance(i, j) = 0.1 * static_cast<double>(i * j);
    }
  }

  meas.topicName_ = "odomTest";
  meas.measurement_ = measurement;
  meas.covariance_ = measurementCovariance;
  meas.updateVector_.resize(STATE_SIZE, true);
  meas.time_ = 1000;

  // The filter shouldn't be initializedyet
  EXPECT_FALSE(derived.getInitializedStatus());

  derived.processMeasurement(meas);

  // Now it's initialized, and the entire filter state
  // should be equal to the first state
  EXPECT_TRUE(derived.getInitializedStatus());
  EXPECT_EQ(derived.getState(), measurement);

  // Process a measurement and make sure it updates the
  // lastMeasurementTime variable
  meas.time_ = 1002;
  derived.processMeasurement(meas);
  EXPECT_EQ(derived.getLastMeasurementTime(), meas.time_);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
