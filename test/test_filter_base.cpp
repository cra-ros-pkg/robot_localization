#include "robot_localization/filter_base.h"
#include "robot_localization/filter_common.h"

#include <Eigen/Dense>

#include <gtest/gtest.h>
#include <queue>
#include <iostream>

namespace RobotLocalization
{
  class FilterDerived : public FilterBase
  {
    public:

      double val;

      FilterDerived() : val(0) { }

      void correct(const Measurement &measurement)
      {
        EXPECT_EQ (val, measurement.time_);
        EXPECT_EQ (measurement.topicName_, "topic");

        EXPECT_EQ (measurement.updateVector_.size(), 10);
        for(size_t i = 0; i < measurement.updateVector_.size(); ++i)
        {
          EXPECT_EQ (measurement.updateVector_[i], true);
        }
      }

      void predict(const double delta)
      {
        val = delta;
      }

      std::priority_queue<Measurement, std::vector<Measurement>, Measurement> getMeasurementQueue()
      {
        return measurementQueue_;
      }
  };

  class FilterDerived2 : public FilterBase
  {
    public:

      FilterDerived2() { }

      void correct(const Measurement &measurement)
      {

      }

      void predict(const double delta)
      {

      }

      void processMeasurement(const Measurement &measurement)
      {
        FilterBase::processMeasurement(measurement);
      }

      void integrateMeasurements(double currentTime,
                                 std::map<std::string, Eigen::VectorXd> &postUpdateStates)
      {
        FilterBase::integrateMeasurements(currentTime, postUpdateStates);
      }
  };
}

TEST (FilterBaseTest, MeasurementStruct)
{
    RobotLocalization::Measurement meas1;
    RobotLocalization::Measurement meas2;

    EXPECT_EQ (meas1.topicName_, std::string(""));
    EXPECT_EQ (meas1.time_, 0);
    EXPECT_EQ (meas2.time_, 0);

    // Comparison test is true if the first
    // argument is > the second, so should
    // be false if they're equal.
    EXPECT_EQ (meas1(meas1, meas2), false);
    EXPECT_EQ (meas2(meas2, meas1), false);

    meas1.time_ = 100;
    meas2.time_ = 200;

    EXPECT_EQ (meas1(meas1, meas2), false);
    EXPECT_EQ (meas1(meas2, meas1), true);
    EXPECT_EQ (meas2(meas1, meas2), false);
    EXPECT_EQ (meas2(meas2, meas1), true);
}

TEST (FilterBaseTest, DerivedFilterGetSet)
{
    using namespace RobotLocalization;

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

    double lastUpdateTime = 5.1;
    derived.setLastUpdateTime(lastUpdateTime);
    EXPECT_EQ(derived.getLastUpdateTime(), lastUpdateTime);

    double lastMeasTime = 3.83;
    derived.setLastMeasurementTime(lastMeasTime);
    EXPECT_EQ(derived.getLastMeasurementTime(), lastMeasTime);

    Eigen::MatrixXd pnCovar(STATE_SIZE, STATE_SIZE);
    for(size_t i = 0; i < STATE_SIZE; ++i)
    {
      for(size_t j = 0; j < STATE_SIZE; ++j)
      {
        pnCovar(i, j) = static_cast<double>(i * j);
      }
    }
    derived.setProcessNoiseCovariance(pnCovar);
    EXPECT_EQ(derived.getProcessNoiseCovariance(), pnCovar);

    Eigen::VectorXd state(STATE_SIZE);
    derived.setState(state);
    EXPECT_EQ(derived.getState(), state);

    EXPECT_EQ(derived.getInitializedStatus(), false);
}

TEST (FilterBaseTest, DerivedFilterEnqueue)
{
    using namespace RobotLocalization;

    FilterDerived derived;

    Eigen::VectorXd measurement(STATE_SIZE);
    for(size_t i = 0; i < STATE_SIZE; ++i)
    {
      measurement[i] = static_cast<double>(i);
    }

    Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);
    for(size_t i = 0; i < STATE_SIZE; ++i)
    {
      for(size_t j = 0; j < STATE_SIZE; ++j)
      {
        measurementCovariance(i, j) = static_cast<double>(i * j);
      }
    }

    std::vector<int> updateVector(STATE_SIZE, true);
    updateVector[5] = false;

    // Ensure that measurements are being placed in the queue correctly
    derived.enqueueMeasurement("odom0",
                                measurement,
                                measurementCovariance,
                                updateVector,
                                1000);

    std::priority_queue<RobotLocalization::Measurement, std::vector<RobotLocalization::Measurement>, RobotLocalization::Measurement> measQueue = derived.getMeasurementQueue();
    Measurement meas = measQueue.top();
    EXPECT_EQ(meas.measurement_, measurement);
    EXPECT_EQ(meas.covariance_, measurementCovariance);
    EXPECT_EQ(meas.updateVector_, updateVector);

    // Enqueue a second measurement with an earlier time. Verify that it's at the
    // top of the queue.
    measurement(0) = -5;
    derived.enqueueMeasurement("odom0",
                                measurement,
                                measurementCovariance,
                                updateVector,
                                999);

    measQueue = derived.getMeasurementQueue();
    meas = measQueue.top();
    EXPECT_EQ(meas.measurement_(0), measurement(0));
}

TEST (FilterBaseTest, MeasurementProcessing)
{
  using namespace RobotLocalization;

  FilterDerived2 derived;

  Measurement meas;

  Eigen::VectorXd measurement(STATE_SIZE);
  for(size_t i = 0; i < STATE_SIZE; ++i)
  {
    measurement[i] = 0.1 * static_cast<double>(i);
  }

  Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);
  for(size_t i = 0; i < STATE_SIZE; ++i)
  {
    for(size_t j = 0; j < STATE_SIZE; ++j)
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

  // Initialize a new one
  FilterDerived2 derived2;

  std::vector<int> updateVector(STATE_SIZE, 1);

  // Enqueue two measurements and integrate them
  derived2.enqueueMeasurement("odom0",
                              measurement,
                              measurementCovariance,
                              updateVector,
                              999);

  derived2.enqueueMeasurement("odom0",
                              measurement,
                              measurementCovariance,
                              updateVector,
                              1000);

  std::map<std::string, Eigen::VectorXd> postUpdateStates;
  derived2.integrateMeasurements(1001, postUpdateStates);

  // Make sure the update times were set correctly
  EXPECT_TRUE(derived.getInitializedStatus());
  EXPECT_EQ(derived2.getLastUpdateTime(), 1001);
  EXPECT_EQ(derived2.getLastMeasurementTime(), 1000);

  // Now enqueue a third and integrate again
  derived2.enqueueMeasurement("odom0",
                              measurement,
                              measurementCovariance,
                              updateVector,
                              1002);

  derived2.integrateMeasurements(1003, postUpdateStates);

  EXPECT_EQ(derived2.getLastMeasurementTime(), 1002);
  EXPECT_EQ(derived2.getLastUpdateTime(), 1003);

  // Set the sensor timeout, and then integrate without
  // enqueueing. This will cause a predict/update cycle
  // and update the last update and measurement times.
  derived2.setSensorTimeout(0.5);
  derived2.integrateMeasurements(1003.52, postUpdateStates);
  EXPECT_EQ(derived2.getLastUpdateTime(), 1003.50);
  EXPECT_EQ(derived2.getLastMeasurementTime(), 1002.50);

  // Enqueue a measurement that's in the past and make sure
  // it doesn't change the last measurement time, but does
  // change the last update time.
  derived2.enqueueMeasurement("odom0",
                              measurement,
                              measurementCovariance,
                              updateVector,
                              1000);

  derived2.integrateMeasurements(1004, postUpdateStates);
  EXPECT_EQ(derived2.getLastMeasurementTime(), 1002.50);
  EXPECT_EQ(derived2.getLastUpdateTime(), 1004);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
