#include "robot_localization/filter_base.h"

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
    RobotLocalization::FilterDerived derived;

    // With the ostream argument as NULL,
    // the debug flag will remain false.
    derived.setDebug(true);

    EXPECT_FALSE(derived.getDebug());

    std::stringstream os;
    derived.setDebug(true, &os);

    EXPECT_TRUE(derived.getDebug());

    double timeout = 7.4;
    derived.setSensorTimeout(timeout);
    EXPECT_EQ(derived.getSensorTimeout(), timeout);

    double lastUpdateTime = 5.1;
    derived.setLastUpdateTime(lastUpdateTime);
    EXPECT_EQ(derived.getLastUpdateTime(), lastUpdateTime);

    double lastMeasTime = 3.83;
    derived.setLastMeasurementTime(lastMeasTime);
    EXPECT_EQ(derived.getLastMeasurementTime(), lastMeasTime);

    Eigen::MatrixXd pnCovar(12, 12);
    for(size_t i = 0; i < 12; ++i)
    {
      for(size_t j = 0; j < 12; ++j)
      {
        pnCovar(i, j) = static_cast<double>(i * j);
      }
    }
    derived.setProcessNoiseCovariance(pnCovar);
    EXPECT_EQ(derived.getProcessNoiseCovariance(), pnCovar);

    Eigen::VectorXd state(12);
    derived.setState(state);
    EXPECT_EQ(derived.getState(), state);

    EXPECT_EQ(derived.getInitializedStatus(), false);
}

TEST (FilterBaseTest, DerivedFilterEnqueue)
{
    RobotLocalization::FilterDerived derived;

    Eigen::VectorXd measurement(12);
    for(size_t i = 0; i < 12; ++i)
    {
      measurement[i] = static_cast<double>(i);
    }

    Eigen::MatrixXd measurementCovariance(12, 12);
    for(size_t i = 0; i < 12; ++i)
    {
      for(size_t j = 0; j < 12; ++j)
      {
        measurementCovariance(i, j) = static_cast<double>(i * j);
      }
    }

    std::vector<int> updateVector(12, true);
    updateVector[5] = false;

    derived.enqueueMeasurement("odom0",
                                measurement,
                                measurementCovariance,
                                updateVector,
                                1000);

    std::priority_queue<RobotLocalization::Measurement, std::vector<RobotLocalization::Measurement>, RobotLocalization::Measurement> measQueue = derived.getMeasurementQueue();
    RobotLocalization::Measurement meas = measQueue.top();
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
  RobotLocalization::FilterDerived2 derived;

  RobotLocalization::Measurement meas;

  Eigen::VectorXd measurement(12);
  for(size_t i = 0; i < 12; ++i)
  {
    measurement[i] = 0.1 * static_cast<double>(i);
  }

  Eigen::MatrixXd measurementCovariance(12, 12);
  for(size_t i = 0; i < 12; ++i)
  {
    for(size_t j = 0; j < 12; ++j)
    {
      measurementCovariance(i, j) = 0.1 * static_cast<double>(i * j);
    }
  }

  meas.topicName_ = "odomTest";
  meas.measurement_ = measurement;
  meas.covariance_ = measurementCovariance;
  meas.updateVector_.resize(12, true);
  meas.time_ = 1000;

  EXPECT_FALSE(derived.getInitializedStatus());

  derived.processMeasurement(meas);

  EXPECT_TRUE(derived.getInitializedStatus());
  EXPECT_EQ(derived.getState(), measurement);

  meas.time_ = 1002;
  derived.processMeasurement(meas);
  EXPECT_EQ(derived.getLastMeasurementTime(), meas.time_);

  RobotLocalization::FilterDerived2 derived2;

  std::vector<int> updateVector(12, 1);

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

  derived2.enqueueMeasurement("odom0",
                              measurement,
                              measurementCovariance,
                              updateVector,
                              1002);

  derived2.integrateMeasurements(1003, postUpdateStates);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
