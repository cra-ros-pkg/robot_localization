/*
 * Copyright (c) 2014, Charles River Analytics, Inc.
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

#include <robot_localization/filter_common.h>
#include <robot_localization/ekf.h>

#include <sstream>
#include <iomanip>
#include <limits>
#include <iostream>

namespace RobotLocalization
{
  FilterBase::FilterBase() :
    state_(STATE_SIZE),
    transferFunction_(STATE_SIZE, STATE_SIZE),
    transferFunctionJacobian_(STATE_SIZE, STATE_SIZE),
    estimateErrorCovariance_(STATE_SIZE, STATE_SIZE),
    covarianceEpsilon_(STATE_SIZE, STATE_SIZE),
    processNoiseCovariance_(STATE_SIZE, STATE_SIZE),
    identity_(STATE_SIZE, STATE_SIZE),
    pi_(3.141592653589793),
    tau_(6.283185307179586),
    debug_(false),
    debugStream_(NULL)
  {
    initialized_ = false;

    // Clear the state
    state_.setZero();

    // Prepare the invariant parts of the transfer
    // function
    transferFunction_.setIdentity();

    // Clear the Jacobian
    transferFunctionJacobian_.setZero();

    // Prepare the invariant parts of the transfer
    // function
    estimateErrorCovariance_.setIdentity();

    // We need the identity for the update equations
    identity_.setIdentity();

    // Set the epsilon matrix to be a matrix with small values on the diagonal
    // It is used to maintain the positive-definite property of the covariance
    covarianceEpsilon_.setIdentity();
    covarianceEpsilon_ *= 0.001;

    // Assume 30Hz from sensor data (configurable)
    sensorTimeout_ = 0.033333333;

    // Initialize our last update and measurement times
    lastUpdateTime_ = 0;
    lastMeasurementTime_ = 0;

    // These can be overridden via the launch parameters,
    // but we need default values.
    processNoiseCovariance_.setZero();
    processNoiseCovariance_(StateMemberX, StateMemberX) = 0.03;
    processNoiseCovariance_(StateMemberY, StateMemberY) = 0.03;
    processNoiseCovariance_(StateMemberZ, StateMemberZ) = 0.4;
    processNoiseCovariance_(StateMemberRoll, StateMemberRoll) = 0.03;
    processNoiseCovariance_(StateMemberPitch, StateMemberPitch) = 0.03;
    processNoiseCovariance_(StateMemberYaw, StateMemberYaw) = 0.06;
    processNoiseCovariance_(StateMemberVx, StateMemberVx) = 0.025;
    processNoiseCovariance_(StateMemberVy, StateMemberVy) = 0.025;
    processNoiseCovariance_(StateMemberVz, StateMemberVz) = 0.05;
    processNoiseCovariance_(StateMemberVroll, StateMemberVroll) = 0.002;
    processNoiseCovariance_(StateMemberVpitch, StateMemberVpitch) = 0.002;
    processNoiseCovariance_(StateMemberVyaw, StateMemberVyaw) = 0.004;
  }

  FilterBase::~FilterBase()
  {
  }

  void FilterBase::enqueueMeasurement(const std::string &topicName,
                                      const Eigen::VectorXd &measurement,
                                      const Eigen::MatrixXd &measurementCovariance,
                                      const std::vector<int> &updateVector,
                                      const double time)
  {
    Measurement meas;

    meas.topicName_ = topicName;
    meas.measurement_ = measurement;
    meas.covariance_ = measurementCovariance;
    meas.updateVector_ = updateVector;
    meas.time_ = time;

    measurementQueue_.push(meas);
  }

  void FilterBase::integrateMeasurements(double currentTime,
                                         std::map<std::string, Eigen::VectorXd> &postUpdateStates)
  {
    if (debug_)
    {
      *debugStream_ << "------ FilterBase::integrateMeasurements ------\n\n";
      *debugStream_ << "Integration time is " << std::setprecision(20) << currentTime << "\n";
    }

    postUpdateStates.clear();

    // If we have any measurements in the queue, process them
    if (!measurementQueue_.empty())
    {
      while (!measurementQueue_.empty())
      {
        Measurement measurement = measurementQueue_.top();
        measurementQueue_.pop();

        processMeasurement(measurement);

        postUpdateStates.insert(std::pair<std::string, Eigen::VectorXd>(measurement.topicName_, state_));
      }

      lastUpdateTime_ = currentTime;
    }
    else if (initialized_)
    {
      // In the event that we don't get any measurements for a long time,
      // we still need to continue to estimate our state. Therefore, we
      // should project the state forward here.
      double lastUpdateDelta = currentTime - lastUpdateTime_;

      // If we get a large delta, then continuously predict until
      if(lastUpdateDelta >= sensorTimeout_)
      {
        double projectTime = sensorTimeout_ * std::floor(lastUpdateDelta / sensorTimeout_);

        if (debug_)
        {
          *debugStream_ << "Sensor timeout! Last measurement was " << std::setprecision(10) << lastMeasurementTime_ << ", current time is " <<
                           currentTime << ", delta is " << lastUpdateDelta << ", projection time is " << projectTime << "\n";
        }

        validateDelta(projectTime);

        predict(projectTime);

        // Update the last measurement time and last update time
        lastMeasurementTime_ += projectTime;
        lastUpdateTime_ += projectTime;
      }

    }
    else
    {
      if (debug_)
      {
        *debugStream_ << "Filter not yet initialized\n";
      }
    }

    if (debug_)
    {
      *debugStream_ << "\n----- /FilterBase::integrateMeasurements ------\n";
    }
  }

  bool FilterBase::getDebug()
  {
    return debug_;
  }

  const Eigen::MatrixXd& FilterBase::getEstimateErrorCovariance()
  {
    return estimateErrorCovariance_;
  }

  bool FilterBase::getInitializedStatus()
  {
    return initialized_;
  }

  double FilterBase::getLastMeasurementTime()
  {
    return lastMeasurementTime_;
  }

  double FilterBase::getLastUpdateTime()
  {
    return lastUpdateTime_;
  }

  const Eigen::MatrixXd& FilterBase::getProcessNoiseCovariance()
  {
    return processNoiseCovariance_;
  }

  double FilterBase::getSensorTimeout()
  {
    return sensorTimeout_;
  }

  const Eigen::VectorXd& FilterBase::getState()
  {
    return state_;
  }

  void FilterBase::processMeasurement(const Measurement &measurement)
  {
    if (debug_)
    {
      *debugStream_ << "------ FilterBase::processMeasurement ------\n";
    }

    double delta = 0.0;

    // If we've had a previous reading, then go through the predict/update
    // cycle. Otherwise, set our state and covariance to whatever we get
    // from this measurement.
    if (initialized_)
    {
      // Determine how much time has passed since our last measurement
      delta = measurement.time_ - lastMeasurementTime_;

      if (debug_)
      {
        *debugStream_ << "Filter is already initialized. Carrying out EKF loop...\n";
        *debugStream_ << "Measurement time is " << std::setprecision(20) << measurement.time_ <<
                         ", last measurement time is " << lastMeasurementTime_ << ", delta is " << delta << "\n";
      }

      // Only want to carry out a prediction if it's
      // forward in time. Otherwise, just correct.
      if (delta > 0)
      {
        validateDelta(delta);

        predict(delta);
      }

      correct(measurement);
    }
    else
    {
      if (debug_)
      {
        *debugStream_ << "First measurement. Initializing filter.\n";
      }

      state_ = measurement.measurement_;

      initialized_ = true;
    }

    if(delta >= 0.0)
    {
      // Update the last measurement and update time.
      // The measurement time is based on the time stamp of the
      // measurement, whereas the update time is based on this
      // node's current ROS time. The update time is used to
      // determine if we have a sensor timeout, whereas the
      // measurement time is used to calculate time deltas for
      // prediction and correction.
      lastMeasurementTime_ = measurement.time_;
    }

    if (debug_)
    {
      *debugStream_ << "\n----- /FilterBase::processMeasurement ------\n";
    }
  }

  void FilterBase::setDebug(const bool debug, std::ostream *outStream)
  {
    if (debug)
    {
      if (outStream != NULL)
      {
        debugStream_ = outStream;
        debug_ = true;
      }
      else
      {
        debug_ = false;
      }
    }
    else
    {
      debug_ = false;
    }
  }

  void FilterBase::setLastMeasurementTime(const double lastMeasurementTime)
  {
    lastMeasurementTime_ = lastMeasurementTime;
  }

  void FilterBase::setLastUpdateTime(const double lastUpdateTime)
  {
    lastUpdateTime_ = lastUpdateTime;
  }

  void FilterBase::setProcessNoiseCovariance(const Eigen::MatrixXd &processNoiseCovariance)
  {
    processNoiseCovariance_ = processNoiseCovariance;
  }

  void FilterBase::setSensorTimeout(const double sensorTimeout)
  {
    sensorTimeout_ = sensorTimeout;
  }

  void FilterBase::setState(const Eigen::VectorXd &state)
  {
    state_ = state;
  }

  void FilterBase::validateDelta(double &delta)
  {
    // This handles issues with ROS time when use_sim_time is on and we're playing from bags.
    if (delta > 100000.0)
    {
      if (debug_)
      {
        *debugStream_ << "Delta was very large. Suspect playing from bag file. Setting to 0.01\n";
      }

      delta = 0.01;
    }
  }

  void FilterBase::wrapStateAngles()
  {
    while (state_(StateMemberRoll) < -pi_)
    {
      state_(StateMemberRoll) += tau_;
    }
    while (state_(StateMemberRoll) > pi_)
    {
      state_(StateMemberRoll) -= tau_;
    }

    while (state_(StateMemberPitch) < -pi_)
    {
      state_(StateMemberPitch) += tau_;
    }
    while (state_(StateMemberPitch) > pi_)
    {
      state_(StateMemberPitch) -= tau_;
    }

    while (state_(StateMemberYaw) < -pi_)
    {
      state_(StateMemberYaw) += tau_;
    }
    while (state_(StateMemberYaw) > pi_)
    {
      state_(StateMemberYaw) -= tau_;
    }
  }
}

std::ostream& operator<<(std::ostream& os, const Eigen::MatrixXd &mat)
{
  os << "[";

  int rowCount = static_cast<int>(mat.rows());

  for (int row = 0; row < rowCount; ++row)
  {
    if (row > 0)
    {
      os << " ";
    }

    for (int col = 0; col < mat.cols(); ++col)
    {
      os << std::setiosflags(std::ios::left) << std::setw(12) << std::setprecision(5) << mat(row, col);
    }

    if (row < rowCount - 1)
    {
      os << "\n";
    }
  }

  os << "]\n";

  return os;
}

std::ostream& operator<<(std::ostream& os, const Eigen::VectorXd &vec)
{
  os << "[";
  for (int dim = 0; dim < vec.rows(); ++dim)
  {
    os << std::setiosflags(std::ios::left) << std::setw(12) << std::setprecision(5) << vec(dim);
  }
  os << "]\n";

  return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<size_t> &vec)
{
  os << "[";
  for (size_t dim = 0; dim < vec.size(); ++dim)
  {
    os << std::setiosflags(std::ios::left) << std::setw(12) << std::setprecision(5) << vec[dim];
  }
  os << "]\n";

  return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<int> &vec)
{
  os << "[";
  for (size_t dim = 0; dim < vec.size(); ++dim)
  {
    os << std::setiosflags(std::ios::left) << std::setw(12) << (vec[dim] ? "true " : "false");
  }
  os << "]\n";

  return os;
}

