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

#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

namespace RobotLocalization
{
  FilterBase::FilterBase() :
    accelerationGains_(TWIST_SIZE, 0.0),
    accelerationLimits_(TWIST_SIZE, 0.0),
    decelerationGains_(TWIST_SIZE, 0.0),
    decelerationLimits_(TWIST_SIZE, 0.0),
    controlAcceleration_(TWIST_SIZE),
    controlTimeout_(0.0),
    controlUpdateVector_(TWIST_SIZE, 0),
    dynamicProcessNoiseCovariance_(STATE_SIZE, STATE_SIZE),
    latestControlTime_(0.0),
    state_(STATE_SIZE),
    predictedState_(STATE_SIZE),
    transferFunction_(STATE_SIZE, STATE_SIZE),
    transferFunctionJacobian_(STATE_SIZE, STATE_SIZE),
    estimateErrorCovariance_(STATE_SIZE, STATE_SIZE),
    covarianceEpsilon_(STATE_SIZE, STATE_SIZE),
    processNoiseCovariance_(STATE_SIZE, STATE_SIZE),
    identity_(STATE_SIZE, STATE_SIZE),
    debug_(false),
    debugStream_(NULL),
    useControl_(false),
    useDynamicProcessNoiseCovariance_(false)
  {
    reset();
  }

  FilterBase::~FilterBase()
  {
  }

  void FilterBase::reset()
  {
    initialized_ = false;

    // Clear the state and predicted state
    state_.setZero();
    predictedState_.setZero();
    controlAcceleration_.setZero();

    // Prepare the invariant parts of the transfer
    // function
    transferFunction_.setIdentity();

    // Clear the Jacobian
    transferFunctionJacobian_.setZero();

    // Set the estimate error covariance. We want our measurements
    // to be accepted rapidly when the filter starts, so we should
    // initialize the state's covariance with large values.
    estimateErrorCovariance_.setIdentity();
    estimateErrorCovariance_ *= 1e-9;

    // We need the identity for the update equations
    identity_.setIdentity();

    // Set the epsilon matrix to be a matrix with small values on the diagonal
    // It is used to maintain the positive-definite property of the covariance
    covarianceEpsilon_.setIdentity();
    covarianceEpsilon_ *= 0.001;

    // Assume 30Hz from sensor data (configurable)
    sensorTimeout_ = 0.033333333;

    // Initialize our measurement time
    lastMeasurementTime_ = 0;

    // These can be overridden via the launch parameters,
    // but we need default values.
    processNoiseCovariance_.setZero();
    processNoiseCovariance_(StateMemberX, StateMemberX) = 0.05;
    processNoiseCovariance_(StateMemberY, StateMemberY) = 0.05;
    processNoiseCovariance_(StateMemberZ, StateMemberZ) = 0.06;
    processNoiseCovariance_(StateMemberRoll, StateMemberRoll) = 0.03;
    processNoiseCovariance_(StateMemberPitch, StateMemberPitch) = 0.03;
    processNoiseCovariance_(StateMemberYaw, StateMemberYaw) = 0.06;
    processNoiseCovariance_(StateMemberVx, StateMemberVx) = 0.025;
    processNoiseCovariance_(StateMemberVy, StateMemberVy) = 0.025;
    processNoiseCovariance_(StateMemberVz, StateMemberVz) = 0.04;
    processNoiseCovariance_(StateMemberVroll, StateMemberVroll) = 0.01;
    processNoiseCovariance_(StateMemberVpitch, StateMemberVpitch) = 0.01;
    processNoiseCovariance_(StateMemberVyaw, StateMemberVyaw) = 0.02;
    processNoiseCovariance_(StateMemberAx, StateMemberAx) = 0.01;
    processNoiseCovariance_(StateMemberAy, StateMemberAy) = 0.01;
    processNoiseCovariance_(StateMemberAz, StateMemberAz) = 0.015;

    dynamicProcessNoiseCovariance_ = processNoiseCovariance_;
  }

  void FilterBase::computeDynamicProcessNoiseCovariance(const Eigen::VectorXd &state, const double delta)
  {
    // A more principled approach would be to get the current velocity from the state, make a diagonal matrix from it,
    // and then rotate it to be in the world frame (i.e., the same frame as the pose data). We could then use this
    // rotated velocity matrix to scale the process noise covariance for the pose variables as
    // rotatedVelocityMatrix * poseCovariance * rotatedVelocityMatrix'
    // However, this presents trouble for robots that may incur rotational error as a result of linear motion (and
    // vice-versa). Instead, we create a diagonal matrix whose diagonal values are the vector norm of the state's
    // velocity. We use that to scale the process noise covariance.
    Eigen::MatrixXd velocityMatrix(TWIST_SIZE, TWIST_SIZE);
    velocityMatrix.setIdentity();
    velocityMatrix.diagonal() *= state.segment(POSITION_V_OFFSET, TWIST_SIZE).norm();

    dynamicProcessNoiseCovariance_.block<TWIST_SIZE, TWIST_SIZE>(POSITION_OFFSET, POSITION_OFFSET) =
      velocityMatrix *
      processNoiseCovariance_.block<TWIST_SIZE, TWIST_SIZE>(POSITION_OFFSET, POSITION_OFFSET) *
      velocityMatrix.transpose();
  }

  const Eigen::VectorXd& FilterBase::getControl()
  {
    return latestControl_;
  }

  double FilterBase::getControlTime()
  {
    return latestControlTime_;
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

  const Eigen::VectorXd& FilterBase::getPredictedState()
  {
    return predictedState_;
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
    FB_DEBUG("------ FilterBase::processMeasurement (" << measurement.topicName_ << ") ------\n");

    double delta = 0.0;

    // If we've had a previous reading, then go through the predict/update
    // cycle. Otherwise, set our state and covariance to whatever we get
    // from this measurement.
    if (initialized_)
    {
      // Determine how much time has passed since our last measurement
      delta = measurement.time_ - lastMeasurementTime_;

      FB_DEBUG("Filter is already initialized. Carrying out predict/correct loop...\n"
               "Measurement time is " << std::setprecision(20) << measurement.time_ <<
               ", last measurement time is " << lastMeasurementTime_ << ", delta is " << delta << "\n");

      // Only want to carry out a prediction if it's
      // forward in time. Otherwise, just correct.
      if (delta > 0)
      {
        validateDelta(delta);
        predict(measurement.time_, delta);

        // Return this to the user
        predictedState_ = state_;
      }

      correct(measurement);
    }
    else
    {
      FB_DEBUG("First measurement. Initializing filter.\n");

      // Initialize the filter, but only with the values we're using
      size_t measurementLength = measurement.updateVector_.size();
      for (size_t i = 0; i < measurementLength; ++i)
      {
        state_[i] = (measurement.updateVector_[i] ? measurement.measurement_[i] : state_[i]);
      }

      // Same for covariance
      for (size_t i = 0; i < measurementLength; ++i)
      {
        for (size_t j = 0; j < measurementLength; ++j)
        {
          estimateErrorCovariance_(i, j) = (measurement.updateVector_[i] && measurement.updateVector_[j] ?
                                            measurement.covariance_(i, j) :
                                            estimateErrorCovariance_(i, j));
        }
      }

      initialized_ = true;
    }

    if (delta >= 0.0)
    {
      lastMeasurementTime_ = measurement.time_;
    }

    FB_DEBUG("------ /FilterBase::processMeasurement (" << measurement.topicName_ << ") ------\n");
  }

  void FilterBase::setControl(const Eigen::VectorXd &control, const double controlTime)
  {
    latestControl_ = control;
    latestControlTime_ = controlTime;
  }

  void FilterBase::setControlParams(const std::vector<int> &updateVector, const double controlTimeout,
    const std::vector<double> &accelerationLimits, const std::vector<double> &accelerationGains,
    const std::vector<double> &decelerationLimits, const std::vector<double> &decelerationGains)
  {
    useControl_ = true;
    controlUpdateVector_ = updateVector;
    controlTimeout_ = controlTimeout;
    accelerationLimits_ = accelerationLimits;
    accelerationGains_ = accelerationGains;
    decelerationLimits_ = decelerationLimits;
    decelerationGains_ = decelerationGains;
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

  void FilterBase::setUseDynamicProcessNoiseCovariance(const bool useDynamicProcessNoiseCovariance)
  {
    useDynamicProcessNoiseCovariance_ = useDynamicProcessNoiseCovariance;
  }

  void FilterBase::setEstimateErrorCovariance(const Eigen::MatrixXd &estimateErrorCovariance)
  {
    estimateErrorCovariance_ = estimateErrorCovariance;
  }

  void FilterBase::setLastMeasurementTime(const double lastMeasurementTime)
  {
    lastMeasurementTime_ = lastMeasurementTime;
  }

  void FilterBase::setProcessNoiseCovariance(const Eigen::MatrixXd &processNoiseCovariance)
  {
    processNoiseCovariance_ = processNoiseCovariance;
    dynamicProcessNoiseCovariance_ = processNoiseCovariance_;
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
      FB_DEBUG("Delta was very large. Suspect playing from bag file. Setting to 0.01\n");

      delta = 0.01;
    }
  }


  void FilterBase::prepareControl(const double referenceTime, const double predictionDelta)
  {
    controlAcceleration_.setZero();

    if (useControl_)
    {
      bool timedOut = ::fabs(referenceTime - latestControlTime_) >= controlTimeout_;

      if (timedOut)
      {
        FB_DEBUG("Control timed out. Reference time was " << referenceTime << ", latest control time was " <<
          latestControlTime_ << ", control timeout was " << controlTimeout_ << "\n");
      }

      for (size_t controlInd = 0; controlInd < TWIST_SIZE; ++controlInd)
      {
        if (controlUpdateVector_[controlInd])
        {
          controlAcceleration_(controlInd) = computeControlAcceleration(state_(controlInd + POSITION_V_OFFSET),
            (timedOut ? 0.0 : latestControl_(controlInd)), accelerationLimits_[controlInd],
            accelerationGains_[controlInd], decelerationLimits_[controlInd], decelerationGains_[controlInd]);
        }
      }
    }
  }

  void FilterBase::wrapStateAngles()
  {
    state_(StateMemberRoll)  = FilterUtilities::clampRotation(state_(StateMemberRoll));
    state_(StateMemberPitch) = FilterUtilities::clampRotation(state_(StateMemberPitch));
    state_(StateMemberYaw)   = FilterUtilities::clampRotation(state_(StateMemberYaw));
  }

  bool FilterBase::checkMahalanobisThreshold(const Eigen::VectorXd &innovation,
                                             const Eigen::MatrixXd &invCovariance,
                                             const double nsigmas)
  {
    double sqMahalanobis = innovation.dot(invCovariance * innovation);
    double threshold = nsigmas * nsigmas;

    if (sqMahalanobis >= threshold)
    {
      FB_DEBUG("Innovation mahalanobis distance test failed. Squared Mahalanobis is: " << sqMahalanobis << "\n" <<
               "Threshold is: " << threshold << "\n" <<
               "Innovation is: " << innovation << "\n" <<
               "Innovation covariance is:\n" << invCovariance << "\n");

      return false;
    }

    return true;
  }
}  // namespace RobotLocalization
