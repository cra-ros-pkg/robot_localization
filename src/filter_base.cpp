/*
 * Copyright (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * Copyright (c) 2017, Locus Robotics, Inc.
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

#include <robot_localization/filter_base.hpp>
#include <robot_localization/filter_common.hpp>
#include <robot_localization/filter_utilities.hpp>

#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>
#include <algorithm>

namespace robot_localization
{
FilterBase::FilterBase()
: initialized_(false), use_control_(false),
  use_dynamic_process_noise_covariance_(false), control_timeout_(0, 0u),
  last_measurement_time_(0, 0, RCL_ROS_TIME), latest_control_time_(0, 0, RCL_ROS_TIME),
  sensor_timeout_(0, 0u), debug_stream_(nullptr),
  acceleration_gains_(TWIST_SIZE, 0.0),
  acceleration_limits_(TWIST_SIZE, 0.0),
  deceleration_gains_(TWIST_SIZE, 0.0),
  deceleration_limits_(TWIST_SIZE, 0.0),
  control_update_vector_(TWIST_SIZE, 0), control_acceleration_(TWIST_SIZE),
  latest_control_(TWIST_SIZE), predicted_state_(STATE_SIZE),
  state_(STATE_SIZE), covariance_epsilon_(STATE_SIZE, STATE_SIZE),
  dynamic_process_noise_covariance_(STATE_SIZE, STATE_SIZE),
  estimate_error_covariance_(STATE_SIZE, STATE_SIZE),
  identity_(STATE_SIZE, STATE_SIZE),
  process_noise_covariance_(STATE_SIZE, STATE_SIZE),
  transfer_function_(STATE_SIZE, STATE_SIZE),
  transfer_function_jacobian_(STATE_SIZE, STATE_SIZE), debug_(false)
{
  reset();
}

FilterBase::~FilterBase() {}

void FilterBase::reset()
{
  initialized_ = false;

  // Clear the state and predicted state
  state_.setZero();
  predicted_state_.setZero();
  control_acceleration_.setZero();

  // Prepare the invariant parts of the transfer
  // function
  transfer_function_.setIdentity();

  // Clear the Jacobian
  transfer_function_jacobian_.setZero();

  // Set the estimate error covariance. We want our measurements
  // to be accepted rapidly when the filter starts, so we should
  // initialize the state's covariance with large values.
  estimate_error_covariance_.setIdentity();
  estimate_error_covariance_ *= 1e-9;

  // We need the identity for the update equations
  identity_.setIdentity();

  // Set the epsilon matrix to be a matrix with small values on the diagonal
  // It is used to maintain the positive-definite property of the covariance
  covariance_epsilon_.setIdentity();
  covariance_epsilon_ *= 0.001;

  // Assume 30Hz from sensor data (configurable)
  sensor_timeout_ = rclcpp::Duration::from_seconds(0.033333333);

  // Initialize our last update and measurement times
  last_measurement_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // These can be overridden via the launch parameters,
  // but we need default values.
  process_noise_covariance_.setZero();
  process_noise_covariance_(StateMemberX, StateMemberX) = 0.05;
  process_noise_covariance_(StateMemberY, StateMemberY) = 0.05;
  process_noise_covariance_(StateMemberZ, StateMemberZ) = 0.06;
  process_noise_covariance_(StateMemberRoll, StateMemberRoll) = 0.03;
  process_noise_covariance_(StateMemberPitch, StateMemberPitch) = 0.03;
  process_noise_covariance_(StateMemberYaw, StateMemberYaw) = 0.06;
  process_noise_covariance_(StateMemberVx, StateMemberVx) = 0.025;
  process_noise_covariance_(StateMemberVy, StateMemberVy) = 0.025;
  process_noise_covariance_(StateMemberVz, StateMemberVz) = 0.04;
  process_noise_covariance_(StateMemberVroll, StateMemberVroll) = 0.01;
  process_noise_covariance_(StateMemberVpitch, StateMemberVpitch) = 0.01;
  process_noise_covariance_(StateMemberVyaw, StateMemberVyaw) = 0.02;
  process_noise_covariance_(StateMemberAx, StateMemberAx) = 0.01;
  process_noise_covariance_(StateMemberAy, StateMemberAy) = 0.01;
  process_noise_covariance_(StateMemberAz, StateMemberAz) = 0.015;

  dynamic_process_noise_covariance_ = process_noise_covariance_;
}

void FilterBase::computeDynamicProcessNoiseCovariance(
  const Eigen::VectorXd & state)
{
  // A more principled approach would be to get the current velocity from the
  // state, make a diagonal matrix from it, and then rotate it to be in the
  // world frame (i.e., the same frame as the pose data). We could then use this
  // rotated velocity matrix to scale the process noise covariance for the pose
  // variables as rotatedVelocityMatrix * poseCovariance *
  // rotatedVelocityMatrix' However, this presents trouble for robots that may
  // incur rotational error as a result of linear motion (and vice-versa).
  // Instead, we create a diagonal matrix whose diagonal values are the vector
  // norm of the state's velocity. We use that to scale the process noise
  // covariance.
  Eigen::MatrixXd velocity_matrix(TWIST_SIZE, TWIST_SIZE);
  velocity_matrix.setIdentity();
  velocity_matrix.diagonal() *=
    state.segment(POSITION_V_OFFSET, TWIST_SIZE).norm();

  dynamic_process_noise_covariance_.block<TWIST_SIZE, TWIST_SIZE>(
    POSITION_OFFSET, POSITION_OFFSET) =
    velocity_matrix *
    process_noise_covariance_.block<TWIST_SIZE, TWIST_SIZE>(
    POSITION_OFFSET,
    POSITION_OFFSET) *
    velocity_matrix.transpose();
}

const Eigen::VectorXd & FilterBase::getControl() {return latest_control_;}

const rclcpp::Time & FilterBase::getControlTime()
{
  return latest_control_time_;
}

bool FilterBase::getDebug() {return debug_;}

const Eigen::MatrixXd & FilterBase::getEstimateErrorCovariance()
{
  return estimate_error_covariance_;
}

bool FilterBase::getInitializedStatus() {return initialized_;}

const rclcpp::Time & FilterBase::getLastMeasurementTime()
{
  return last_measurement_time_;
}

const Eigen::VectorXd & FilterBase::getPredictedState()
{
  return predicted_state_;
}

const Eigen::MatrixXd & FilterBase::getProcessNoiseCovariance()
{
  return process_noise_covariance_;
}

const rclcpp::Duration & FilterBase::getSensorTimeout()
{
  return sensor_timeout_;
}

const Eigen::VectorXd & FilterBase::getState() {return state_;}

void FilterBase::processMeasurement(const Measurement & measurement)
{
  FB_DEBUG(
    "------ FilterBase::processMeasurement (" << measurement.topic_name_ <<
      ") ------\n");

  rclcpp::Duration delta(0, 0u);

  // If we've had a previous reading, then go through the predict/update
  // cycle. Otherwise, set our state and covariance to whatever we get
  // from this measurement.
  if (initialized_) {
    // Determine how much time has passed since our last measurement
    delta = measurement.time_ - last_measurement_time_;

    FB_DEBUG(
      "Filter is already initialized. Carrying out predict/correct loop...\n"
      "Measurement time is " <<
        std::setprecision(20) << measurement.time_.nanoseconds() <<
        ", last measurement time is " << last_measurement_time_.nanoseconds() <<
        ", delta is " << delta.nanoseconds() << "\n");

    // Only want to carry out a prediction if it's
    // forward in time. Otherwise, just correct.
    if (delta > rclcpp::Duration(0, 0u)) {
      validateDelta(delta);
      predict(measurement.time_, delta);

      // Return this to the user
      predicted_state_ = state_;
    }

    correct(measurement);
  } else {
    FB_DEBUG("First measurement. Initializing filter.\n");

    // Initialize the filter, but only with the values we're using
    size_t measurement_length = measurement.update_vector_.size();
    for (size_t i = 0; i < measurement_length; ++i) {
      state_[i] = (measurement.update_vector_[i] ? measurement.measurement_[i] :
        state_[i]);
    }

    // Same for covariance
    for (size_t i = 0; i < measurement_length; ++i) {
      for (size_t j = 0; j < measurement_length; ++j) {
        estimate_error_covariance_(i, j) =
          (measurement.update_vector_[i] && measurement.update_vector_[j] ?
          measurement.covariance_(i, j) :
          estimate_error_covariance_(i, j));
      }
    }

    initialized_ = true;
  }

  if (delta >= rclcpp::Duration(0, 0u)) {
    // Update the last measurement and update time.
    // The measurement time is based on the time stamp of the
    // measurement, whereas the update time is based on this
    // node's current ROS time. The update time is used to
    // determine if we have a sensor timeout, whereas the
    // measurement time is used to calculate time deltas for
    // prediction and correction.
    last_measurement_time_ = measurement.time_;
  }

  FB_DEBUG(
    "------ /FilterBase::processMeasurement (" << measurement.topic_name_ <<
      ") ------\n");
}

void FilterBase::setControl(
  const Eigen::VectorXd & control,
  const rclcpp::Time & control_time)
{
  latest_control_ = control;
  latest_control_time_ = control_time;
}

void FilterBase::setControlParams(
  const std::vector<bool> & update_vector,
  const rclcpp::Duration & control_timeout,
  const std::vector<double> & acceleration_limits,
  const std::vector<double> & acceleration_gains,
  const std::vector<double> & deceleration_limits,
  const std::vector<double> & deceleration_gains)
{
  use_control_ = true;
  control_update_vector_ = update_vector;
  control_timeout_ = control_timeout;
  acceleration_limits_ = acceleration_limits;
  acceleration_gains_ = acceleration_gains;
  deceleration_limits_ = deceleration_limits;
  deceleration_gains_ = deceleration_gains;
}

void FilterBase::setDebug(const bool debug, std::ostream * out_stream)
{
  if (debug) {
    if (out_stream != NULL) {
      debug_stream_ = out_stream;
      debug_ = true;
    } else {
      debug_ = false;
    }
  } else {
    debug_ = false;
  }
}

void FilterBase::setUseDynamicProcessNoiseCovariance(
  const bool use_dynamic_process_noise_covariance)
{
  use_dynamic_process_noise_covariance_ = use_dynamic_process_noise_covariance;
}

void FilterBase::setEstimateErrorCovariance(
  const Eigen::MatrixXd & estimate_error_covariance)
{
  estimate_error_covariance_ = estimate_error_covariance;
}

void FilterBase::setLastMeasurementTime(
  const rclcpp::Time & last_measurement_time)
{
  last_measurement_time_ = last_measurement_time;
}

void FilterBase::setProcessNoiseCovariance(
  const Eigen::MatrixXd & process_noise_covariance)
{
  process_noise_covariance_ = process_noise_covariance;
  dynamic_process_noise_covariance_ = process_noise_covariance_;
}

void FilterBase::setSensorTimeout(const rclcpp::Duration & sensor_timeout)
{
  sensor_timeout_ = sensor_timeout;
}

void FilterBase::setState(const Eigen::VectorXd & state) {state_ = state;}

void FilterBase::validateDelta(rclcpp::Duration & /*delta*/)
{
  // TODO(someone): Need to verify this condition B'Coz
  // rclcpp::Duration::from_seconds(100000.0) value is 0.00010000000000000000479
  // This handles issues with ROS time when use_sim_time is on and we're playing
  // from bags.
  /* if (delta > rclcpp::Duration::from_seconds(100000.0))
  {
    FB_DEBUG("Delta was very large. Suspect playing from bag file. Setting to
  0.01\n");

    delta = rclcpp::Duration::from_seconds(0.01);
  } */
}

void FilterBase::prepareControl(
  const rclcpp::Time & reference_time,
  const rclcpp::Duration &)
{
  control_acceleration_.setZero();

  if (use_control_) {
    bool timed_out =
      (reference_time - latest_control_time_ >= control_timeout_);

    if (timed_out) {
      FB_DEBUG(
        "Control timed out. Reference time was " <<
          reference_time.nanoseconds() << ", latest control time was " <<
          latest_control_time_.nanoseconds() << ", control timeout was " <<
          control_timeout_.nanoseconds() << "\n");
    }

    for (size_t controlInd = 0; controlInd < TWIST_SIZE; ++controlInd) {
      if (control_update_vector_[controlInd]) {
        control_acceleration_(controlInd) = computeControlAcceleration(
          state_(controlInd + POSITION_V_OFFSET),
          (timed_out ? 0.0 : latest_control_(controlInd)),
          acceleration_limits_[controlInd], acceleration_gains_[controlInd],
          deceleration_limits_[controlInd], deceleration_gains_[controlInd]);
      }
    }
  }
}

inline double FilterBase::computeControlAcceleration(
  const double state,
  const double control,
  const double acceleration_limit,
  const double acceleration_gain,
  const double deceleration_limit,
  const double deceleration_gain)
{
  FB_DEBUG("---------- FilterBase::computeControlAcceleration ----------\n");

  const double error = control - state;
  const bool same_sign = (::fabs(error) <= ::fabs(control) + 0.01);
  const double set_point = (same_sign ? control : 0.0);
  const bool decelerating = ::fabs(set_point) < ::fabs(state);
  double limit = acceleration_limit;
  double gain = acceleration_gain;

  if (decelerating) {
    limit = deceleration_limit;
    gain = deceleration_gain;
  }

  const double final_accel = std::min(std::max(gain * error, -limit), limit);

  FB_DEBUG(
    "Control value: " <<
      control << "\n" <<
      "State value: " << state << "\n" <<
      "Error: " << error << "\n" <<
      "Same sign: " << (same_sign ? "true" : "false") << "\n" <<
      "Set point: " << set_point << "\n" <<
      "Decelerating: " << (decelerating ? "true" : "false") << "\n" <<
      "Limit: " << limit << "\n" <<
      "Gain: " << gain << "\n" <<
      "Final is " << final_accel << "\n");

  return final_accel;
}

void FilterBase::wrapStateAngles()
{
  state_(StateMemberRoll) =
    filter_utilities::clampRotation(state_(StateMemberRoll));
  state_(StateMemberPitch) =
    filter_utilities::clampRotation(state_(StateMemberPitch));
  state_(StateMemberYaw) =
    filter_utilities::clampRotation(state_(StateMemberYaw));
}

bool FilterBase::checkMahalanobisThreshold(
  const Eigen::VectorXd & innovation,
  const Eigen::MatrixXd & innovation_covariance, const double n_sigmas)
{
  double squared_mahalanobis =
    innovation.dot(innovation_covariance * innovation);
  double threshold = n_sigmas * n_sigmas;

  if (squared_mahalanobis >= threshold) {
    FB_DEBUG(
      "Innovation mahalanobis distance test failed. Squared Mahalanobis is: " <<
        squared_mahalanobis << "\nThreshold is: " << threshold << "\n" <<
        "Innovation is: " << innovation << "\n" <<
        "Innovation covariance is:\n" <<
        innovation_covariance << "\n");

    return false;
  }

  return true;
}
}  // namespace robot_localization
