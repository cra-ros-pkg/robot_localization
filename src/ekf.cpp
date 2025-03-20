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

#include <Eigen/Dense>
#include <angles/angles.h>
#include <vector>
#include <robot_localization/ekf.hpp>
#include <robot_localization/filter_common.hpp>
#include <rclcpp/duration.hpp>

namespace robot_localization
{
Ekf::Ekf()
: FilterBase()
{}

Ekf::Ekf(KinematicState state)
: FilterBase(state)
{}

Ekf::~Ekf() {}

void Ekf::correct(const Measurement & measurement)
{
  FB_DEBUG(
    "---------------------- Ekf::correct ----------------------\n" <<
      "State is:\n" <<
      state_ <<
      "\n"
      "Topic is:\n" <<
      measurement.topic_name_ <<
      "\n"
      "Measurement is:\n" <<
      measurement.measurement_ <<
      "\n"
      "Measurement topic name is:\n" <<
      measurement.topic_name_ <<
      "\n\n"
      "Measurement covariance is:\n" <<
      measurement.covariance_ << "\n");

  // We don't want to update everything, so we need to build matrices that only
  // update the measured parts of our state vector. Throughout prediction and
  // correction, we attempt to maximize efficiency in Eigen.

  // First, determine how many state vector values we're updating
  std::vector<size_t> update_indices;
  for (size_t i = 0; i < measurement.update_vector_.size(); ++i) {
    if (measurement.update_vector_[i]) {
      // Handle nan and inf values in measurements
      if (std::isnan(measurement.measurement_(i))) {
        FB_DEBUG(
          "Value at index " << i <<
            " was nan. Excluding from update.\n");
      } else if (std::isinf(measurement.measurement_(i))) {
        FB_DEBUG(
          "Value at index " << i <<
            " was inf. Excluding from update.\n");
      } else {
        update_indices.push_back(i);
      }
    }
  }

  FB_DEBUG("Update indices are:\n" << update_indices << "\n");

  size_t update_size = update_indices.size();

  // Now set up the relevant matrices
  Eigen::VectorXd state_subset(update_size);       // x (in most literature)
  Eigen::VectorXd measurement_subset(update_size);  // z
  Eigen::MatrixXd measurement_covariance_subset(update_size, update_size);  // R
  Eigen::MatrixXd state_to_measurement_subset(update_size, state_.rows());  // H
  Eigen::MatrixXd kalman_gain_subset(state_.rows(), update_size);          // K
  Eigen::VectorXd innovation_subset(update_size);  // z - Hx

  state_subset.setZero();
  measurement_subset.setZero();
  measurement_covariance_subset.setZero();
  state_to_measurement_subset.setZero();
  kalman_gain_subset.setZero();
  innovation_subset.setZero();

  // Now build the sub-matrices from the full-sized matrices
  for (size_t i = 0; i < update_size; ++i) {
    measurement_subset(i) = measurement.measurement_(update_indices[i]);
    state_subset(i) = state_(update_indices[i]);

    for (size_t j = 0; j < update_size; ++j) {
      measurement_covariance_subset(i, j) =
        measurement.covariance_(update_indices[i], update_indices[j]);
    }

    // Handle negative (read: bad) covariances in the measurement. Rather
    // than exclude the measurement or make up a covariance, just take
    // the absolute value.
    if (measurement_covariance_subset(i, i) < 0) {
      FB_DEBUG(
        "WARNING: Negative covariance for index " <<
          i << " of measurement (value is" <<
          measurement_covariance_subset(i, i) <<
          "). Using absolute value...\n");

      measurement_covariance_subset(i, i) =
        ::fabs(measurement_covariance_subset(i, i));
    }

    // If the measurement variance for a given variable is very
    // near 0 (as in e-50 or so) and the variance for that
    // variable in the covariance matrix is also near zero, then
    // the Kalman gain computation will blow up. Really, no
    // measurement can be completely without error, so add a small
    // amount in that case.
    if (measurement_covariance_subset(i, i) < 1e-9) {
      FB_DEBUG(
        "WARNING: measurement had very small error covariance for index " <<
          update_indices[i] <<
          ". Adding some noise to maintain filter stability.\n");

      measurement_covariance_subset(i, i) = 1e-9;
    }
  }

  // The state-to-measurement function, h, will now be a measurement_size x
  // full_state_size matrix, with ones in the (i, i) locations of the values to
  // be updated
  for (size_t i = 0; i < update_size; ++i) {
    state_to_measurement_subset(i, update_indices[i]) = 1;
  }

  FB_DEBUG(
    "Current state subset is:\n" <<
      state_subset << "\nMeasurement subset is:\n" <<
      measurement_subset << "\nMeasurement covariance subset is:\n" <<
      measurement_covariance_subset <<
      "\nState-to-measurement subset is:\n" <<
      state_to_measurement_subset << "\n");

  // (1) Compute the Kalman gain: K = (PH') / (HPH' + R)
  Eigen::MatrixXd pht =
    estimate_error_covariance_ * state_to_measurement_subset.transpose();
  Eigen::MatrixXd hphr_inverse =
    (state_to_measurement_subset * pht + measurement_covariance_subset)
    .inverse();
  kalman_gain_subset.noalias() = pht * hphr_inverse;

  innovation_subset = (measurement_subset - state_subset);

  // Wrap angles in the innovation
  for (size_t i = 0; i < update_size; ++i) {
    if (update_indices[i] == StateMemberRoll ||
      update_indices[i] == StateMemberPitch ||
      update_indices[i] == StateMemberYaw)
    {
      innovation_subset(i) = ::angles::normalize_angle(innovation_subset(i));
    }
  }

  // (2) Check Mahalanobis distance between mapped measurement and state.
  if (checkMahalanobisThreshold(
      innovation_subset, hphr_inverse,
      measurement.mahalanobis_thresh_))
  {
    // (3) Apply the gain to the difference between the state and measurement: x
    // = x + K(z - Hx)
    state_.noalias() += kalman_gain_subset * innovation_subset;

    // (4) Update the estimate error covariance using the Joseph form: (I -
    // KH)P(I - KH)' + KRK'
    Eigen::MatrixXd gain_residual = identity_;
    gain_residual.noalias() -= kalman_gain_subset * state_to_measurement_subset;
    estimate_error_covariance_ =
      gain_residual * estimate_error_covariance_ * gain_residual.transpose();
    estimate_error_covariance_.noalias() += kalman_gain_subset *
      measurement_covariance_subset *
      kalman_gain_subset.transpose();

    // Handle wrapping of angles
    wrapStateAngles();

    FB_DEBUG(
      "Kalman gain subset is:\n" <<
        kalman_gain_subset << "\nInnovation is:\n" <<
        innovation_subset << "\nCorrected full state is:\n" <<
        state_ << "\nCorrected full estimate error covariance is:\n" <<
        estimate_error_covariance_ <<
        "\n\n---------------------- /Ekf::correct ----------------------\n");
  }
}

void Ekf::predict(
  const rclcpp::Time & reference_time,
  const rclcpp::Duration & delta)
{
  const double delta_sec = filter_utilities::toSec(delta);

  FB_DEBUG(
    "---------------------- Ekf::predict ----------------------\n" <<
      "delta is " << filter_utilities::toSec(delta) << "\n" <<
      "state is " << state_ << "\n");

  jacobian_creator_->copyState(state_);

  prepareControl(reference_time, delta);

  // Prepare the transfer function
  
  transfer_function_ = jacobian_creator_->createTransferFunction(delta_sec);

  transfer_function_jacobian_ = jacobian_creator_->createTransferJacobian(transfer_function_);

  FB_DEBUG(
    "Transfer function is:\n" <<
      transfer_function_ << "\nTransfer function Jacobian is:\n" <<
      transfer_function_jacobian_ << "\nProcess noise covariance is:\n" <<
      process_noise_covariance_ << "\nCurrent state is:\n" <<
      state_ << "\n");

  Eigen::MatrixXd * process_noise_covariance = &process_noise_covariance_;

  if (use_dynamic_process_noise_covariance_) {
    computeDynamicProcessNoiseCovariance(state_);
    process_noise_covariance = &dynamic_process_noise_covariance_;
  }

  // (1) Apply control terms, which are actually accelerations
  state_(StateMemberVroll) +=
    control_acceleration_(ControlMemberVroll) * delta_sec;
  state_(StateMemberVpitch) +=
    control_acceleration_(ControlMemberVpitch) * delta_sec;
  state_(StateMemberVyaw) +=
    control_acceleration_(ControlMemberVyaw) * delta_sec;

  state_(StateMemberAx) = (control_update_vector_[ControlMemberVx] ?
    control_acceleration_(ControlMemberVx) :
    state_(StateMemberAx));
  state_(StateMemberAy) = (control_update_vector_[ControlMemberVy] ?
    control_acceleration_(ControlMemberVy) :
    state_(StateMemberAy));
  state_(StateMemberAz) = (control_update_vector_[ControlMemberVz] ?
    control_acceleration_(ControlMemberVz) :
    state_(StateMemberAz));

  // (2) Project the state forward: x = Ax + Bu (really, x = f(x, u))
  state_ = transfer_function_ * state_;

  // Handle wrapping
  wrapStateAngles();

  FB_DEBUG(
    "Predicted state is:\n" <<
      state_ << "\nCurrent estimate error covariance is:\n" <<
      estimate_error_covariance_ << "\n");

  // (3) Project the error forward: P = J * P * J' + Q
  estimate_error_covariance_ =
    (transfer_function_jacobian_ * estimate_error_covariance_ *
    transfer_function_jacobian_.transpose());
  estimate_error_covariance_.noalias() +=
    delta_sec * (*process_noise_covariance);

  FB_DEBUG(
    "Predicted estimate error covariance is:\n" <<
      estimate_error_covariance_ <<
      "\n\n--------------------- /Ekf::predict ----------------------\n");
}

}  // namespace robot_localization
