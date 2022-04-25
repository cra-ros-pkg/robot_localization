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

#include <robot_localization/filter_common.hpp>
#include <robot_localization/ukf.hpp>
#include <angles/angles.h>
#include <Eigen/Cholesky>
#include <vector>


namespace robot_localization
{
Ukf::Ukf()
: FilterBase(),
  uncorrected_(true)
{
  size_t sigma_count = (STATE_SIZE << 1) + 1;
  sigma_points_.resize(sigma_count, Eigen::VectorXd(STATE_SIZE));
  state_weights_.resize(sigma_count);
  covar_weights_.resize(sigma_count);
}

Ukf::~Ukf() {}

void Ukf::setConstants(double alpha, double kappa, double beta)
{
  // Prepare constants
  size_t sigma_count = (STATE_SIZE << 1) + 1;
  lambda_ = alpha * alpha * (STATE_SIZE + kappa) - STATE_SIZE;

  state_weights_[0] = lambda_ / (STATE_SIZE + lambda_);
  covar_weights_[0] = state_weights_[0] + (1 - (alpha * alpha) + beta);
  sigma_points_[0].setZero();
  for (size_t i = 1; i < sigma_count; ++i) {
    sigma_points_[i].setZero();
    state_weights_[i] = 1 / (2 * (STATE_SIZE + lambda_));
    covar_weights_[i] = state_weights_[i];
  }
}

void Ukf::correct(const Measurement & measurement)
{
  FB_DEBUG(
    "---------------------- Ukf::correct ----------------------\n" <<
      "State is:\n" <<
      state_ << "\nMeasurement is:\n" <<
      measurement.measurement_ << "\nMeasurement covariance is:\n" <<
      measurement.covariance_ << "\n");

  // In our implementation, it may be that after we call predict once, we call
  // correct several times in succession (multiple measurements with different
  // time stamps). In that event, the sigma points need to be updated to reflect
  // the current state. Throughout prediction and correction, we attempt to
  // maximize efficiency in Eigen.
  if (!uncorrected_) {
    // Take the square root of a small fraction of the
    // estimate_error_covariance_ using LL' decomposition
    weighted_covar_sqrt_ =
      ((STATE_SIZE + lambda_) * estimate_error_covariance_).llt().matrixL();

    // Compute sigma points

    // First sigma point is the current state
    sigma_points_[0] = state_;

    // Next STATE_SIZE sigma points are state + weighted_covar_sqrt_[ith column]
    // STATE_SIZE sigma points after that are state - weighted_covar_sqrt_[ith
    // column]
    for (size_t sigma_ind = 0; sigma_ind < STATE_SIZE; ++sigma_ind) {
      sigma_points_[sigma_ind + 1] =
        state_ + weighted_covar_sqrt_.col(sigma_ind);
      sigma_points_[sigma_ind + 1 + STATE_SIZE] =
        state_ - weighted_covar_sqrt_.col(sigma_ind);
    }
  }

  // We don't want to update everything, so we need to build matrices that only
  // update the measured parts of our state vector

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

  size_t updateSize = update_indices.size();

  // Now set up the relevant matrices
  Eigen::VectorXd state_subset(updateSize);       // x (in most literature)
  Eigen::VectorXd measurement_subset(updateSize);  // z
  Eigen::MatrixXd measurement_covariance_subset(updateSize, updateSize);  // R
  Eigen::MatrixXd state_to_measurement_subset(updateSize, STATE_SIZE);   // H
  Eigen::MatrixXd kalman_gain_subset(STATE_SIZE, updateSize);            // K
  Eigen::VectorXd innovation_subset(updateSize);  // z - Hx
  Eigen::VectorXd predicted_measurement(updateSize);
  Eigen::VectorXd sigma_diff(updateSize);
  Eigen::MatrixXd predicted_meas_covar(updateSize, updateSize);
  Eigen::MatrixXd cross_covar(STATE_SIZE, updateSize);

  std::vector<Eigen::VectorXd> sigma_point_measurements(
    sigma_points_.size(), Eigen::VectorXd(updateSize));

  state_subset.setZero();
  measurement_subset.setZero();
  measurement_covariance_subset.setZero();
  state_to_measurement_subset.setZero();
  kalman_gain_subset.setZero();
  innovation_subset.setZero();
  predicted_measurement.setZero();
  predicted_meas_covar.setZero();
  cross_covar.setZero();

  // Now build the sub-matrices from the full-sized matrices
  for (size_t i = 0; i < updateSize; ++i) {
    measurement_subset(i) = measurement.measurement_(update_indices[i]);
    state_subset(i) = state_(update_indices[i]);

    for (size_t j = 0; j < updateSize; ++j) {
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
      measurement_covariance_subset(i, i) = 1e-9;

      FB_DEBUG(
        "WARNING: measurement had very small error covariance for index " <<
          update_indices[i] <<
          ". Adding some noise to maintain filter stability.\n");
    }
  }

  // The state-to-measurement function, h, will now be a measurement_size x
  // full_state_size matrix, with ones in the (i, i) locations of the values to
  // be updated
  for (size_t i = 0; i < updateSize; ++i) {
    state_to_measurement_subset(i, update_indices[i]) = 1;
  }

  FB_DEBUG(
    "Current state subset is:\n" <<
      state_subset << "\nMeasurement subset is:\n" <<
      measurement_subset << "\nMeasurement covariance subset is:\n" <<
      measurement_covariance_subset <<
      "\nState-to-measurement subset is:\n" <<
      state_to_measurement_subset << "\n");

  // (1) Generate sigma points, use them to generate a predicted measurement
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind) {
    sigma_point_measurements[sigma_ind] =
      state_to_measurement_subset * sigma_points_[sigma_ind];
    predicted_measurement.noalias() +=
      state_weights_[sigma_ind] * sigma_point_measurements[sigma_ind];
  }

  // (2) Use the sigma point measurements and predicted measurement to compute a
  // predicted measurement covariance matrix P_zz and a state/measurement
  // cross-covariance matrix P_xz.
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind) {
    sigma_diff = sigma_point_measurements[sigma_ind] - predicted_measurement;
    predicted_meas_covar.noalias() +=
      covar_weights_[sigma_ind] * (sigma_diff * sigma_diff.transpose());
    cross_covar.noalias() +=
      covar_weights_[sigma_ind] *
      ((sigma_points_[sigma_ind] - state_) * sigma_diff.transpose());
  }

  // (3) Compute the Kalman gain, making sure to use the actual measurement
  // covariance: K = P_xz * (P_zz + R)^-1
  Eigen::MatrixXd inv_innov_cov =
    (predicted_meas_covar + measurement_covariance_subset).inverse();
  kalman_gain_subset = cross_covar * inv_innov_cov;

  // (4) Apply the gain to the difference between the actual and predicted
  // measurements: x = x + K(z - z_hat)
  innovation_subset = (measurement_subset - predicted_measurement);

  // Wrap angles in the innovation
  for (size_t i = 0; i < updateSize; ++i) {
    if (update_indices[i] == StateMemberRoll ||
      update_indices[i] == StateMemberPitch ||
      update_indices[i] == StateMemberYaw)
    {
      innovation_subset(i) = ::angles::normalize_angle(innovation_subset(i));
    }
  }

  // (5) Check Mahalanobis distance of innovation
  if (checkMahalanobisThreshold(
      innovation_subset, inv_innov_cov,
      measurement.mahalanobis_thresh_))
  {
    state_.noalias() += kalman_gain_subset * innovation_subset;

    // (6) Compute the new estimate error covariance P = P - (K * P_zz * K')
    estimate_error_covariance_.noalias() -=
      (kalman_gain_subset * predicted_meas_covar *
      kalman_gain_subset.transpose());

    wrapStateAngles();

    // Mark that we need to re-compute sigma points for successive corrections
    uncorrected_ = false;

    FB_DEBUG(
      "Predicated measurement covariance is:\n" <<
        predicted_meas_covar << "\nCross covariance is:\n" <<
        cross_covar << "\nKalman gain subset is:\n" <<
        kalman_gain_subset << "\nInnovation:\n" <<
        innovation_subset << "\nCorrected full state is:\n" <<
        state_ << "\nCorrected full estimate error covariance is:\n" <<
        estimate_error_covariance_ <<
        "\n\n---------------------- /Ukf::correct ----------------------\n");
  }
}

void Ukf::predict(
  const rclcpp::Time & reference_time,
  const rclcpp::Duration & delta)
{
  const double delta_sec = filter_utilities::toSec(delta);

  FB_DEBUG(
    "---------------------- Ukf::predict ----------------------\n" <<
      "delta is " << delta_sec << "\nstate is " << state_ << "\n");

  double roll = state_(StateMemberRoll);
  double pitch = state_(StateMemberPitch);
  double yaw = state_(StateMemberYaw);

  // We'll need these trig calculations a lot.
  double sp = ::sin(pitch);
  double cp = ::cos(pitch);
  double cpi = 1.0 / cp;
  double tp = sp * cpi;

  double sr = ::sin(roll);
  double cr = ::cos(roll);

  double sy = ::sin(yaw);
  double cy = ::cos(yaw);

  prepareControl(reference_time, delta);

  // Prepare the transfer function
  transfer_function_(StateMemberX, StateMemberVx) = cy * cp * delta_sec;
  transfer_function_(StateMemberX, StateMemberVy) =
    (cy * sp * sr - sy * cr) * delta_sec;
  transfer_function_(StateMemberX, StateMemberVz) =
    (cy * sp * cr + sy * sr) * delta_sec;
  transfer_function_(StateMemberX, StateMemberAx) =
    0.5 * transfer_function_(StateMemberX, StateMemberVx) * delta_sec;
  transfer_function_(StateMemberX, StateMemberAy) =
    0.5 * transfer_function_(StateMemberX, StateMemberVy) * delta_sec;
  transfer_function_(StateMemberX, StateMemberAz) =
    0.5 * transfer_function_(StateMemberX, StateMemberVz) * delta_sec;
  transfer_function_(StateMemberY, StateMemberVx) = sy * cp * delta_sec;
  transfer_function_(StateMemberY, StateMemberVy) =
    (sy * sp * sr + cy * cr) * delta_sec;
  transfer_function_(StateMemberY, StateMemberVz) =
    (sy * sp * cr - cy * sr) * delta_sec;
  transfer_function_(StateMemberY, StateMemberAx) =
    0.5 * transfer_function_(StateMemberY, StateMemberVx) * delta_sec;
  transfer_function_(StateMemberY, StateMemberAy) =
    0.5 * transfer_function_(StateMemberY, StateMemberVy) * delta_sec;
  transfer_function_(StateMemberY, StateMemberAz) =
    0.5 * transfer_function_(StateMemberY, StateMemberVz) * delta_sec;
  transfer_function_(StateMemberZ, StateMemberVx) = -sp * delta_sec;
  transfer_function_(StateMemberZ, StateMemberVy) = cp * sr * delta_sec;
  transfer_function_(StateMemberZ, StateMemberVz) = cp * cr * delta_sec;
  transfer_function_(StateMemberZ, StateMemberAx) =
    0.5 * transfer_function_(StateMemberZ, StateMemberVx) * delta_sec;
  transfer_function_(StateMemberZ, StateMemberAy) =
    0.5 * transfer_function_(StateMemberZ, StateMemberVy) * delta_sec;
  transfer_function_(StateMemberZ, StateMemberAz) =
    0.5 * transfer_function_(StateMemberZ, StateMemberVz) * delta_sec;
  transfer_function_(StateMemberRoll, StateMemberVroll) = delta_sec;
  transfer_function_(StateMemberRoll, StateMemberVpitch) = sr * tp * delta_sec;
  transfer_function_(StateMemberRoll, StateMemberVyaw) = cr * tp * delta_sec;
  transfer_function_(StateMemberPitch, StateMemberVpitch) = cr * delta_sec;
  transfer_function_(StateMemberPitch, StateMemberVyaw) = -sr * delta_sec;
  transfer_function_(StateMemberYaw, StateMemberVpitch) = sr * cpi * delta_sec;
  transfer_function_(StateMemberYaw, StateMemberVyaw) = cr * cpi * delta_sec;
  transfer_function_(StateMemberVx, StateMemberAx) = delta_sec;
  transfer_function_(StateMemberVy, StateMemberAy) = delta_sec;
  transfer_function_(StateMemberVz, StateMemberAz) = delta_sec;

  // (1) Take the square root of a small fraction of the
  // estimate_error_covariance_ using LL' decomposition
  weighted_covar_sqrt_ =
    ((STATE_SIZE + lambda_) * estimate_error_covariance_).llt().matrixL();

  // (2) Compute sigma points *and* pass them through the transfer function to
  // save the extra loop

  // First sigma point is the current state
  sigma_points_[0] = transfer_function_ * state_;

  // Next STATE_SIZE sigma points are state + weighted_covar_sqrt_[ith column]
  // STATE_SIZE sigma points after that are state - weighted_covar_sqrt_[ith
  // column]
  for (size_t sigma_ind = 0; sigma_ind < STATE_SIZE; ++sigma_ind) {
    sigma_points_[sigma_ind + 1] =
      transfer_function_ * (state_ + weighted_covar_sqrt_.col(sigma_ind));
    sigma_points_[sigma_ind + 1 + STATE_SIZE] =
      transfer_function_ * (state_ - weighted_covar_sqrt_.col(sigma_ind));
  }

  // (3) Sum the weighted sigma points to generate a new state prediction
  state_.setZero();
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind) {
    state_.noalias() += state_weights_[sigma_ind] * sigma_points_[sigma_ind];
  }

  // (4) Now us the sigma points and the predicted state to compute a predicted
  // covariance
  estimate_error_covariance_.setZero();
  Eigen::VectorXd sigma_diff(STATE_SIZE);
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind) {
    sigma_diff = (sigma_points_[sigma_ind] - state_);
    estimate_error_covariance_.noalias() +=
      covar_weights_[sigma_ind] * (sigma_diff * sigma_diff.transpose());
  }

  // (5) Not strictly in the theoretical UKF formulation, but necessary here
  // to ensure that we actually incorporate the process_noise_covariance_
  Eigen::MatrixXd * process_noise_covariance = &process_noise_covariance_;

  if (use_dynamic_process_noise_covariance_) {
    computeDynamicProcessNoiseCovariance(state_);
    process_noise_covariance = &dynamic_process_noise_covariance_;
  }

  estimate_error_covariance_.noalias() +=
    delta_sec * (*process_noise_covariance);

  // Keep the angles bounded
  wrapStateAngles();

  // Mark that we can keep these sigma points
  uncorrected_ = true;

  FB_DEBUG(
    "Predicted state is:\n" <<
      state_ << "\nPredicted estimate error covariance is:\n" <<
      estimate_error_covariance_ <<
      "\n\n--------------------- /Ukf::predict ----------------------\n");
}

}  // namespace robot_localization
