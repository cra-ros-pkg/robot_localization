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
#include <robot_localization/ukf.hpp>

#include <robot_localization/filter_common.hpp>
#include <robot_localization/filter_utilities.hpp>

#include <Eigen/Cholesky>

#include <vector>


namespace robot_localization
{

Ukf::Ukf(const double alpha, const double kappa, const double beta) :
  FilterBase(),  // Must initialize filter base!
  uncorrected_(true)
{
  const size_t sigma_count = static_cast<size_t>((STATE_SIZE << 1) + 1);
  sigma_points_.resize(sigma_count, Eigen::VectorXd(STATE_SIZE));

  // Prepare constants
  const double state_size_double = static_cast<double>(STATE_SIZE);
  lambda_ = alpha * alpha * (state_size_double + kappa) - state_size_double;

  state_weights_.resize(sigma_count);
  covar_weights_.resize(sigma_count);

  state_weights_[0] = lambda_ / (state_size_double + lambda_);
  covar_weights_[0] = state_weights_[0] + (1 - (alpha * alpha) + beta);
  sigma_points_[0].setZero();
  for (size_t i = 1; i < sigma_count; ++i)
  {
    sigma_points_[i].setZero();
    state_weights_[i] = 1.0 / (2.0 * (state_size_double + lambda_));
    covar_weights_[i] = state_weights_[i];
  }
}

void Ukf::correct(const Measurement & measurement)
{
  FB_DEBUG("---------------------- Ukf::correct ----------------------\n" <<
    "State is:\n" << state_ <<
    "\nMeasurement is:\n" << measurement.measurement_ <<
    "\nMeasurement covariance is:\n" << measurement.covariance_ << "\n");

  // In our implementation, it may be that after we call predict once, we call correct several
  // times in succession (multiple measurements with different time stamps). In that event, the
  // sigma points need to be updated to reflect the current state. Throughout prediction and
  // correction, we attempt to maximize efficiency in Eigen.
  if (!uncorrected_)
  {
    // Take the square root of a small fraction of the
    // estimate_error_covariance_ using LL' decomposition
    weighted_covar_sqrt_ = ((STATE_SIZE + lambda_) * estimate_error_covariance_).llt().matrixL();

    // Compute sigma points

    // First sigma point is the current state
    sigma_points_[0] = state_;

    // Next STATE_SIZE sigma points are state + weighted_covar_sqrt_[ith column]
    // STATE_SIZE sigma points after that are state - weighted_covar_sqrt_[ith
    // column]
    for (size_t sigma_ind = 0; sigma_ind < STATE_SIZE; ++sigma_ind)
    {
      sigma_points_[sigma_ind + 1] = state_ + weighted_covar_sqrt_.col(sigma_ind);
      sigma_points_[sigma_ind + 1 + STATE_SIZE] = state_ - weighted_covar_sqrt_.col(sigma_ind);
    }
  }

  // We don't want to update everything, so we need to build matrices that only
  // update the measured parts of our state vector

  // First, determine how many state vector values we're updating
  std::vector<size_t> update_indices;
  for (size_t i = 0; i < measurement.update_vector_.size(); ++i)
  {
    if (measurement.update_vector_[i])
    {
      // Handle nan and inf values in measurements
      if (std::isnan(measurement.measurement_(i)))
      {
        FB_DEBUG("Value at index " << i << " was nan. Excluding from update.\n");
      } else if (std::isinf(measurement.measurement_(i))) {
        FB_DEBUG("Value at index " << i << " was inf. Excluding from update.\n");
      } else {
        update_indices.push_back(i);
      }
    }
  }

  FB_DEBUG("Update indices are:\n" << update_indices << "\n");

  size_t update_size = update_indices.size();

  // Now set up the relevant matrices
  Eigen::VectorXd state_subset(update_size);                                // x
  Eigen::VectorXd measurement_subset(update_size);                          // z
  Eigen::MatrixXd measurement_covariance_subset(update_size, update_size);  // R
  Eigen::MatrixXd state_to_measurement_subset(update_size, STATE_SIZE);     // H
  Eigen::MatrixXd kalman_gain_subset(STATE_SIZE, update_size);              // K
  Eigen::VectorXd innovation_subset(update_size);                           // z - Hx
  Eigen::VectorXd predicted_measurement(update_size);
  Eigen::VectorXd sigma_diff(update_size);
  Eigen::MatrixXd predicted_meas_covar(update_size, update_size);
  Eigen::MatrixXd cross_covar(STATE_SIZE, update_size);

  std::vector<Eigen::VectorXd> sigma_point_measurements(
    sigma_points_.size(),
    Eigen::VectorXd(update_size));

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
  for (size_t i = 0; i < update_size; ++i)
  {
    measurement_subset(i) = measurement.measurement_(update_indices[i]);
    state_subset(i) = state_(update_indices[i]);

    for (size_t j = 0; j < update_size; ++j)
    {
      measurement_covariance_subset(i, j) =
        measurement.covariance_(update_indices[i], update_indices[j]);
    }

    // Handle negative (read: bad) covariances in the measurement. Rather than exclude the
    // measurement or make up a covariance, just take the absolute value.
    if (measurement_covariance_subset(i, i) < 0)
    {
      FB_DEBUG("WARNING: Negative covariance for index " << i << " of measurement (value is" <<
        measurement_covariance_subset(i, i) << "). Using absolute value...\n");

      measurement_covariance_subset(i, i) = std::abs(measurement_covariance_subset(i, i));
    }

    // If the measurement variance for a given variable is very near 0 (as in e-50 or so) and the
    // variance for that variable in the covariance matrix is also near zero, then the Kalman gain
    // computation will blow up. Really, no measurement can be completely without error, so add a
    // small amount in that case.
    if (measurement_covariance_subset(i, i) < 1e-9)
    {
      measurement_covariance_subset(i, i) = 1e-9;

      FB_DEBUG("WARNING: measurement had very small error covariance for index " <<
        update_indices[i] << ". Adding some noise to maintain filter stability.\n");
    }
  }

  // The state-to-measurement function, h, will now be a measurement_size x full_state_size matrix,
  // with ones in the (i, i) locations of the values to be updated
  for (size_t i = 0; i < update_size; ++i)
  {
    state_to_measurement_subset(i, update_indices[i]) = 1;
  }

  FB_DEBUG("Current state subset is:\n" << state_subset <<
    "\nMeasurement subset is:\n" << measurement_subset <<
    "\nMeasurement covariance subset is:\n" << measurement_covariance_subset <<
    "\nState-to-measurement subset is:\n" << state_to_measurement_subset << "\n");

  // (1) Generate sigma points, use them to generate a predicted measurement
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind)
  {
    sigma_point_measurements[sigma_ind] = state_to_measurement_subset * sigma_points_[sigma_ind];
    predicted_measurement.noalias() +=
      state_weights_[sigma_ind] * sigma_point_measurements[sigma_ind];
  }

  // (2) Use the sigma point measurements and predicted measurement to compute a
  // predicted measurement covariance matrix P_zz and a state/measurement
  // cross-covariance matrix P_xz.
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind)
  {
    sigma_diff = sigma_point_measurements[sigma_ind] - predicted_measurement;
    predicted_meas_covar.noalias() +=
      covar_weights_[sigma_ind] * (sigma_diff * sigma_diff.transpose());
    cross_covar.noalias() +=
      covar_weights_[sigma_ind] * ((sigma_points_[sigma_ind] - state_) * sigma_diff.transpose());
  }

  // (3) Compute the Kalman gain, making sure to use the actual measurement
  // covariance: K = P_xz * (P_zz + R)^-1
  Eigen::MatrixXd inv_innov_cov = (predicted_meas_covar + measurement_covariance_subset).inverse();
  kalman_gain_subset = cross_covar * inv_innov_cov;

  // (4) Apply the gain to the difference between the actual and predicted measurements:
  // x = x + K(z - z_hat)
  innovation_subset = (measurement_subset - predicted_measurement);

  // Wrap angles in the innovation
  for (size_t i = 0; i < update_size; ++i)
  {
    if (update_indices[i] == StateMemberRoll ||
        update_indices[i] == StateMemberPitch ||
        update_indices[i] == StateMemberYaw)
    {
      innovation_subset(i) = filter_utilities::clampRotation(innovation_subset(i));
    }
  }

  // (5) Check Mahalanobis distance of innovation
  if (checkMahalanobisThreshold(innovation_subset, inv_innov_cov,
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
      "Predicated measurement covariance is:\n" << predicted_meas_covar <<
      "\nCross covariance is:\n" << cross_covar <<
      "\nKalman gain subset is:\n" << kalman_gain_subset <<
      "\nInnovation:\n" << innovation_subset <<
      "\nCorrected full state is:\n" << state_ <<
      "\nCorrected full estimate error covariance is:\n" << estimate_error_covariance_ <<
      "\n\n---------------------- /Ukf::correct ----------------------\n");
  }
}

Eigen::VectorXd Ukf::project(const Eigen::VectorXd& state, const double delta_sec)
{
  Eigen::VectorXd projected = state;

  const double roll = state(StateMemberRoll);
  const double pitch = state(StateMemberPitch);
  const double yaw = state(StateMemberYaw);
  const double x_vel = state(StateMemberVx);
  const double y_vel = state(StateMemberVy);
  const double z_vel = state(StateMemberVz);
  const double roll_vel = state(StateMemberVroll);
  const double pitch_vel = state(StateMemberVpitch);
  const double yaw_vel = state(StateMemberVyaw);
  const double x_acc = state(StateMemberAx);
  const double y_acc = state(StateMemberAy);
  const double z_acc = state(StateMemberAz);

  // We'll need these trig calculations a lot.
  const double sp = ::sin(pitch);
  const double cp = ::cos(pitch);
  const double cpi = 1.0 / cp;
  const double tp = sp * cpi;

  const double sr = ::sin(roll);
  const double cr = ::cos(roll);

  const double sy = ::sin(yaw);
  const double cy = ::cos(yaw);

  // (1) Project the state forward: x = Ax + Bu (really, x = f(x, u))
  const double x_vx = cy * cp * delta_sec;
  const double x_vy = (cy * sp * sr - sy * cr) * delta_sec;
  const double x_vz = (cy * sp * cr + sy * sr) * delta_sec;
  const double x_ax = 0.5 * x_vx * delta_sec;
  const double x_ay = 0.5 * x_vy * delta_sec;
  const double x_az = 0.5 * x_vz * delta_sec;
  projected(StateMemberX) +=
    x_vel * x_vx + y_vel * x_vy + z_vel * x_vz + x_acc * x_ax + y_acc * x_ay + z_acc * x_az;

  const double y_vx = sy * cp * delta_sec;
  const double y_vy = (sy * sp * sr + cy * cr) * delta_sec;
  const double y_vz = (sy * sp * cr - cy * sr) * delta_sec;
  const double y_ax = 0.5 * y_vx * delta_sec;
  const double y_ay = 0.5 * y_vy * delta_sec;
  const double y_az = 0.5 * y_vz * delta_sec;
  projected(StateMemberY) +=
    x_vel * y_vx + y_vel * y_vy + z_vel * y_vz + x_acc * y_ax + y_acc * y_ay + z_acc * y_az;

  const double z_vx = -sp * delta_sec;
  const double z_vy = cp * sr * delta_sec;
  const double z_vz = cp * cr * delta_sec;
  const double z_ax = 0.5 * z_vx * delta_sec;
  const double z_ay = 0.5 * z_vy * delta_sec;
  const double z_az = 0.5 * z_vz * delta_sec;
  projected(StateMemberZ) +=
    x_vel * z_vx + y_vel * z_vy + z_vel * z_vz + x_acc * z_ax + y_acc * z_ay + z_acc * z_az;

  const double r_vr = delta_sec;
  const double r_vp = sr * tp * delta_sec;
  const double r_vw = cr * tp * delta_sec;
  projected(StateMemberRoll) += roll_vel * r_vr + pitch_vel * r_vp + yaw_vel * r_vw;

  const double p_vp = cr * delta_sec;
  const double p_vw = -sr * delta_sec;
  projected(StateMemberPitch) += pitch_vel * p_vp + yaw_vel * p_vw;

  const double w_vp = sr * cpi * delta_sec;
  const double w_vw = cr * cpi * delta_sec;
  projected(StateMemberYaw) += pitch_vel * w_vp + yaw_vel * w_vw;

  projected(StateMemberVx) += x_acc * delta_sec;
  projected(StateMemberVy) += y_acc * delta_sec;
  projected(StateMemberVz) += y_acc * delta_sec;

  // We model control terms as accelerations
  projected(StateMemberVroll) += control_acceleration_(ControlMemberVroll) * delta_sec;
  projected(StateMemberVpitch) += control_acceleration_(ControlMemberVpitch) * delta_sec;
  projected(StateMemberVyaw) += control_acceleration_(ControlMemberVyaw) * delta_sec;

  projected(StateMemberAx) = (control_update_vector_[ControlMemberVx] ?
    control_acceleration_(ControlMemberVx) : state(StateMemberAx));
  projected(StateMemberAy) = (control_update_vector_[ControlMemberVy] ?
    control_acceleration_(ControlMemberVy) : state(StateMemberAy));
  projected(StateMemberAz) = (control_update_vector_[ControlMemberVz] ?
    control_acceleration_(ControlMemberVz) : state(StateMemberAz));

  return projected;
}

void Ukf::predict(
  const rclcpp::Time& reference_time,
  const rclcpp::Duration& delta)
{
  const double delta_sec = filter_utilities::toSec(delta);

  FB_DEBUG("---------------------- Ukf::predict ----------------------\n" <<
    "delta is " << delta_sec << "\n" <<
    "state is " << state_ << "\n");

  prepareControl(reference_time, delta);

  // (1) Take the square root of a small fraction of the estimate_error_covariance_ using LL'
  // decomposition
  weighted_covar_sqrt_ = ((STATE_SIZE + lambda_) * estimate_error_covariance_).llt().matrixL();

  // (2) Project all the sigma points forward. The first is just the projection of the current state
  sigma_points_[0] = project(state_, delta_sec);

  // The next STATE_SIZE sigma points are state + weighted_covar_sqrt_[ith column]
  // The STATE_SIZE sigma points after that are state - weighted_covar_sqrt_[ith column]
  for (size_t sigma_ind = 1; sigma_ind <= STATE_SIZE; ++sigma_ind)
  {
    const size_t covar_ind = sigma_ind - 1;
    sigma_points_[sigma_ind] = project(state_ + weighted_covar_sqrt_.col(covar_ind), delta_sec);
    sigma_points_[sigma_ind + STATE_SIZE] =
      project(state_ - weighted_covar_sqrt_.col(covar_ind), delta_sec);
  }

  // (3) Sum the weighted sigma points to generate a new state prediction
  state_.setZero();
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind)
  {
    state_.noalias() += state_weights_[sigma_ind] * sigma_points_[sigma_ind];
  }

  // (4) Now us the sigma points and the predicted state to compute a predicted
  // covariance
  estimate_error_covariance_.setZero();
  Eigen::VectorXd sigma_diff(STATE_SIZE);
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind)
  {
    sigma_diff = (sigma_points_[sigma_ind] - state_);
    estimate_error_covariance_.noalias() +=
      covar_weights_[sigma_ind] * (sigma_diff * sigma_diff.transpose());
  }

  // (5) Not strictly in the theoretical UKF formulation, but necessary here to ensure that we
  // actually incorporate the process_noise_covariance_
  Eigen::MatrixXd* process_noise_covariance = &process_noise_covariance_;

  if (use_dynamic_process_noise_covariance_)
  {
    computeDynamicProcessNoiseCovariance(state_);
    process_noise_covariance = &dynamic_process_noise_covariance_;
  }

  estimate_error_covariance_.noalias() += delta_sec * (*process_noise_covariance);

  // Keep the angles bounded
  wrapStateAngles();

  // Mark that we can keep these sigma points
  uncorrected_ = true;

  FB_DEBUG(
    "Predicted state is:\n" << state_ <<
    "\nPredicted estimate error covariance is:\n" << estimate_error_covariance_ <<
    "\n\n--------------------- /Ukf::predict ----------------------\n");
}

}  // namespace robot_localization
