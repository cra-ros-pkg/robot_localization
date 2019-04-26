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
#include <robot_localization/ekf.hpp>

#include <robot_localization/filter_common.hpp>
#include <robot_localization/filter_utilities.hpp>

#include <Eigen/Dense>
#include <rclcpp/duration.hpp>

#include <vector>


namespace robot_localization
{

Ekf::Ekf() :
  FilterBase()  // Must initialize filter base!
{}

void Ekf::correct(const Measurement& measurement)
{
  FB_DEBUG("---------------------- Ekf::correct ----------------------\n" <<
    "State is:\n" << state_ <<
    "\nTopic is:\n" << measurement.topic_name_ <<
    "\nMeasurement is:\n" << measurement.measurement_ <<
    "\nMeasurement topic name is:\n" << measurement.topic_name_ <<
    "\n\nMeasurement covariance is:\n" << measurement.covariance_ << "\n");

  // We don't want to update everything, so we need to build matrices that only
  // update the measured parts of our state vector. Throughout prediction and
  // correction, we attempt to maximize efficiency in Eigen.

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
  Eigen::MatrixXd state_to_measurement_subset(update_size, state_.rows());  // H
  Eigen::MatrixXd kalman_gain_subset(state_.rows(), update_size);           // K
  Eigen::VectorXd innovation_subset(update_size);                           // z - Hx

  state_subset.setZero();
  measurement_subset.setZero();
  measurement_covariance_subset.setZero();
  state_to_measurement_subset.setZero();
  kalman_gain_subset.setZero();
  innovation_subset.setZero();

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
      FB_DEBUG("WARNING: measurement had very small error covariance for index " <<
        update_indices[i] << ". Adding some noise to maintain filter stability.\n");

      measurement_covariance_subset(i, i) = 1e-9;
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

  // (1) Compute the Kalman gain: K = (PH') / (HPH' + R)
  Eigen::MatrixXd pht = estimate_error_covariance_ * state_to_measurement_subset.transpose();
  Eigen::MatrixXd hphr_inverse =
    (state_to_measurement_subset * pht + measurement_covariance_subset).inverse();
  kalman_gain_subset.noalias() = pht * hphr_inverse;

  innovation_subset = (measurement_subset - state_subset);

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

  // (2) Check Mahalanobis distance between mapped measurement and state.
  if (checkMahalanobisThreshold(innovation_subset, hphr_inverse, measurement.mahalanobis_thresh_))
  {
    // (3) Apply the gain to the difference between the state and measurement: x
    // = x + K(z - Hx)
    state_.noalias() += kalman_gain_subset * innovation_subset;

    // (4) Update the estimate error covariance using the Joseph form: (I - KH)P(I - KH)' + KRK'
    Eigen::MatrixXd gain_residual = identity_;
    gain_residual.noalias() -= kalman_gain_subset * state_to_measurement_subset;
    estimate_error_covariance_ =
      gain_residual * estimate_error_covariance_ * gain_residual.transpose();
    estimate_error_covariance_.noalias() +=
      kalman_gain_subset *
      measurement_covariance_subset *
      kalman_gain_subset.transpose();

    // Handle wrapping of angles
    wrapStateAngles();

    FB_DEBUG(
      "Kalman gain subset is:\n" << kalman_gain_subset <<
      "\nInnovation is:\n" << innovation_subset <<
      "\nCorrected full state is:\n" << state_ <<
      "\nCorrected full estimate error covariance is:\n" << estimate_error_covariance_ <<
      "\n\n---------------------- /Ekf::correct ----------------------\n");
  }
}

void Ekf::predict(const rclcpp::Time& reference_time, const rclcpp::Duration& delta)
{
  const double delta_sec = filter_utilities::toSec(delta);

  FB_DEBUG("---------------------- Ekf::predict ----------------------\n" <<
    "delta is " << delta_sec << "\n" <<
    "state is " << state_ << "\n");

  prepareControl(reference_time, delta);

  const double roll = state_(StateMemberRoll);
  const double pitch = state_(StateMemberPitch);
  const double yaw = state_(StateMemberYaw);
  const double x_vel = state_(StateMemberVx);
  const double y_vel = state_(StateMemberVy);
  const double z_vel = state_(StateMemberVz);
  const double roll_vel = state_(StateMemberVroll);
  const double pitch_vel = state_(StateMemberVpitch);
  const double yaw_vel = state_(StateMemberVyaw);
  const double x_acc = state_(StateMemberAx);
  const double y_acc = state_(StateMemberAy);
  const double z_acc = state_(StateMemberAz);

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
  state_(StateMemberX) +=
    x_vel * x_vx + y_vel * x_vy + z_vel * x_vz + x_acc * x_ax + y_acc * x_ay + z_acc * x_az;

  const double y_vx = sy * cp * delta_sec;
  const double y_vy = (sy * sp * sr + cy * cr) * delta_sec;
  const double y_vz = (sy * sp * cr - cy * sr) * delta_sec;
  const double y_ax = 0.5 * y_vx * delta_sec;
  const double y_ay = 0.5 * y_vy * delta_sec;
  const double y_az = 0.5 * y_vz * delta_sec;
  state_(StateMemberY) +=
    x_vel * y_vx + y_vel * y_vy + z_vel * y_vz + x_acc * y_ax + y_acc * y_ay + z_acc * y_az;

  const double z_vx = -sp * delta_sec;
  const double z_vy = cp * sr * delta_sec;
  const double z_vz = cp * cr * delta_sec;
  const double z_ax = 0.5 * z_vx * delta_sec;
  const double z_ay = 0.5 * z_vy * delta_sec;
  const double z_az = 0.5 * z_vz * delta_sec;
  state_(StateMemberZ) +=
    x_vel * z_vx + y_vel * z_vy + z_vel * z_vz + x_acc * z_ax + y_acc * z_ay + z_acc * z_az;

  const double r_vr = delta_sec;
  const double r_vp = sr * tp * delta_sec;
  const double r_vw = cr * tp * delta_sec;
  state_(StateMemberRoll) += roll_vel * r_vr + pitch_vel * r_vp + yaw_vel * r_vw;

  const double p_vp = cr * delta_sec;
  const double p_vw = -sr * delta_sec;
  state_(StateMemberPitch) += pitch_vel * p_vp + yaw_vel * p_vw;

  const double w_vp = sr * cpi * delta_sec;
  const double w_vw = cr * cpi * delta_sec;
  state_(StateMemberYaw) += pitch_vel * w_vp + yaw_vel * w_vw;

  state_(StateMemberVx) += x_acc * delta_sec;
  state_(StateMemberVy) += y_acc * delta_sec;
  state_(StateMemberVz) += y_acc * delta_sec;

  // We model control terms as accelerations
  state_(StateMemberVroll) += control_acceleration_(ControlMemberVroll) * delta_sec;
  state_(StateMemberVpitch) += control_acceleration_(ControlMemberVpitch) * delta_sec;
  state_(StateMemberVyaw) += control_acceleration_(ControlMemberVyaw) * delta_sec;

  state_(StateMemberAx) = (control_update_vector_[ControlMemberVx] ?
    control_acceleration_(ControlMemberVx) : state_(StateMemberAx));
  state_(StateMemberAy) = (control_update_vector_[ControlMemberVy] ?
    control_acceleration_(ControlMemberVy) : state_(StateMemberAy));
  state_(StateMemberAz) = (control_update_vector_[ControlMemberVz] ?
    control_acceleration_(ControlMemberVz) : state_(StateMemberAz));

  // Handle wrapping
  wrapStateAngles();

  // (3) Prepare the transfer function Jacobian. This function is analytically derived from the
  // transfer function.
  double x_coeff = 0.0;
  double y_coeff = 0.0;
  double z_coeff = 0.0;
  const double one_half_at_squared = 0.5 * delta_sec * delta_sec;

  y_coeff = cy * sp * cr + sy * sr;
  z_coeff = -cy * sp * sr + sy * cr;
  const double dFx_dR = (y_coeff * y_vel + z_coeff * z_vel) * delta_sec +
    (y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
  const double dFR_dR = 1.0 + (cr * tp * pitch_vel - sr * tp * yaw_vel) * delta_sec;

  x_coeff = -cy * sp;
  y_coeff = cy * cp * sr;
  z_coeff = cy * cp * cr;
  const double dFx_dP =
    (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_sec +
    (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
  const double dFR_dP = (cpi * cpi * sr * pitch_vel + cpi * cpi * cr * yaw_vel) * delta_sec;

  x_coeff = -sy * cp;
  y_coeff = -sy * sp * sr - cy * cr;
  z_coeff = -sy * sp * cr + cy * sr;
  const double dFx_dY =
    (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_sec +
    (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;

  y_coeff = sy * sp * cr - cy * sr;
  z_coeff = -sy * sp * sr - cy * cr;
  const double dFy_dR =
    (y_coeff * y_vel + z_coeff * z_vel) * delta_sec +
    (y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
  const double dFP_dR = (-sr * pitch_vel - cr * yaw_vel) * delta_sec;

  x_coeff = -sy * sp;
  y_coeff = sy * cp * sr;
  z_coeff = sy * cp * cr;
  const double dFy_dP =
    (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_sec +
    (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;

  x_coeff = cy * cp;
  y_coeff = cy * sp * sr - sy * cr;
  z_coeff = cy * sp * cr + sy * sr;
  const double dFy_dY =
    (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_sec +
    (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;

  y_coeff = cp * cr;
  z_coeff = -cp * sr;
  const double dFz_dR =
    (y_coeff * y_vel + z_coeff * z_vel) * delta_sec +
    (y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
  const double dFY_dR = (cr * cpi * pitch_vel - sr * cpi * yaw_vel) * delta_sec;

  x_coeff = -cp;
  y_coeff = -sp * sr;
  z_coeff = -sp * cr;
  const double dFz_dP =
    (x_coeff * x_vel + y_coeff * y_vel + z_coeff * z_vel) * delta_sec +
    (x_coeff * x_acc + y_coeff * y_acc + z_coeff * z_acc) * one_half_at_squared;
  const double dFY_dP = (sr * tp * cpi * pitch_vel - cr * tp * cpi * yaw_vel) * delta_sec;

  // Much of the transfer function Jacobian is identical to the transfer function
  transfer_function_jacobian_(StateMemberX, StateMemberVx) = x_vx;
  transfer_function_jacobian_(StateMemberX, StateMemberVy) = x_vy;
  transfer_function_jacobian_(StateMemberX, StateMemberVz) = x_vz;
  transfer_function_jacobian_(StateMemberX, StateMemberAx) = x_ax;
  transfer_function_jacobian_(StateMemberX, StateMemberAy) = x_ay;
  transfer_function_jacobian_(StateMemberX, StateMemberAz) = x_az;
  transfer_function_jacobian_(StateMemberY, StateMemberVx) = y_vx;
  transfer_function_jacobian_(StateMemberY, StateMemberVy) = y_vy;
  transfer_function_jacobian_(StateMemberY, StateMemberVz) = y_vz;
  transfer_function_jacobian_(StateMemberY, StateMemberAx) = y_ax;
  transfer_function_jacobian_(StateMemberY, StateMemberAy) = y_ay;
  transfer_function_jacobian_(StateMemberY, StateMemberAz) = y_az;
  transfer_function_jacobian_(StateMemberZ, StateMemberVx) = z_vx;
  transfer_function_jacobian_(StateMemberZ, StateMemberVy) = z_vy;
  transfer_function_jacobian_(StateMemberZ, StateMemberVz) = z_vz;
  transfer_function_jacobian_(StateMemberZ, StateMemberAx) = z_ax;
  transfer_function_jacobian_(StateMemberZ, StateMemberAy) = z_ay;
  transfer_function_jacobian_(StateMemberZ, StateMemberAz) = z_az;
  transfer_function_jacobian_(StateMemberRoll, StateMemberVroll) = r_vr;
  transfer_function_jacobian_(StateMemberRoll, StateMemberVpitch) = r_vp;
  transfer_function_jacobian_(StateMemberRoll, StateMemberVyaw) = r_vw;
  transfer_function_jacobian_(StateMemberPitch, StateMemberVpitch) = p_vp;
  transfer_function_jacobian_(StateMemberPitch, StateMemberVyaw) = p_vw;
  transfer_function_jacobian_(StateMemberYaw, StateMemberVpitch) = w_vp;
  transfer_function_jacobian_(StateMemberYaw, StateMemberVyaw) = w_vw;
  transfer_function_jacobian_(StateMemberVx, StateMemberAx) = delta_sec;
  transfer_function_jacobian_(StateMemberVy, StateMemberAy) = delta_sec;
  transfer_function_jacobian_(StateMemberVz, StateMemberAz) = delta_sec;

  transfer_function_jacobian_(StateMemberX, StateMemberRoll) = dFx_dR;
  transfer_function_jacobian_(StateMemberX, StateMemberPitch) = dFx_dP;
  transfer_function_jacobian_(StateMemberX, StateMemberYaw) = dFx_dY;
  transfer_function_jacobian_(StateMemberY, StateMemberRoll) = dFy_dR;
  transfer_function_jacobian_(StateMemberY, StateMemberPitch) = dFy_dP;
  transfer_function_jacobian_(StateMemberY, StateMemberYaw) = dFy_dY;
  transfer_function_jacobian_(StateMemberZ, StateMemberRoll) = dFz_dR;
  transfer_function_jacobian_(StateMemberZ, StateMemberPitch) = dFz_dP;
  transfer_function_jacobian_(StateMemberRoll, StateMemberRoll) = dFR_dR;
  transfer_function_jacobian_(StateMemberRoll, StateMemberPitch) = dFR_dP;
  transfer_function_jacobian_(StateMemberPitch, StateMemberRoll) = dFP_dR;
  transfer_function_jacobian_(StateMemberYaw, StateMemberRoll) = dFY_dR;
  transfer_function_jacobian_(StateMemberYaw, StateMemberPitch) = dFY_dP;

  FB_DEBUG("\nTransfer function Jacobian is:\n" << transfer_function_jacobian_ <<
    "\nProcess noise covariance is:\n" << process_noise_covariance_ <<
    "\nCurrent state is:\n" << state_ << "\n");

  Eigen::MatrixXd* process_noise_covariance = &process_noise_covariance_;

  if (use_dynamic_process_noise_covariance_)
  {
    computeDynamicProcessNoiseCovariance(state_);
    process_noise_covariance = &dynamic_process_noise_covariance_;
  }

  FB_DEBUG("Predicted state is:\n" << state_ <<
    "\nCurrent estimate error covariance is:\n" << estimate_error_covariance_ << "\n");

  // (3) Project the error forward: P = J * P * J' + Q
  estimate_error_covariance_ =
    (transfer_function_jacobian_ *
     estimate_error_covariance_ *
     transfer_function_jacobian_.transpose());
  estimate_error_covariance_.noalias() += delta_sec * (*process_noise_covariance);

  FB_DEBUG("Predicted estimate error covariance is:\n" << estimate_error_covariance_ <<
    "\n\n--------------------- /Ekf::predict ----------------------\n");
}

}  // namespace robot_localization
