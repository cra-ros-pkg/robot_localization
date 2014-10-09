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

#include <XmlRpcException.h>

#include <sstream>
#include <iomanip>
#include <limits>

namespace RobotLocalization
{
  Ekf::Ekf() :
    FilterBase() // Must initialize filter base!
  {
  }

  Ekf::~Ekf()
  {
  }

  void Ekf::correct(const Measurement &measurement)
  {
    if (getDebug())
    {
      *debugStream_ << "---------------------- Ekf::correct ----------------------\n";
      *debugStream_ << "State is:\n";
      *debugStream_ << state_ << "\n";
      *debugStream_ << "Measurement is:\n";
      *debugStream_ << measurement.measurement_ << "\n";
      *debugStream_ << "Measurement covariance is:\n";
      *debugStream_ << measurement.covariance_ << "\n";
    }

    // We don't want to update everything, so we need to build matrices that only update
    // the measured parts of our state vector

    // First, determine how many state vector values we're updating
    std::vector<size_t> updateIndices;
    for (size_t i = 0; i < measurement.updateVector_.size(); ++i)
    {
      if (measurement.updateVector_[i])
      {
        // Handle nan and inf values in measurements
        if (std::isnan(measurement.measurement_(i)))
        {
          if (getDebug())
          {
            *debugStream_ << "Value at index " << i << " was nan. Excluding from update.\n";
          }
        }
        else if (std::isinf(measurement.measurement_(i)))
        {
          if (getDebug())
          {
            *debugStream_ << "Value at index " << i << " was inf. Excluding from update.\n";
          }
        }
        else
        {
          updateIndices.push_back(i);
        }
      }
    }

    if (getDebug())
    {
      *debugStream_ << "Update indices are:\n";
      *debugStream_ << updateIndices << "\n";
    }

    size_t updateSize = updateIndices.size();

    // Now set up the relevant matrices
    Eigen::VectorXd stateSubset(updateSize);                             // x (in most literature)
    Eigen::VectorXd measurementSubset(updateSize);                       // z
    Eigen::MatrixXd measurementCovarianceSubset(updateSize, updateSize); // R
    Eigen::MatrixXd stateToMeasurementSubset(updateSize, state_.rows()); // H
    Eigen::MatrixXd kalmanGainSubset(state_.rows(), updateSize);         // K
    Eigen::VectorXd innovationSubset(updateSize);                        // z - Hx

    stateSubset.setZero();
    measurementSubset.setZero();
    measurementCovarianceSubset.setZero();
    stateToMeasurementSubset.setZero();
    kalmanGainSubset.setZero();
    innovationSubset.setZero();

    // Now build the sub-matrices from the full-sized matrices
    for (size_t i = 0; i < updateSize; ++i)
    {
      measurementSubset(i) = measurement.measurement_(updateIndices[i]);
      stateSubset(i) = state_(updateIndices[i]);

      for (size_t j = 0; j < updateSize; ++j)
      {
        measurementCovarianceSubset(i, j) = measurement.covariance_(updateIndices[i], updateIndices[j]);
      }

      // Handle negative (read: bad) covariances in the measurement. Rather
      // than exclude the measurement or make up a covariance, just take
      // the absolute value.
      if (measurementCovarianceSubset(i, i) < 0)
      {
        if (getDebug())
        {
          *debugStream_ << "WARNING: Negative covariance for index " << i << " of measurement (value is" << measurementCovarianceSubset(i, i)
            << "). Using absolute value...\n";
        }

        measurementCovarianceSubset(i, i) = ::fabs(measurementCovarianceSubset(i, i));
      }

      // If the measurement variance for a given variable is very
      // near 0 (as in e-50 or so) and the variance for that
      // variable in the covariance matrix is also near zero, then
      // the Kalman gain computation will blow up. Really, no
      // measurement can be completely without error, so add a small
      // amount in that case.
      if (measurementCovarianceSubset(i, i) < 1e-6)
      {
        measurementCovarianceSubset(i, i) = 1e-6;

        if (getDebug())
        {
          *debugStream_ << "WARNING: measurement had very small error covariance for index " << updateIndices[i] << ". Adding some noise to maintain filter stability.\n";
        }
      }
    }

    // The state-to-measurement function, h, will now be a measurement_size x full_state_size
    // matrix, with ones in the (i, i) locations of the values to be updated
    for (size_t i = 0; i < updateSize; ++i)
    {
      stateToMeasurementSubset(i, updateIndices[i]) = 1;
    }

    if (getDebug())
    {
      *debugStream_ << "Current state subset is:\n";
      *debugStream_ << stateSubset << "\n";
      *debugStream_ << "Measurement subset is:\n";
      *debugStream_ << measurementSubset << "\n";
      *debugStream_ << "Measurement covariance subset is:\n";
      *debugStream_ << measurementCovarianceSubset << "\n";
      *debugStream_ << "State-to-measurement subset is:\n";
      *debugStream_ << stateToMeasurementSubset << "\n";
    }

    // (1) Compute the Kalman gain: K = (PH') / (HPH' + R)
    Eigen::MatrixXd pht = estimateErrorCovariance_ * stateToMeasurementSubset.transpose();
    kalmanGainSubset = pht * (stateToMeasurementSubset * pht + measurementCovarianceSubset).inverse();

    // (2) Apply the gain to the difference between the state and measurement: x = x + K(z - Hx)
    innovationSubset = (measurementSubset - stateSubset);

    // Wrap angles in the innovation
    for (size_t i = 0; i < updateSize; ++i)
    {
      if (updateIndices[i] == StateMemberRoll || updateIndices[i] == StateMemberPitch || updateIndices[i] == StateMemberYaw)
      {
        if (innovationSubset(i) < -pi_)
        {
          innovationSubset(i) += tau_;
        }
        else if (innovationSubset(i) > pi_)
        {
          innovationSubset(i) -= tau_;
        }
      }
    }

    state_ = state_ + kalmanGainSubset * innovationSubset;

    // (3) Update the estimate error covariance using the Joseph form: (I - KH)P(I - KH)' + KRK'
    Eigen::MatrixXd gainResidual = identity_ - kalmanGainSubset * stateToMeasurementSubset;
    estimateErrorCovariance_ = gainResidual * estimateErrorCovariance_.eval() * gainResidual.transpose() +
        kalmanGainSubset * measurementCovarianceSubset * kalmanGainSubset.transpose();

    // Handle wrapping of angles
    wrapStateAngles();

    if (getDebug())
    {
      *debugStream_ << "Kalman gain subset is:\n";
      *debugStream_ << kalmanGainSubset << "\n";
      *debugStream_ << "Innovation:\n";
      *debugStream_ << innovationSubset << "\n\n";
      *debugStream_ << "Corrected full state is:\n";
      *debugStream_ << state_ << "\n";
      *debugStream_ << "Corrected full estimate error covariance is:\n";
      *debugStream_ << estimateErrorCovariance_ << "\n";
      *debugStream_ << "\n---------------------- /Ekf::correct ----------------------\n";
    }
  }

  void Ekf::predict(const double delta)
  {
    if (getDebug())
    {
      *debugStream_ << "---------------------- Ekf::predict ----------------------\n";
      *debugStream_ << "delta is " << delta << "\n";
      *debugStream_ << "state is " << state_ << "\n";
    }

    double roll = state_(StateMemberRoll);
    double pitch = state_(StateMemberPitch);
    double yaw = state_(StateMemberYaw);
    double xVel = state_(StateMemberVx);
    double yVel = state_(StateMemberVy);
    double zVel = state_(StateMemberVz);
    double xAcc = state_(StateMemberAx);
    double yAcc = state_(StateMemberAy);
    double zAcc = state_(StateMemberAz);

    // We'll need these trig calculations a lot.
    double cr = cos(roll);
    double cp = cos(pitch);
    double cy = cos(yaw);
    double sr = sin(roll);
    double sp = sin(pitch);
    double sy = sin(yaw);

    // Prepare the transfer function
    transferFunction_(StateMemberX, StateMemberVx) = cy * cp * delta;
    transferFunction_(StateMemberX, StateMemberVy) = (cy * sp * sr - sy * cr) * delta;
    transferFunction_(StateMemberX, StateMemberVz) = (cy * sp * cr + sy * sr) * delta;
    transferFunction_(StateMemberX, StateMemberAx) = 0.5 * transferFunction_(StateMemberX, StateMemberVx) * delta;
    transferFunction_(StateMemberX, StateMemberAy) = 0.5 * transferFunction_(StateMemberX, StateMemberVy) * delta;
    transferFunction_(StateMemberX, StateMemberAz) = 0.5 * transferFunction_(StateMemberX, StateMemberVz) * delta;
    transferFunction_(StateMemberY, StateMemberVx) = sy * cp * delta;
    transferFunction_(StateMemberY, StateMemberVy) = (sy * sp * sr + cy * cr) * delta;
    transferFunction_(StateMemberY, StateMemberVz) = (sy * sp * cr - cy * sr) * delta;
    transferFunction_(StateMemberY, StateMemberAx) = 0.5 * transferFunction_(StateMemberY, StateMemberVx) * delta;
    transferFunction_(StateMemberY, StateMemberAy) = 0.5 * transferFunction_(StateMemberY, StateMemberVy) * delta;
    transferFunction_(StateMemberY, StateMemberAz) = 0.5 * transferFunction_(StateMemberY, StateMemberVz) * delta;
    transferFunction_(StateMemberZ, StateMemberVx) = -sp * delta;
    transferFunction_(StateMemberZ, StateMemberVy) = cp * sr * delta;
    transferFunction_(StateMemberZ, StateMemberVz) = cp * cr * delta;
    transferFunction_(StateMemberZ, StateMemberAx) = 0.5 * transferFunction_(StateMemberZ, StateMemberVx) * delta;
    transferFunction_(StateMemberZ, StateMemberAy) = 0.5 * transferFunction_(StateMemberZ, StateMemberVy) * delta;
    transferFunction_(StateMemberZ, StateMemberAz) = 0.5 * transferFunction_(StateMemberZ, StateMemberVz) * delta;
    transferFunction_(StateMemberRoll, StateMemberVroll) = delta;
    transferFunction_(StateMemberPitch, StateMemberVpitch) = delta;
    transferFunction_(StateMemberYaw, StateMemberVyaw) = delta;
    transferFunction_(StateMemberVx, StateMemberAx) = delta;
    transferFunction_(StateMemberVy, StateMemberAy) = delta;
    transferFunction_(StateMemberVz, StateMemberAz) = delta;

    // Prepare the transfer function Jacobian. This function is analytically derived from the
    // transfer function.
    double xCoeff = 0.0;
    double yCoeff = 0.0;
    double zCoeff = 0.0;
    double oneHalfATSquared = 0.5 * delta * delta;

    yCoeff = cy * sp * cr + sy * sr;
    zCoeff = -cy * sp * sr + sy * cr;
    double dF0dr = (yCoeff * yVel + zCoeff * zVel) * delta +
                   (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    xCoeff = -cy * sp;
    yCoeff = cy * cp * sr;
    zCoeff = cy * cp * cr;
    double dF0dp = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                   (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    xCoeff = -sy * cp;
    yCoeff = -sy * sp * sr - cy * cr;
    zCoeff = -sy * sp * cr + cy * sr;
    double dF0dy = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                   (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    yCoeff = sy * sp * cr - cy * sr;
    zCoeff = -sy * sp * sr - cy * cr;
    double dF1dr = (yCoeff * yVel + zCoeff * zVel) * delta +
                   (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    xCoeff = -sy * sp;
    yCoeff = sy * cp * sr;
    zCoeff = sy * cp * cr;
    double dF1dp = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                   (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    xCoeff = cy * cp;
    yCoeff = cy * sp * sr - sy * cr;
    zCoeff = cy * sp * cr + sy * sr;
    double dF1dy = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                   (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    yCoeff = cp * cr;
    zCoeff = -cp * sr;
    double dF2dr = (yCoeff * yVel + zCoeff * zVel) * delta +
                   (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    xCoeff = -cp;
    yCoeff = -sp * sr;
    zCoeff = -sp * cr;
    double dF2dp = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                   (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    // Much of the transfer function Jacobian is identical to the transfer function
    transferFunctionJacobian_ = transferFunction_;
    transferFunctionJacobian_(StateMemberX, StateMemberRoll) = dF0dr;
    transferFunctionJacobian_(StateMemberX, StateMemberPitch) = dF0dp;
    transferFunctionJacobian_(StateMemberX, StateMemberYaw) = dF0dy;
    transferFunctionJacobian_(StateMemberY, StateMemberRoll) = dF1dr;
    transferFunctionJacobian_(StateMemberY, StateMemberPitch) = dF1dp;
    transferFunctionJacobian_(StateMemberY, StateMemberYaw) = dF1dy;
    transferFunctionJacobian_(StateMemberZ, StateMemberRoll) = dF2dr;
    transferFunctionJacobian_(StateMemberZ, StateMemberPitch) = dF2dp;

    if (getDebug())
    {
      *debugStream_ << "Transfer function is:\n";
      *debugStream_ << transferFunction_ << "\n";
      *debugStream_ << "Transfer function Jacobian is:\n";
      *debugStream_ << transferFunctionJacobian_ << "\n";
      *debugStream_ << "Process noise covariance is " << "\n";
      *debugStream_ << processNoiseCovariance_ << "\n";
      *debugStream_ << "Current state is:\n";
      *debugStream_ << state_ << "\n";
    }

    // (1) Project the state forward: x = Ax (really, x = f(x))
    state_ = transferFunction_ * state_;

    // Handle wrapping
    wrapStateAngles();

    if (getDebug())
    {
      *debugStream_ << "Predicted state is:\n";
      *debugStream_ << state_ << "\n";
      *debugStream_ << "Current estimate error covariance is:\n";
      *debugStream_ << estimateErrorCovariance_ << "\n";
    }

    // (2) Project the error forward: P = J * P * J' + Q
    estimateErrorCovariance_ = (transferFunctionJacobian_ * estimateErrorCovariance_ * transferFunctionJacobian_.transpose())
      + (processNoiseCovariance_ * delta);

    if (getDebug())
    {
      *debugStream_ << "Predicted estimate error covariance is:\n";
      *debugStream_ << estimateErrorCovariance_ << "\n";
      *debugStream_ << "\n--------------------- /Ekf::predict ----------------------\n";
    }
  }

}


