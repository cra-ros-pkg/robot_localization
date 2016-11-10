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

#include "robot_localization/robot_localization_estimator.h"

namespace RobotLocalization
{
RobotLocalizationEstimator::RobotLocalizationEstimator(unsigned int buffer_capacity)
{
  state_buffer_.set_capacity(buffer_capacity);
}

RobotLocalizationEstimator::~RobotLocalizationEstimator()
{
}

void RobotLocalizationEstimator::setState(const EstimatorState& state)
{
  // If newly received state is newer than any in the buffer, push back
  if ( state_buffer_.empty() || state.time_stamp > state_buffer_.back().time_stamp )
  {
    state_buffer_.push_back(state);
  }
  // If it is older, put it in the right position
  else
  {
    for ( boost::circular_buffer<EstimatorState>::iterator it = state_buffer_.begin(); it != state_buffer_.end(); ++it )
    {
      if ( state.time_stamp <= it->time_stamp )
      {
        state_buffer_.insert(it, state);
        return;
      }
    }
  }
}

int RobotLocalizationEstimator::getState(EstimatorState& state) const
{
  // If there's nothing in the buffer, there's nothing to give.
  if ( state_buffer_.size() == 0 )
  {
    return -1;
  }

  state = state_buffer_.back();

  return 0;
}

int RobotLocalizationEstimator::getState(const double time, EstimatorState& state) const
{
  // If there's nothing in the buffer, there's nothing to give.
  if ( state_buffer_.size() == 0 )
  {
    return -1;
  }

  // Set state to the most recent one for now
  state = state_buffer_.back();

  // Go through buffer from new to old
  EstimatorState last_state_before_time = state_buffer_.front();
  EstimatorState next_state_after_time = state_buffer_.back();
  bool previous_state_found = false;
  bool next_state_found = false;

  for (boost::circular_buffer<EstimatorState>::const_reverse_iterator it = state_buffer_.rbegin(); it != state_buffer_.rend(); ++it)
  {
    /* If the time stamp of the current state from the buffer is
       * older than the requested time, store it as the last state
       * before the requested time. If it is younger, save it as the
       * next one after, and go on to find the last one before.
       */
    if ( it->time_stamp <= time )
    {
      last_state_before_time = *it;
      previous_state_found = true;
      break;
    }
    else
    {
      next_state_after_time = *it;
      next_state_found = true;
    }
  }

  // If we found a previous state and a next state, we can do interpolation
  if ( previous_state_found && next_state_found )
  {
    interpolate(last_state_before_time, next_state_after_time, time, state);
  }

  // If only a previous state is found, we can do extrapolation into the future
  else if ( previous_state_found )
  {
    extrapolate(last_state_before_time, time, state);
  }

  // If only a next state is found, we'll have to extrapolate into the past.
  else if ( next_state_found )
  {
    // Warn the user that the buffer may be too small!
    extrapolate(next_state_after_time, time, state);
  }

  return 0;
}

void RobotLocalizationEstimator::setBufferCapacity(const int capacity)
{
  state_buffer_.set_capacity(capacity);
}

void RobotLocalizationEstimator::clearBuffer()
{
  state_buffer_.clear();
}

unsigned int RobotLocalizationEstimator::capacity() const
{
  return state_buffer_.capacity();
}

unsigned int RobotLocalizationEstimator::size() const
{
  return state_buffer_.size();
}

void RobotLocalizationEstimator::extrapolate(const EstimatorState& boundary_state,
                                             const double requested_time,
                                             EstimatorState& state_at_req_time) const
{
  state_at_req_time = boundary_state;
  state_at_req_time.time_stamp = requested_time;

  // Calculate how much time we need to extrapolate into the future
  double delta = requested_time - boundary_state.time_stamp;

  // Get the state variables from the boundary state
  double roll = boundary_state.state(StateMemberRoll);
  double pitch = boundary_state.state(StateMemberPitch);
  double yaw = boundary_state.state(StateMemberYaw);
  double xVel = boundary_state.state(StateMemberVx);
  double yVel = boundary_state.state(StateMemberVy);
  double zVel = boundary_state.state(StateMemberVz);
  double rollVel = boundary_state.state(StateMemberVroll);
  double pitchVel = boundary_state.state(StateMemberVpitch);
  double yawVel = boundary_state.state(StateMemberVyaw);
  double xAcc = boundary_state.state(StateMemberAx);
  double yAcc = boundary_state.state(StateMemberAy);
  double zAcc = boundary_state.state(StateMemberAz);

  // We'll need these trig calculations a lot.
  double sp = 0.0;
  double cp = 0.0;
  ::sincos(pitch, &sp, &cp);

  double sr = 0.0;
  double cr = 0.0;
  ::sincos(roll, &sr, &cr);

  double sy = 0.0;
  double cy = 0.0;
  ::sincos(yaw, &sy, &cy);

  Eigen::MatrixXd transfer_function(STATE_SIZE, STATE_SIZE);
  transfer_function.setZero();

  // Prepare the transfer function
  transfer_function(StateMemberX, StateMemberVx) = cy * cp * delta;
  transfer_function(StateMemberX, StateMemberVy) = (cy * sp * sr - sy * cr) * delta;
  transfer_function(StateMemberX, StateMemberVz) = (cy * sp * cr + sy * sr) * delta;
  transfer_function(StateMemberX, StateMemberAx) = 0.5 * transfer_function(StateMemberX, StateMemberVx) * delta;
  transfer_function(StateMemberX, StateMemberAy) = 0.5 * transfer_function(StateMemberX, StateMemberVy) * delta;
  transfer_function(StateMemberX, StateMemberAz) = 0.5 * transfer_function(StateMemberX, StateMemberVz) * delta;
  transfer_function(StateMemberY, StateMemberVx) = sy * cp * delta;
  transfer_function(StateMemberY, StateMemberVy) = (sy * sp * sr + cy * cr) * delta;
  transfer_function(StateMemberY, StateMemberVz) = (sy * sp * cr - cy * sr) * delta;
  transfer_function(StateMemberY, StateMemberAx) = 0.5 * transfer_function(StateMemberY, StateMemberVx) * delta;
  transfer_function(StateMemberY, StateMemberAy) = 0.5 * transfer_function(StateMemberY, StateMemberVy) * delta;
  transfer_function(StateMemberY, StateMemberAz) = 0.5 * transfer_function(StateMemberY, StateMemberVz) * delta;
  transfer_function(StateMemberZ, StateMemberVx) = -sp * delta;
  transfer_function(StateMemberZ, StateMemberVy) = cp * sr * delta;
  transfer_function(StateMemberZ, StateMemberVz) = cp * cr * delta;
  transfer_function(StateMemberZ, StateMemberAx) = 0.5 * transfer_function(StateMemberZ, StateMemberVx) * delta;
  transfer_function(StateMemberZ, StateMemberAy) = 0.5 * transfer_function(StateMemberZ, StateMemberVy) * delta;
  transfer_function(StateMemberZ, StateMemberAz) = 0.5 * transfer_function(StateMemberZ, StateMemberVz) * delta;
  transfer_function(StateMemberRoll, StateMemberVroll) = transfer_function(StateMemberX, StateMemberVx);
  transfer_function(StateMemberRoll, StateMemberVpitch) = transfer_function(StateMemberX, StateMemberVy);
  transfer_function(StateMemberRoll, StateMemberVyaw) = transfer_function(StateMemberX, StateMemberVz);
  transfer_function(StateMemberPitch, StateMemberVroll) = transfer_function(StateMemberY, StateMemberVx);
  transfer_function(StateMemberPitch, StateMemberVpitch) = transfer_function(StateMemberY, StateMemberVy);
  transfer_function(StateMemberPitch, StateMemberVyaw) = transfer_function(StateMemberY, StateMemberVz);
  transfer_function(StateMemberYaw, StateMemberVroll) = transfer_function(StateMemberZ, StateMemberVx);
  transfer_function(StateMemberYaw, StateMemberVpitch) = transfer_function(StateMemberZ, StateMemberVy);
  transfer_function(StateMemberYaw, StateMemberVyaw) = transfer_function(StateMemberZ, StateMemberVz);
  transfer_function(StateMemberVx, StateMemberAx) = delta;
  transfer_function(StateMemberVy, StateMemberAy) = delta;
  transfer_function(StateMemberVz, StateMemberAz) = delta;

  // Prepare the transfer function Jacobian. This function is analytically derived from the
  // transfer function.
  double xCoeff = 0.0;
  double yCoeff = 0.0;
  double zCoeff = 0.0;
  double oneHalfATSquared = 0.5 * delta * delta;

  yCoeff = cy * sp * cr + sy * sr;
  zCoeff = -cy * sp * sr + sy * cr;
  double dFx_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                  (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
  double dFR_dR = 1 + (yCoeff * pitchVel + zCoeff * yawVel) * delta;

  xCoeff = -cy * sp;
  yCoeff = cy * cp * sr;
  zCoeff = cy * cp * cr;
  double dFx_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                  (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
  double dFR_dP = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

  xCoeff = -sy * cp;
  yCoeff = -sy * sp * sr - cy * cr;
  zCoeff = -sy * sp * cr + cy * sr;
  double dFx_dY = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                  (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
  double dFR_dY = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

  yCoeff = sy * sp * cr - cy * sr;
  zCoeff = -sy * sp * sr - cy * cr;
  double dFy_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                  (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
  double dFP_dR = (yCoeff * pitchVel + zCoeff * yawVel) * delta;

  xCoeff = -sy * sp;
  yCoeff = sy * cp * sr;
  zCoeff = sy * cp * cr;
  double dFy_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                  (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
  double dFP_dP = 1 + (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

  xCoeff = cy * cp;
  yCoeff = cy * sp * sr - sy * cr;
  zCoeff = cy * sp * cr + sy * sr;
  double dFy_dY = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                  (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
  double dFP_dY = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

  yCoeff = cp * cr;
  zCoeff = -cp * sr;
  double dFz_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                  (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
  double dFY_dR = (yCoeff * pitchVel + zCoeff * yawVel) * delta;

  xCoeff = -cp;
  yCoeff = -sp * sr;
  zCoeff = -sp * cr;
  double dFz_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                  (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
  double dFY_dP = (xCoeff * rollVel + yCoeff * pitchVel + zCoeff * yawVel) * delta;

  // Much of the transfer function Jacobian is identical to the transfer function
  Eigen::MatrixXd transferFunctionJacobian = transfer_function;
  transferFunctionJacobian(StateMemberX, StateMemberRoll) = dFx_dR;
  transferFunctionJacobian(StateMemberX, StateMemberPitch) = dFx_dP;
  transferFunctionJacobian(StateMemberX, StateMemberYaw) = dFx_dY;
  transferFunctionJacobian(StateMemberY, StateMemberRoll) = dFy_dR;
  transferFunctionJacobian(StateMemberY, StateMemberPitch) = dFy_dP;
  transferFunctionJacobian(StateMemberY, StateMemberYaw) = dFy_dY;
  transferFunctionJacobian(StateMemberZ, StateMemberRoll) = dFz_dR;
  transferFunctionJacobian(StateMemberZ, StateMemberPitch) = dFz_dP;
  transferFunctionJacobian(StateMemberRoll, StateMemberRoll) = dFR_dR;
  transferFunctionJacobian(StateMemberRoll, StateMemberPitch) = dFR_dP;
  transferFunctionJacobian(StateMemberRoll, StateMemberYaw) = dFR_dY;
  transferFunctionJacobian(StateMemberPitch, StateMemberRoll) = dFP_dR;
  transferFunctionJacobian(StateMemberPitch, StateMemberPitch) = dFP_dP;
  transferFunctionJacobian(StateMemberPitch, StateMemberYaw) = dFP_dY;
  transferFunctionJacobian(StateMemberYaw, StateMemberRoll) = dFY_dR;
  transferFunctionJacobian(StateMemberYaw, StateMemberPitch) = dFY_dP;

  // Project the state forward: x = Ax (really, x = f(x))
  state_at_req_time.state = transfer_function * state_at_req_time.state;

  // Handle wrapping
  wrapStateAngles(state_at_req_time);

  // Project the error forward: P = J * P * J' + Q
  state_at_req_time.covariance = (transferFunctionJacobian * boundary_state.covariance *
                                  transferFunctionJacobian.transpose());
  state_at_req_time.covariance.noalias() += (processNoiseCovariance_ * delta);

  return;
}

void RobotLocalizationEstimator::interpolate(const EstimatorState& given_state_1,
                                             const EstimatorState& given_state_2,
                                             const double requested_time,
                                             EstimatorState& state_at_req_time) const
{
  /*
   * TODO: Right now, we only extrapolate from the last known state before the requested time.
   * But as the state after the requested time is also known, we may want to perform
   * interpolation between states.
   */
  extrapolate(given_state_1, requested_time, state_at_req_time);
  return;
}

}  // namespace RobotLocalization
