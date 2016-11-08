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
  RobotLocalizationEstimator::RobotLocalizationEstimator()
  {
  }

  RobotLocalizationEstimator::~RobotLocalizationEstimator()
  {
  }

  void RobotLocalizationEstimator::setState(const EstimatorState& state)
  {
    state_buffer_.push_back(state);
  }

  int RobotLocalizationEstimator::getState(EstimatorState& state)
  {
    // If there's nothing in the buffer, there's nothing to give.
    if ( state_buffer_.size() == 0 )
    {
      return -1;
    }

    state = state_buffer_.back();

    return 0;
  }

  int RobotLocalizationEstimator::getState(const double time, EstimatorState& state)
  {
    // If there's nothing in the buffer, there's nothing to give.
    if ( state_buffer_.size() == 0 )
    {
      return -1;
    }

    // Set state to the most recent one for now
    state = state_buffer_.back();


    // Go through buffer and get last saved state before the given time
    EstimatorState last_state_before_time = state_buffer_.front();
    bool last_state_found = false;

    for ( boost::circular_buffer<EstimatorState>::const_iterator it = state_buffer_.begin(); it != state_buffer_.end(); ++it )
    {
      if ( it->time_stamp <= time )
      {
        last_state_before_time = *it;
        last_state_found = true;
      }
    }

    // If we found a last state, we know about a state before the queried time
    if ( last_state_found )
    {
      // If, additionally, it is NOT the most recent state we know, we can do interpolation
      if ( last_state_before_time.time_stamp != state_buffer_.back().time_stamp )
      {
        state = last_state_before_time;
      }

      // If it IS the most recent one, we'll need to extrapolate into the future
      else
      {
        state = last_state_before_time;
      }
    }

    // If we only know about states after the queried time, we'll have to extrapolate into the past.
    else
    {
      // Warn the user that the buffer may be too small!
      state = last_state_before_time;
    }

    return 0;

  }

}  // namespace RobotLocalization
