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

#include <vector>
#include <ros/ros.h>

#include <gtest/gtest.h>

TEST(RLETest, StateBuffer)
{
  // Generate a few estimator states
  std::vector<RobotLocalization::EstimatorState> states;

  for ( int i = 0; i < 10; i++ )
  {
    /*
     * t = i s;
     * x = i m;
     * vx = 1.0 m/s;
     */
    RobotLocalization::EstimatorState state;
    state.time_stamp = i;
    state.state(RobotLocalization::StateMemberX) = i;
    state.state(RobotLocalization::StateMemberY) = 0;
    state.state(RobotLocalization::StateMemberZ) = 0;

    state.state(RobotLocalization::StateMemberRoll) = 0;
    state.state(RobotLocalization::StateMemberPitch) = 0;
    state.state(RobotLocalization::StateMemberYaw) = 0;

    state.state(RobotLocalization::StateMemberVx) = 1;
    state.state(RobotLocalization::StateMemberVy) = 0;
    state.state(RobotLocalization::StateMemberVz) = 0;

    state.state(RobotLocalization::StateMemberVroll) = 0;
    state.state(RobotLocalization::StateMemberVpitch) = 0;
    state.state(RobotLocalization::StateMemberVyaw) = 0;

    state.state(RobotLocalization::StateMemberAx) = 0;
    state.state(RobotLocalization::StateMemberAy) = 0;
    state.state(RobotLocalization::StateMemberAz) = 0;
    states.push_back(state);
  }

  // Instantiate a robot localization estimator with a buffer capacity of 5
  int buffer_capacity = 5;
  Eigen::MatrixXd process_noise_covariance = Eigen::MatrixXd::Identity(RobotLocalization::STATE_SIZE,
                                                                       RobotLocalization::STATE_SIZE);
  RobotLocalization::RobotLocalizationEstimator estimator(buffer_capacity, RobotLocalization::FilterTypes::EKF,
                                                          process_noise_covariance);

  RobotLocalization::EstimatorState state;

  // Add the states in chronological order
  for ( int i = 0; i < 6; i++ )
  {
    estimator.setState(states[i]);

    // Check that the state is added correctly
    estimator.getState(states[i].time_stamp, state);
    EXPECT_EQ(state.time_stamp, states[i].time_stamp);
  }

  // We filled the buffer with more states that it can hold, so its size should now be equal to the capacity
  EXPECT_EQ(static_cast<int>(estimator.getSize()), buffer_capacity);

  // Clear the buffer and check if it's really empty afterwards
  estimator.clearBuffer();
  EXPECT_EQ(estimator.getSize(), 0u);

  // Add states at time 1 through 3 inclusive to the buffer (buffer is not yet full)
  for ( int i = 1; i < 4; i++ )
  {
    estimator.setState(states[i]);
  }

  // Now add a state at time 0, but let's change it a bit (set StateMemberY=12) so that we can inspect if it is
  // correctly added to the buffer.
  RobotLocalization::EstimatorState state_2 = states[0];
  state_2.state(RobotLocalization::StateMemberY) = 12;
  estimator.setState(state_2);
  EXPECT_EQ(RobotLocalization::EstimatorResults::Exact,
            estimator.getState(states[0].time_stamp, state));

  // Check if the state is correctly added
  EXPECT_EQ(state.state, state_2.state);

  // Add some more states. State at t=0 should now be dropped, so we should get the prediction, which means y=0
  for ( int i = 5; i < 8; i++ )
  {
    estimator.setState(states[i]);
  }
  EXPECT_EQ(RobotLocalization::EstimatorResults::ExtrapolationIntoPast,
            estimator.getState(states[0].time_stamp, state));
  EXPECT_EQ(states[0].state, state.state);

  // Estimate a state that is not in the buffer, but can be determined by interpolation. The predicted state vector
  // should be equal to the designed state at the requested time.
  EXPECT_EQ(RobotLocalization::EstimatorResults::Interpolation,
            estimator.getState(states[4].time_stamp, state));
  EXPECT_EQ(states[4].state, state.state);

  // Estimate a state that is not in the buffer, but can be determined by extrapolation into the future. The predicted
  // state vector should be equal to the designed state at the requested time.
  EXPECT_EQ(RobotLocalization::EstimatorResults::ExtrapolationIntoFuture,
            estimator.getState(states[8].time_stamp, state));
  EXPECT_EQ(states[8].state, state.state);

  // Add missing state somewhere in the middle
  estimator.setState(states[4]);

  // Overwrite state at t=3 (oldest state now in the buffer) and check if it's correctly overwritten.
  state_2 = states[3];
  state_2.state(RobotLocalization::StateMemberVy) = -1.0;
  estimator.setState(state_2);
  EXPECT_EQ(RobotLocalization::EstimatorResults::Exact,
            estimator.getState(states[3].time_stamp, state));
  EXPECT_EQ(state_2.state, state.state);

  // Add state that came too late
  estimator.setState(states[0]);

  // Check if getState needed to do extrapolation into the past
  EXPECT_EQ(estimator.getState(states[0].time_stamp, state),
      RobotLocalization::EstimatorResults::ExtrapolationIntoPast);

  // Check state at t=0. This can only work correctly if the state at t=3 is
  // overwritten and the state at zero is not in the buffer.
  EXPECT_DOUBLE_EQ(3.0, state.state(RobotLocalization::StateMemberY));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_robot_localization_estimator");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
