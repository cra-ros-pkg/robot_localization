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
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_robot_localization_estimator");

  // Generate a few empty estimator states
  std::vector<RobotLocalization::EstimatorState> states;

  for ( int i = 0; i < 10; i++ )
  {
    RobotLocalization::EstimatorState state;
    state.time_stamp = i;
    states.push_back(state);
  }

  // Instantiate a robot localization estimator with a buffer size of 5
  RobotLocalization::RobotLocalizationEstimator estimator(5);

  // Add the states in order from old to new
  for ( int i = 0; i < 10; i++ )
  {
    estimator.setState(states[i]);
    std::cout << "setting state " << states[i] << "\n" << std::endl;

    // inspect estimator
    std::cout << "estimator content: " << estimator << std::endl;
    std::cout << " ------------------------\n " << std::endl;
  }

  // Clear the estimator buffer
  estimator.clearBuffer();

  std::cout << "---------------------------\n\nCLEARING BUFFER!!!\n\n" << std::endl;

  // inspect estimator
  std::cout << "estimator content: " << estimator << std::endl;
  std::cout << " ------------------------\n " << std::endl;

  // Set states in another order than linearly
  for ( int i = 1; i < 4; i++ )
  {
    estimator.setState(states[i]);
    std::cout << "setting state " << states[i] << "\n" << std::endl;

    // inspect estimator
    std::cout << "estimator content: " << estimator << std::endl;
    std::cout << " ------------------------\n " << std::endl;
  }

  // Add a state that came late, but there's space in the buffer
  estimator.setState(states[0]);
  std::cout << "setting state " << states[0] << "\n" << std::endl;

  // inspect estimator
  std::cout << "estimator content: " << estimator << std::endl;
  std::cout << " ------------------------\n " << std::endl;

  // Add some more states
  for ( int i = 5; i < 8; i++ )
  {
    estimator.setState(states[i]);
    std::cout << "setting state " << states[i] << "\n" << std::endl;

    // inspect estimator
    std::cout << "estimator content: " << estimator << std::endl;
    std::cout << " ------------------------\n " << std::endl;
  }

  // Add state somewhere in the middle
  estimator.setState(states[4]);
  std::cout << "setting state " << states[4] << "\n" << std::endl;

  // inspect estimator
  std::cout << "estimator content: " << estimator << std::endl;
  std::cout << " ------------------------\n " << std::endl;

  // Add state that came too late
  estimator.setState(states[0]);
  std::cout << "setting state " << states[0] << "\n" << std::endl;

  // inspect estimator
  std::cout << "estimator content: " << estimator << std::endl;
  std::cout << " ------------------------\n " << std::endl;

  for ( int i = 0; i < 10; i+=3 )
  {
    RobotLocalization::EstimatorState state;
    estimator.getState(i, state);
    std::cout << "state at " << i << ":\n" << state << std::endl;
  }


  return 0;
}
