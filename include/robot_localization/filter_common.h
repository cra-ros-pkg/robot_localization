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

#ifndef ROBOT_LOCALIZATION_FILTER_COMMON_H
#define ROBOT_LOCALIZATION_FILTER_COMMON_H

namespace RobotLocalization
{

//! @brief Enumeration that defines the state vector
//!
enum StateMembers
{
  StateMemberX = 0,
  StateMemberY,
  StateMemberZ,
  StateMemberRoll,
  StateMemberPitch,
  StateMemberYaw,
  StateMemberVx,
  StateMemberVy,
  StateMemberVz,
  StateMemberVroll,
  StateMemberVpitch,
  StateMemberVyaw,
  StateMemberAx,
  StateMemberAy,
  StateMemberAz
};

//! @brief Enumeration that defines the control vector
//!
enum ControlMembers
{
  ControlMemberVx,
  ControlMemberVy,
  ControlMemberVz,
  ControlMemberVroll,
  ControlMemberVpitch,
  ControlMemberVyaw
};

//! @brief Global constants that define our state
//! vector size and offsets to groups of values
//! within that state.
const int STATE_SIZE = 15;
const int POSITION_OFFSET = StateMemberX;
const int ORIENTATION_OFFSET = StateMemberRoll;
const int POSITION_V_OFFSET = StateMemberVx;
const int ORIENTATION_V_OFFSET = StateMemberVroll;
const int POSITION_A_OFFSET = StateMemberAx;

//! @brief Pose and twist messages each
//! contain six variables
const int POSE_SIZE = 6;
const int TWIST_SIZE = 6;
const int POSITION_SIZE = 3;
const int ORIENTATION_SIZE = 3;
const int ACCELERATION_SIZE = 3;

//! @brief Common variables
const double PI = 3.141592653589793;
const double TAU = 6.283185307179587;

}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_FILTER_COMMON_H
