/*
 * Copyright (c) 2015, Charles River Analytics, Inc.
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

#include "robot_localization/ros_filter_utilities.h"
#include "robot_localization/filter_common.h"

std::ostream& operator<<(std::ostream& os, const tf::Vector3 &vec)
{
  os << "(" << std::setprecision(20) << vec.getX() << " " << vec.getY() << " " << vec.getZ() << ")\n";

  return os;
}

std::ostream& operator<<(std::ostream& os, const tf::Quaternion &quat)
{
  double roll, pitch, yaw;
  tf::Matrix3x3 orTmp(quat);
  orTmp.getRPY(roll, pitch, yaw);

  os << "(" << std::setprecision(20) << roll << ", " << pitch << ", " << yaw << ")\n";

  return os;
}

std::ostream& operator<<(std::ostream& os, const tf::Transform &trans)
{
  os << "Origin: " << trans.getOrigin() <<
        "Rotation (RPY): " << trans.getRotation();

  return os;
}

namespace RobotLocalization
{
  namespace RosFilterUtilities
  {
    void quatToRPY(const tf::Quaternion &quat, double &roll, double &pitch, double &yaw)
    {
      tf::Matrix3x3 orTmp(quat);
      orTmp.getRPY(roll, pitch, yaw);
    }

    void stateToTF(const Eigen::VectorXd &state, tf::Transform &stateTF)
    {
      stateTF.setOrigin(tf::Vector3(state(StateMemberX),
                                    state(StateMemberY),
                                    state(StateMemberZ)));
      tf::Quaternion quat;
      quat.setRPY(state(StateMemberRoll),
                  state(StateMemberPitch),
                  state(StateMemberYaw));

      stateTF.setRotation(quat);
    }

    void TFtoState(const tf::Transform &stateTF, Eigen::VectorXd &state)
    {
      state(StateMemberX) = stateTF.getOrigin().getX();
      state(StateMemberY) = stateTF.getOrigin().getY();
      state(StateMemberZ) = stateTF.getOrigin().getZ();
      quatToRPY(stateTF.getRotation(), state(StateMemberRoll), state(StateMemberPitch), state(StateMemberYaw));
    }
  }
}
