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

#include "robot_localization/ros_filter_utilities.h"
#include "robot_localization/filter_common.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/console.h>

#include <string>
#include <vector>

std::ostream& operator<<(std::ostream& os, const tf2::Vector3 &vec)
{
  os << "(" << std::setprecision(20) << vec.getX() << " " << vec.getY() << " " << vec.getZ() << ")\n";

  return os;
}

std::ostream& operator<<(std::ostream& os, const tf2::Quaternion &quat)
{
  double roll, pitch, yaw;
  tf2::Matrix3x3 orTmp(quat);
  orTmp.getRPY(roll, pitch, yaw);

  os << "(" << std::setprecision(20) << roll << ", " << pitch << ", " << yaw << ")\n";

  return os;
}

std::ostream& operator<<(std::ostream& os, const tf2::Transform &trans)
{
  os << "Origin: " << trans.getOrigin() <<
        "Rotation (RPY): " << trans.getRotation();

  return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<double> &vec)
{
  os << "(" << std::setprecision(20);

  for (size_t i = 0; i < vec.size(); ++i)
  {
    os << vec[i] << " ";
  }

  os << ")\n";

  return os;
}

namespace RobotLocalization
{
namespace RosFilterUtilities
{

  double getYaw(const tf2::Quaternion quat)
  {
    tf2::Matrix3x3 mat(quat);

    double dummy;
    double yaw;
    mat.getRPY(dummy, dummy, yaw);

    return yaw;
  }

  bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
                           const std::string &targetFrame,
                           const std::string &sourceFrame,
                           const ros::Time &time,
                           const ros::Duration &timeout,
                           tf2::Transform &targetFrameTrans,
                           const bool silent)
  {
    bool retVal = true;

    // First try to transform the data at the requested time
    try
    {
      tf2::fromMsg(buffer.lookupTransform(targetFrame, sourceFrame, time, timeout).transform,
                   targetFrameTrans);
    }
    catch (tf2::TransformException &ex)
    {
      // The issue might be that the transforms that are available are not close
      // enough temporally to be used. In that case, just use the latest available
      // transform and warn the user.
      try
      {
        tf2::fromMsg(buffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0)).transform,
                     targetFrameTrans);

        if (!silent)
        {
          ROS_WARN_STREAM_THROTTLE(2.0, "Transform from " << sourceFrame << " to " << targetFrame <<
                                        " was unavailable for the time requested. Using latest instead.\n");
        }
      }
      catch(tf2::TransformException &ex)
      {
        if (!silent)
        {
          ROS_WARN_STREAM_THROTTLE(2.0, "Could not obtain transform from " << sourceFrame <<
                                        " to " << targetFrame << ". Error was " << ex.what() << "\n");
        }

        retVal = false;
      }
    }

    // Transforming from a frame id to itself can fail when the tf tree isn't
    // being broadcast (e.g., for some bag files). This is the only failure that
    // would throw an exception, so check for this situation before giving up.
    if (!retVal)
    {
      if (targetFrame == sourceFrame)
      {
        targetFrameTrans.setIdentity();
        retVal = true;
      }
    }

    return retVal;
  }

  bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
                           const std::string &targetFrame,
                           const std::string &sourceFrame,
                           const ros::Time &time,
                           tf2::Transform &targetFrameTrans,
                           const bool silent)
  {
    return lookupTransformSafe(buffer, targetFrame, sourceFrame, time, ros::Duration(0), targetFrameTrans, silent);
  }

  void quatToRPY(const tf2::Quaternion &quat, double &roll, double &pitch, double &yaw)
  {
    tf2::Matrix3x3 orTmp(quat);
    orTmp.getRPY(roll, pitch, yaw);
  }

  void stateToTF(const Eigen::VectorXd &state, tf2::Transform &stateTF)
  {
    stateTF.setOrigin(tf2::Vector3(state(StateMemberX),
                                   state(StateMemberY),
                                   state(StateMemberZ)));
    tf2::Quaternion quat;
    quat.setRPY(state(StateMemberRoll),
                state(StateMemberPitch),
                state(StateMemberYaw));

    stateTF.setRotation(quat);
  }

  void TFtoState(const tf2::Transform &stateTF, Eigen::VectorXd &state)
  {
    state(StateMemberX) = stateTF.getOrigin().getX();
    state(StateMemberY) = stateTF.getOrigin().getY();
    state(StateMemberZ) = stateTF.getOrigin().getZ();
    quatToRPY(stateTF.getRotation(), state(StateMemberRoll), state(StateMemberPitch), state(StateMemberYaw));
  }

}  // namespace RosFilterUtilities
}  // namespace RobotLocalization
