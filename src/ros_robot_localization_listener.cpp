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

#include <functional>
#include <rclcpp/qos.hpp>

#include <Eigen/Dense>
//#include <eigen_conversions/eigen_msg.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "robot_localization/ros_robot_localization_listener.hpp"

namespace robot_localization
{
  RosRobotLocalizationListener::RosRobotLocalizationListener(
    rclcpp::Node::SharedPtr node, const std::string& ns):
    estimator_(0u),
    qos1_(1),
    qos10_(10),
    odom_sub_(node, "~" + ns + "/odom", qos1_.get_rmw_qos_profile()),
    accel_sub_(node, "~" + ns + "/acceleration", qos1_.get_rmw_qos_profile()),
    sync_(odom_sub_, accel_sub_, 10u)
  {
    int buffer_size = node->declare_parameter<int>("~/buffer_size", 10);
    estimator_.setBufferCapacity(buffer_size);

    sync_.registerCallback(std::bind(
      &robot_localization::RosRobotLocalizationListener::odomAndAccelCallback,
      this, std::placeholders::_1, std::placeholders::_2));
  }

  RosRobotLocalizationListener::~RosRobotLocalizationListener()
  {
  }

  void RosRobotLocalizationListener::odomAndAccelCallback(
    const std::shared_ptr<nav_msgs::msg::Odometry const>& odom,
    const std::shared_ptr<geometry_msgs::msg::AccelWithCovarianceStamped const>& accel)
  {
    // Instantiate a state that can be added to the robot localization estimator
    EstimatorState state;
    // Set its time stamp and the state received from the messages
    state.time_stamp = odom->header.stamp.sec;

    // Pose: Position
    state.state(StateMemberX) = odom->pose.pose.position.x;
    state.state(StateMemberY) = odom->pose.pose.position.y;
    state.state(StateMemberZ) = odom->pose.pose.position.z;

    // Pose: Orientation
    tf2::Quaternion orientation_quat;
    tf2::fromMsg(odom->pose.pose.orientation, orientation_quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(orientation_quat).getRPY(roll, pitch, yaw);

    state.state(StateMemberRoll) = roll;
    state.state(StateMemberPitch) = pitch;
    state.state(StateMemberYaw) = yaw;

    // Pose: Covariance
    for ( unsigned int i = 0; i < POSE_SIZE; i++ )
    {
      for ( unsigned int j = 0; j < POSE_SIZE; j++ )
      {
        state.covariance(POSITION_OFFSET + i, POSITION_OFFSET + j) = odom->pose.covariance[i*POSE_SIZE + j];
      }
    }

    // Velocity: Linear
    state.state(StateMemberVx) = odom->twist.twist.linear.x;
    state.state(StateMemberVy) = odom->twist.twist.linear.y;
    state.state(StateMemberVz) = odom->twist.twist.linear.z;

    // Velocity: Angular
    state.state(StateMemberVroll) = odom->twist.twist.angular.x;
    state.state(StateMemberVpitch) = odom->twist.twist.angular.y;
    state.state(StateMemberVyaw) = odom->twist.twist.angular.z;

    // Velocity: Covariance
    for ( unsigned int i = 0; i < TWIST_SIZE; i++ )
    {
      for ( unsigned int j = 0; j < TWIST_SIZE; j++ )
      {
        state.covariance(POSITION_V_OFFSET + i, POSITION_V_OFFSET + j) = odom->twist.covariance[i*TWIST_SIZE + j];
      }
    }

    // Acceleration: Linear
    state.state(StateMemberAx) = accel->accel.accel.linear.x;
    state.state(StateMemberAy) = accel->accel.accel.linear.y;
    state.state(StateMemberAz) = accel->accel.accel.linear.z;

    // Acceleration: Angular is not available in state

    // Acceleration: Covariance
    for ( unsigned int i = 0; i < ACCELERATION_SIZE; i++ )
    {
      for ( unsigned int j = 0; j < ACCELERATION_SIZE; j++ )
      {
        state.covariance(POSITION_A_OFFSET + i, POSITION_A_OFFSET + j) = accel->accel.covariance[i*TWIST_SIZE + j];
      }
    }

    // Add the state to the buffer, so that we can later interpolate between this and earlier states
    estimator_.setState(state);

    return;
  }

  bool RosRobotLocalizationListener::getState(const double time, Eigen::VectorXd& state, Eigen::MatrixXd& covariance)
  {
    EstimatorState estimator_state;
    if (!estimator_.getState(time,estimator_state))
      return false;
    state = estimator_state.state;
    covariance = estimator_state.covariance;

    return true;
  }

  bool RosRobotLocalizationListener::getState(const rclcpp::Time& rclcpp_time, Eigen::VectorXd& state, Eigen::MatrixXd& covariance)
  {
    double time = rclcpp_time.nanoseconds() / 100000000;
    return getState(time, state, covariance);
  }
}  // namespace robot_localization

