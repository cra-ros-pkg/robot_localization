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

#include <robot_localization/ros_robot_localization_listener.hpp>

namespace robot_localization
{
  RosRobotLocalizationListener::RosRobotLocalizationListener(
    rclcpp::Node::SharedPtr node):
    estimator_(0u),
    qos1_(1),
    qos10_(10),
    odom_sub_(node, "~/odom", qos1_.get_rmw_qos_profile()),
    accel_sub_(node, "~/acceleration", qos1_.get_rmw_qos_profile()),
    sync_(odom_sub_, accel_sub_, 10u)
  {
    int buffer_size = 0;
    node->declare_parameter<int>("~/buffer_size", buffer_size);
    node->get_parameter_or<int>("~/buffer_size", buffer_size, buffer_size);

    estimator_.setBufferCapacity(buffer_size);

    sync_.registerCallback(std::bind(
      &robot_localization::RosRobotLocalizationListener::odomAndAccelCallback,
      this, std::placeholders::_1, std::placeholders::_2));
  }

  RobotLocalizationEstimator::~RobotLocalizationEstimator()
  {
  }

  void RosRobotLocalizationListener::odomAndAccelCallback(
    const std::shared_ptr<nav_msgs::msg::Odometry const>& odom,
    const std::shared_ptr<geometry_msgs::msg::AccelWithCovarianceStamped const>& accel)
  {
    return;
  }


}  // namespace robot_localization

