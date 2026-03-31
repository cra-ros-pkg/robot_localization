/*
 * Copyright (c) 2016, TNO IVS, Helmond
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

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_localization/ros_robot_localization_listener.hpp"
#include "robot_localization/filter_common.hpp"

std::shared_ptr<rclcpp::Node> node;
std::unique_ptr<robot_localization::RosRobotLocalizationListener> g_listener;

TEST(LocalizationListenerTest, testGetStateOfBaseLink)
{
  rclcpp::spin_some(node);

  rclcpp::Time time2(1001, 0);

  Eigen::VectorXd state(robot_localization::STATE_SIZE);
  Eigen::MatrixXd covariance(robot_localization::STATE_SIZE, robot_localization::STATE_SIZE);


  std::string base_frame("base_link");
  g_listener->getState(time2, base_frame, state, covariance);

  EXPECT_DOUBLE_EQ(1.0, state(robot_localization::StateMemberX));
  EXPECT_DOUBLE_EQ(0.0, state(robot_localization::StateMemberY));
  EXPECT_DOUBLE_EQ(0.0, state(robot_localization::StateMemberZ));

  EXPECT_FLOAT_EQ(M_PI / 4, state(robot_localization::StateMemberRoll));
  EXPECT_FLOAT_EQ(0.0, state(robot_localization::StateMemberPitch));
  EXPECT_FLOAT_EQ(0.0, state(robot_localization::StateMemberYaw));

  EXPECT_DOUBLE_EQ(M_PI / 4.0, state(robot_localization::StateMemberVroll));
  EXPECT_DOUBLE_EQ(0.0, state(robot_localization::StateMemberVpitch));
  EXPECT_DOUBLE_EQ(0.0, state(robot_localization::StateMemberVyaw));
}

TEST(LocalizationListenerTest, GetStateOfRelatedFrame)
{
  rclcpp::spin_some(node);

  Eigen::VectorXd state(robot_localization::STATE_SIZE);
  Eigen::MatrixXd covariance(robot_localization::STATE_SIZE, robot_localization::STATE_SIZE);

  rclcpp::Time time1(1000, 0);
  rclcpp::Time time2(1001, 0);

  std::string sensor_frame("sensor");

  EXPECT_TRUE(g_listener->getState(time1, sensor_frame, state, covariance) );

  EXPECT_FLOAT_EQ(0.0, state(robot_localization::StateMemberX));
  EXPECT_FLOAT_EQ(1.0, state(robot_localization::StateMemberY));
  EXPECT_FLOAT_EQ(0.0, state(robot_localization::StateMemberZ));

  EXPECT_FLOAT_EQ(0.0, state(robot_localization::StateMemberRoll));
  EXPECT_FLOAT_EQ(0.0, state(robot_localization::StateMemberPitch));
  EXPECT_FLOAT_EQ(M_PI / 2, state(robot_localization::StateMemberYaw));

  EXPECT_TRUE(1e-12 > state(robot_localization::StateMemberVx));
  EXPECT_FLOAT_EQ(-1.0, state(robot_localization::StateMemberVy));
  EXPECT_FLOAT_EQ(M_PI / 4.0, state(robot_localization::StateMemberVz));

  EXPECT_TRUE(1e-12 > state(robot_localization::StateMemberVroll));
  EXPECT_FLOAT_EQ(-M_PI / 4.0, state(robot_localization::StateMemberVpitch));
  EXPECT_FLOAT_EQ(0.0, state(robot_localization::StateMemberVyaw));

  EXPECT_TRUE(g_listener->getState(time2, sensor_frame, state, covariance));

  EXPECT_FLOAT_EQ(1.0, state(robot_localization::StateMemberX));
  EXPECT_FLOAT_EQ(sqrt(2) / 2.0, state(robot_localization::StateMemberY));
  EXPECT_FLOAT_EQ(sqrt(2) / 2.0, state(robot_localization::StateMemberZ));

  EXPECT_TRUE(1e-12 > state(robot_localization::StateMemberRoll));
  EXPECT_TRUE(1e-12 > fabs(-M_PI / 4.0 - state(robot_localization::StateMemberPitch)));
  EXPECT_FLOAT_EQ(M_PI / 2, state(robot_localization::StateMemberYaw));

  EXPECT_TRUE(1e-12 > state(robot_localization::StateMemberVx));
  EXPECT_FLOAT_EQ(-1.0, state(robot_localization::StateMemberVy));
  EXPECT_FLOAT_EQ(M_PI / 4, state(robot_localization::StateMemberVz));

  EXPECT_TRUE(1e-12 > state(robot_localization::StateMemberVroll));
  EXPECT_FLOAT_EQ(-M_PI / 4.0, state(robot_localization::StateMemberVpitch));
  EXPECT_FLOAT_EQ(0, state(robot_localization::StateMemberVyaw));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("test_ros_robot_localization_listener");

  g_listener = std::make_unique<robot_localization::RosRobotLocalizationListener>(node);

  ::testing::InitGoogleTest(&argc, argv);

  int res = RUN_ALL_TESTS();

  rclcpp::shutdown();
  node = nullptr;
  g_listener = nullptr;

  return res;
}
