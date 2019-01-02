/*
 * Copyright (c) 2014, 2015, 2016 Charles River Analytics, Inc.
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

// Header file names has been updated as per ros2
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include "robot_localization/srv/set_pose.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

nav_msgs::msg::Odometry filtered_;
bool stateUpdated_;

void filterCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  filtered_ = *msg;
  stateUpdated_ = true;
}

void resetFilter(rclcpp::Node::SharedPtr node_)
{
  // ros2 type service-client has been implemented
  auto client =
    node_->create_client<robot_localization::srv::SetPose>("set_pose");
  auto setPoseRequest =
    std::make_shared<robot_localization::srv::SetPose::Request>();

  setPoseRequest->pose.pose.pose.orientation.w = 1;
  setPoseRequest->pose.header.frame_id = "odom";

  for (size_t ind = 0; ind < 36; ind += 7) {
    setPoseRequest->pose.pose.covariance[ind] = 1e-6;
  }

  setPoseRequest->pose.header.stamp = node_->now();

  if (!client->wait_for_service(10s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }

  auto result = client->async_send_request(setPoseRequest);
  auto ret = rclcpp::spin_until_future_complete(node_, result,
      5s);                                            // Wait for the result.

  double deltaX = 0.0;
  double deltaY = 0.0;
  double deltaZ = 0.0;

  if (ret == rclcpp::executor::FutureReturnCode::SUCCESS) {
    // timing and spinning has been changed as per ros2
    rclcpp::spin_some(node_);
    rclcpp::Rate(100).sleep();
    deltaX = filtered_.pose.pose.position.x -
      setPoseRequest->pose.pose.pose.position.x;
    deltaY = filtered_.pose.pose.position.y -
      setPoseRequest->pose.pose.pose.position.y;
    deltaZ = filtered_.pose.pose.position.z -
      setPoseRequest->pose.pose.pose.position.z;
  }

  EXPECT_LT(::sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ), 0.1);
}
TEST(InterfacesTest, OdomPoseBasicIO) {
  stateUpdated_ = false;
  // node handle is created as per ros2
  auto node_ =
    rclcpp::Node::make_shared("InterfacesTest_OdomPoseBasicIO_testcase");

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 5;
  // publish and subscribe calls have been changed as per ros2
  auto odomPub = node_->create_publisher<nav_msgs::msg::Odometry>(
    "odom_input0", custom_qos_profile);

  auto filteredSub = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", filterCallback);

  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 20.0;
  odom.pose.pose.position.y = 10.0;
  odom.pose.pose.position.z = -40.0;

  odom.pose.covariance[0] = 2.0;
  odom.pose.covariance[7] = 2.0;
  odom.pose.covariance[14] = 2.0;

  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  // changed the spinnin and timing as per ros2
  rclcpp::Rate loopRate(50);
  for (size_t i = 0; i < 50; ++i) {
    odom.header.stamp = node_->now();
    odomPub->publish(odom);
    rclcpp::spin_some(node_);
    loopRate.sleep();
  }

  // Now check the values from the callback
  EXPECT_LT(::fabs(filtered_.pose.pose.position.x - odom.pose.pose.position.x),
    0.01);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y),
    0.01);          // Configuration for this variable for this sensor is false
  EXPECT_LT(::fabs(filtered_.pose.pose.position.z - odom.pose.pose.position.z),
    0.01);

  EXPECT_LT(filtered_.pose.covariance[0], 0.5);
  EXPECT_LT(filtered_.pose.covariance[7],
    0.25);          // Configuration for this variable for this sensor is false
  EXPECT_LT(filtered_.pose.covariance[14], 0.5);

  resetFilter(node_);
}

TEST(InterfacesTest, OdomTwistBasicIO) {
  // node handle is created as per ros2
  auto node_ =
    rclcpp::Node::make_shared("InterfacesTest_OdomTwistBasicIO_testcase");

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 5;
  // publish and subscribe calls have been changed as per ros2
  auto odomPub = node_->create_publisher<nav_msgs::msg::Odometry>(
    "odom_input2", custom_qos_profile);

  auto filteredSub = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", filterCallback);

  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = 5.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = 0.0;

  for (size_t ind = 0; ind < 36; ind += 7) {
    odom.twist.covariance[ind] = 1e-6;
  }

  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  // changed the spinnin and timing as per ros2
  rclcpp::Rate loopRate(20);
  for (size_t i = 0; i < 400; ++i) {
    odom.header.stamp = node_->now();
    odomPub->publish(odom);
    rclcpp::spin_some(node_);
    loopRate.sleep();
  }
  rclcpp::spin_some(node_);

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - odom.twist.twist.linear.x),
    0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.x - 100.0), 2.0);

  resetFilter(node_);

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 5.0;

  for (size_t i = 0; i < 200; ++i) {
    odom.header.stamp = node_->now();
    odomPub->publish(odom);
    rclcpp::spin_some(node_);

    loopRate.sleep();
  }
  rclcpp::spin_some(node_);

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.y - odom.twist.twist.linear.y),
    0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y - 50.0), 1.0);

  resetFilter(node_);

  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 5.0;

  for (size_t i = 0; i < 100; ++i) {
    odom.header.stamp = node_->now();
    odomPub->publish(odom);
    rclcpp::spin_some(node_);
    loopRate.sleep();
  }
  rclcpp::spin_some(node_);

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.z - odom.twist.twist.linear.z),
    0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.z - 25.0), 1.0);

  resetFilter(node_);

  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.linear.x = 1.0;
  odom.twist.twist.angular.z = (M_PI / 2) / (100.0 * 0.05);

  for (size_t i = 0; i < 100; ++i) {
    odom.header.stamp = node_->now();
    odomPub->publish(odom);
    rclcpp::spin_some(node_);
    loopRate.sleep();
  }
  rclcpp::spin_some(node_);

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - odom.twist.twist.linear.x),
    0.1);
  EXPECT_LT(
    ::fabs(filtered_.twist.twist.angular.z - odom.twist.twist.angular.z),
    0.1);
  EXPECT_LT(
    ::fabs(filtered_.pose.pose.position.x - filtered_.pose.pose.position.y),
    0.5);

  resetFilter(node_);

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.angular.z = 0.0;
  odom.twist.twist.angular.x = -(M_PI / 2) / (100.0 * 0.05);

  // First, roll the vehicle on its side
  for (size_t i = 0; i < 100; ++i) {
    odom.header.stamp = node_->now();
    odomPub->publish(odom);
    rclcpp::spin_some(node_);
    loopRate.sleep();
  }
  rclcpp::spin_some(node_);

  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = (M_PI / 2) / (100.0 * 0.05);

  // Now, pitch it down (positive pitch velocity in vehicle frame)
  for (size_t i = 0; i < 100; ++i) {
    odom.header.stamp = node_->now();
    odomPub->publish(odom);
    rclcpp::spin_some(node_);
    loopRate.sleep();
  }
  rclcpp::spin_some(node_);

  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.linear.x = 3.0;

  // We should now be on our side and facing -Y. Move forward in
  // the vehicle frame X direction, and make sure Y decreases in
  // the world frame.
  for (size_t i = 0; i < 100; ++i) {
    odom.header.stamp = node_->now();

    odomPub->publish(odom);
    rclcpp::spin_some(node_);
    loopRate.sleep();
  }
  rclcpp::spin_some(node_);

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - odom.twist.twist.linear.x),
    0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y + 15), 1.0);

  resetFilter(node_);
}

TEST(InterfacesTest, PoseBasicIO) {
  // node handle is created as per ros2
  auto node_ = rclcpp::Node::make_shared("InterfacesTest_PoseBasicIO_testcase");
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 5;
  // publish and subscribe calls have been changed as per ros2
  auto posePub =
    node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose_input0", custom_qos_profile);

  auto filteredSub = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", filterCallback);

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.pose.pose.position.x = 20.0;
  pose.pose.pose.position.y = 10.0;
  pose.pose.pose.position.z = -40.0;
  pose.pose.pose.orientation.x = 0;
  pose.pose.pose.orientation.y = 0;
  pose.pose.pose.orientation.z = 0;
  pose.pose.pose.orientation.w = 1;

  for (size_t ind = 0; ind < 36; ind += 7) {
    pose.pose.covariance[ind] = 1e-6;
  }

  pose.header.frame_id = "odom";
  // changed the spinnin and timing as per ros2
  rclcpp::Rate loopRate(50);

  for (size_t i = 0; i < 50; ++i) {
    pose.header.stamp = node_->now();
    posePub->publish(pose);
    rclcpp::spin_some(node_);
    loopRate.sleep();
  }

  // Now check the values from the callback
  EXPECT_LT(::fabs(filtered_.pose.pose.position.x - pose.pose.pose.position.x),
    0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y),
    0.1);          // Configuration for this variable for this sensor is false
  EXPECT_LT(::fabs(filtered_.pose.pose.position.z - pose.pose.pose.position.z),
    0.1);

  EXPECT_LT(filtered_.pose.covariance[0], 0.5);
  EXPECT_LT(filtered_.pose.covariance[7],
    0.25);          // Configuration for this variable for this sensor is false
  EXPECT_LT(filtered_.pose.covariance[14], 0.5);

  resetFilter(node_);
}

TEST(InterfacesTest, TwistBasicIO) {
  // node handle is created as per ros2
  auto node_ =
    rclcpp::Node::make_shared("InterfacesTest_TwistBasicIO_testcase");
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 5;
  // publish and subscribe calls have been changed as per ros2
  auto twistPub =
    node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_input0", custom_qos_profile);

  auto filteredSub = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", filterCallback);

  geometry_msgs::msg::TwistWithCovarianceStamped twist;
  twist.twist.twist.linear.x = 5.0;
  twist.twist.twist.linear.y = 0;
  twist.twist.twist.linear.z = 0;
  twist.twist.twist.angular.x = 0;
  twist.twist.twist.angular.y = 0;
  twist.twist.twist.angular.z = 0;

  for (size_t ind = 0; ind < 36; ind += 7) {
    twist.twist.covariance[ind] = 1e-6;
  }

  twist.header.frame_id = "base_link";
  // changed the spinnin and timing as per ros2
  rclcpp::Rate loopRate(20);
  for (size_t i = 0; i < 400; ++i) {
    twist.header.stamp = node_->now();
    twistPub->publish(twist);
    rclcpp::spin_some(node_);
    loopRate.sleep();
  }
  rclcpp::spin_some(node_);

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - twist.twist.twist.linear.x),
    0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.x - 100.0), 2.0);

  resetFilter(node_);

  twist.twist.twist.linear.x = 0.0;
  twist.twist.twist.linear.y = 5.0;

  for (size_t i = 0; i < 200; ++i) {
    twist.header.stamp = node_->now();
    twistPub->publish(twist);
    rclcpp::spin_some(node_);

    loopRate.sleep();
  }
  rclcpp::spin_some(node_);

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.y - twist.twist.twist.linear.y),
    0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y - 50.0), 1.0);

  resetFilter(node_);

  twist.twist.twist.linear.y = 0.0;
  twist.twist.twist.linear.z = 5.0;

  for (size_t i = 0; i < 100; ++i) {
    twist.header.stamp = node_->now();
    twistPub->publish(twist);
    rclcpp::spin_some(node_);
    loopRate.sleep();
  }
  rclcpp::spin_some(node_);

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.z - twist.twist.twist.linear.z),
    0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.z - 25.0), 1.0);

  resetFilter(node_);

  twist.twist.twist.linear.z = 0.0;
  twist.twist.twist.linear.x = 1.0;
  twist.twist.twist.angular.z = (M_PI / 2) / (100.0 * 0.05);
  for (size_t i = 0; i < 100; ++i) {
    twist.header.stamp = node_->now();
    twistPub->publish(twist);
    rclcpp::spin_some(node_);
    loopRate.sleep();
  }
  rclcpp::spin_some(node_);

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - twist.twist.twist.linear.x),
    0.1);
  EXPECT_LT(
    ::fabs(filtered_.twist.twist.angular.z - twist.twist.twist.angular.z),
    0.1);
  EXPECT_LT(
    ::fabs(filtered_.pose.pose.position.x - filtered_.pose.pose.position.y),
    0.5);

  resetFilter(node_);

  twist.twist.twist.linear.x = 0.0;
  twist.twist.twist.angular.z = 0.0;
  twist.twist.twist.angular.x = -(M_PI / 2) / (100.0 * 0.05);

  // First, roll the vehicle on its side
  for (size_t i = 0; i < 100; ++i) {
    twist.header.stamp = node_->now();
    twistPub->publish(twist);
    rclcpp::spin_some(node_);

    loopRate.sleep();
  }
  rclcpp::spin_some(node_);

  twist.twist.twist.angular.x = 0.0;
  twist.twist.twist.angular.y = (M_PI / 2) / (100.0 * 0.05);

  // Now, pitch it down (positive pitch velocity in vehicle frame)
  for (size_t i = 0; i < 100; ++i) {
    twist.header.stamp = node_->now();
    twistPub->publish(twist);
    rclcpp::spin_some(node_);

    loopRate.sleep();
  }
  rclcpp::spin_some(node_);

  twist.twist.twist.angular.y = 0.0;
  twist.twist.twist.linear.x = 3.0;

  // We should now be on our side and facing -Y. Move forward in
  // the vehicle frame X direction, and make sure Y decreases in
  // the world frame.
  for (size_t i = 0; i < 100; ++i) {
    twist.header.stamp = node_->now();
    twistPub->publish(twist);
    rclcpp::spin_some(node_);
    loopRate.sleep();
  }
  rclcpp::spin_some(node_);

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - twist.twist.twist.linear.x),
    0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y + 15), 1.0);

  resetFilter(node_);
}

TEST(InterfacesTest, ImuPoseBasicIO) {
  // node handle is created as per ros2
  auto node_ =
    rclcpp::Node::make_shared("InterfacesTest_ImuPoseBasicIO_testcase");
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 5;
  // publish and subscribe calls have been changed as per ros2
  auto imuPub = node_->create_publisher<sensor_msgs::msg::Imu>(
    "/imu_input0", custom_qos_profile);

  auto filteredSub = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", filterCallback);

  sensor_msgs::msg::Imu imu;
  tf2::Quaternion quat;
  quat.setRPY(M_PI / 4, -M_PI / 4, M_PI / 2);
  imu.orientation = tf2::toMsg(quat);

  for (size_t ind = 0; ind < 9; ind += 4) {
    imu.orientation_covariance[ind] = 1e-6;
  }

  imu.header.frame_id = "base_link";

  // Make sure the pose reset worked. Test will timeout
  // if this fails.
  // changed the spinnin and timing as per ros2
  rclcpp::Rate loopRate1(50);
  for (size_t i = 0; i < 50; ++i) {
    imu.header.stamp = node_->now();
    imuPub->publish(imu);
    rclcpp::spin_some(node_);
    loopRate1.sleep();
  }

  // Now check the values from the callback
  tf2::fromMsg(filtered_.pose.pose.orientation, quat);
  tf2::Matrix3x3 mat(quat);
  double r, p, y;
  mat.getRPY(r, p, y);
  EXPECT_LT(::fabs(r - M_PI / 4), 0.1);
  EXPECT_LT(::fabs(p + M_PI / 4), 0.1);
  EXPECT_LT(::fabs(y - M_PI / 2), 0.1);

  EXPECT_LT(filtered_.pose.covariance[21], 0.5);
  EXPECT_LT(filtered_.pose.covariance[28], 0.25);
  EXPECT_LT(filtered_.pose.covariance[35], 0.5);

  resetFilter(node_);

  // Test to see if the orientation data is ignored when we set the
  // first covariance value to -1
  sensor_msgs::msg::Imu imuIgnore;
  imuIgnore.orientation.x = 0.1;
  imuIgnore.orientation.y = 0.2;
  imuIgnore.orientation.z = 0.3;
  imuIgnore.orientation.w = 0.4;
  imuIgnore.orientation_covariance[0] = -1;

  rclcpp::Rate loopRate2(50);
  for (size_t i = 0; i < 50; ++i) {
    imuIgnore.header.stamp = node_->now();
    imuPub->publish(imuIgnore);
    loopRate2.sleep();
    rclcpp::spin_some(node_);
  }

  tf2::fromMsg(filtered_.pose.pose.orientation, quat);
  mat.setRotation(quat);
  mat.getRPY(r, p, y);

  EXPECT_LT(::fabs(r), 1e-3);
  EXPECT_LT(::fabs(p), 1e-3);
  EXPECT_LT(::fabs(y), 1e-3);

  resetFilter(node_);
}

TEST(InterfacesTest, ImuTwistBasicIO) {
  // node handle is created as per ros2
  auto node_ =
    rclcpp::Node::make_shared("InterfacesTest_ImuTwistBasicIO_testcase");
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 5;
  // publish and subscribe calls have been changed as per ros2
  auto imuPub = node_->create_publisher<sensor_msgs::msg::Imu>(
    "/imu_input1", custom_qos_profile);

  auto filteredSub = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", filterCallback);

  sensor_msgs::msg::Imu imu;
  tf2::Quaternion quat;
  imu.angular_velocity.x = (M_PI / 2.0);

  for (size_t ind = 0; ind < 9; ind += 4) {
    imu.angular_velocity_covariance[ind] = 1e-6;
  }

  imu.header.frame_id = "base_link";
  // changed the spinnin and timing as per ros2
  rclcpp::Rate loopRate(50);
  for (size_t i = 0; i < 50; ++i) {
    imu.header.stamp = node_->now();
    imuPub->publish(imu);
    loopRate.sleep();
    rclcpp::spin_some(node_);
  }

  // Now check the values from the callback
  tf2::fromMsg(filtered_.pose.pose.orientation, quat);
  tf2::Matrix3x3 mat(quat);
  double r, p, y;
  mat.getRPY(r, p, y);

  // Tolerances may seem loose, but the initial state covariances
  // are tiny, so the filter is sluggish to accept velocity data
  EXPECT_LT(::fabs(r - M_PI / 2.0), 0.7);
  EXPECT_LT(::fabs(p), 0.1);
  EXPECT_LT(::fabs(y), 0.1);

  EXPECT_LT(filtered_.twist.covariance[21], 1e-3);
  EXPECT_LT(filtered_.pose.covariance[21], 0.1);

  resetFilter(node_);

  imu.angular_velocity.x = 0.0;
  imu.angular_velocity.y = -(M_PI / 2.0);

  // Make sure the pose reset worked. Test will timeout
  // if this fails.
  for (size_t i = 0; i < 50; ++i) {
    imu.header.stamp = node_->now();
    imuPub->publish(imu);
    loopRate.sleep();
    rclcpp::spin_some(node_);
  }

  // Now check the values from the callback
  tf2::fromMsg(filtered_.pose.pose.orientation, quat);
  mat.setRotation(quat);
  mat.getRPY(r, p, y);
  EXPECT_LT(::fabs(r), 0.1);
  EXPECT_LT(::fabs(p + M_PI / 2.0), 0.7);
  EXPECT_LT(::fabs(y), 0.1);

  EXPECT_LT(filtered_.twist.covariance[28], 1e-3);
  EXPECT_LT(filtered_.pose.covariance[28], 0.1);

  resetFilter(node_);

  imu.angular_velocity.y = 0;
  imu.angular_velocity.z = M_PI / 4.0;

  // Make sure the pose reset worked. Test will timeout
  // if this fails.
  for (size_t i = 0; i < 50; ++i) {
    imu.header.stamp = node_->now();
    imuPub->publish(imu);
    loopRate.sleep();
    rclcpp::spin_some(node_);
  }

  // Now check the values from the callback
  tf2::fromMsg(filtered_.pose.pose.orientation, quat);
  mat.setRotation(quat);
  mat.getRPY(r, p, y);
  EXPECT_LT(::fabs(r), 0.1);
  EXPECT_LT(::fabs(p), 0.1);
  EXPECT_LT(::fabs(y - M_PI / 4.0), 0.7);

  EXPECT_LT(filtered_.twist.covariance[35], 1e-3);
  EXPECT_LT(filtered_.pose.covariance[35], 0.1);

  resetFilter(node_);

  // Test to see if the angular velocity data is ignored when we set the
  // first covariance value to -1
  sensor_msgs::msg::Imu imuIgnore;
  imuIgnore.angular_velocity.x = 100;
  imuIgnore.angular_velocity.y = 100;
  imuIgnore.angular_velocity.z = 100;
  imuIgnore.angular_velocity_covariance[0] = -1;

  for (size_t i = 0; i < 50; ++i) {
    imuIgnore.header.stamp = node_->now();
    imuPub->publish(imuIgnore);
    loopRate.sleep();
    rclcpp::spin_some(node_);
  }

  tf2::fromMsg(filtered_.pose.pose.orientation, quat);
  mat.setRotation(quat);
  mat.getRPY(r, p, y);
  EXPECT_LT(::fabs(r), 1e-3);
  EXPECT_LT(::fabs(p), 1e-3);
  EXPECT_LT(::fabs(y), 1e-3);

  resetFilter(node_);
}

TEST(InterfacesTest, ImuAccBasicIO) {
  // node handle is created as per ros2
  auto node_ =
    rclcpp::Node::make_shared("InterfacesTest_ImuAccBasicIO_testcase");
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 5;
  // publish and subscribe calls have been changed as per ros2
  auto imuPub = node_->create_publisher<sensor_msgs::msg::Imu>(
    "imu_input2", custom_qos_profile);

  auto filteredSub = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", filterCallback);

  sensor_msgs::msg::Imu imu;
  imu.header.frame_id = "base_link";
  imu.linear_acceleration_covariance[0] = 1e-6;
  imu.linear_acceleration_covariance[4] = 1e-6;
  imu.linear_acceleration_covariance[8] = 1e-6;

  imu.linear_acceleration.x = 1;
  imu.linear_acceleration.y = -1;
  imu.linear_acceleration.z = 1;

  // Move with constant acceleration for 1 second,
  // then check our velocity.
  // changed the spinnin and timing as per ros2
  rclcpp::Rate loopRate(50);
  for (size_t i = 0; i < 50; ++i) {
    imu.header.stamp = node_->now();
    imuPub->publish(imu);
    loopRate.sleep();
    rclcpp::spin_some(node_);
  }

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - 1.0), 0.4);
  EXPECT_LT(::fabs(filtered_.twist.twist.linear.y + 1.0), 0.4);
  EXPECT_LT(::fabs(filtered_.twist.twist.linear.z - 1.0), 0.4);

  imu.linear_acceleration.x = 0.0;
  imu.linear_acceleration.y = 0.0;
  imu.linear_acceleration.z = 0.0;

  // Now move for another second, and see where we end up
  for (size_t i = 0; i < 50; ++i) {
    imu.header.stamp = node_->now();
    imuPub->publish(imu);
    loopRate.sleep();
    rclcpp::spin_some(node_);
  }

  EXPECT_LT(::fabs(filtered_.pose.pose.position.x - 1.2), 0.4);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y + 1.2), 0.4);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.z - 1.2), 0.4);

  resetFilter(node_);

  // Test to see if the linear acceleration data is ignored when we set the
  // first covariance value to -1
  sensor_msgs::msg::Imu imuIgnore;
  imuIgnore.linear_acceleration.x = 1000;
  imuIgnore.linear_acceleration.y = 1000;
  imuIgnore.linear_acceleration.z = 1000;
  imuIgnore.linear_acceleration_covariance[0] = -1;

  for (size_t i = 0; i < 50; ++i) {
    imuIgnore.header.stamp = node_->now();
    imuPub->publish(imuIgnore);
    loopRate.sleep();
    rclcpp::spin_some(node_);
  }

  EXPECT_LT(::fabs(filtered_.pose.pose.position.x), 1e-3);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y), 1e-3);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.z), 1e-3);

  resetFilter(node_);
}

TEST(InterfacesTest, OdomDifferentialIO) {
  // node handle is created as per ros2
  auto node_ =
    rclcpp::Node::make_shared("InterfacesTest_OdomDifferentialIO_testcase");
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 5;
  // publish and subscribe calls have been changed as per ros2
  auto odomPub = node_->create_publisher<nav_msgs::msg::Odometry>(
    "/odom_input1", custom_qos_profile);

  auto filteredSub = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", filterCallback);

  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 20.0;
  odom.pose.pose.position.y = 10.0;
  odom.pose.pose.position.z = -40.0;

  odom.pose.pose.orientation.w = 1;

  odom.pose.covariance[0] = 2.0;
  odom.pose.covariance[7] = 2.0;
  odom.pose.covariance[14] = 2.0;
  odom.pose.covariance[21] = 0.2;
  odom.pose.covariance[28] = 0.2;
  odom.pose.covariance[35] = 0.2;

  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  // No guaranteeing that the zero state
  // we're expecting to see here isn't just
  // a result of zeroing it out previously,
  // so check 10 times in succession.
  size_t zeroCount = 0;
  while (zeroCount++ < 10) {
    odom.header.stamp = node_->now();
    odomPub->publish(odom);
    rclcpp::spin_some(node_);

    EXPECT_LT(::fabs(filtered_.pose.pose.position.x), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.position.y), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.position.z), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.orientation.x), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.orientation.y), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.orientation.z), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.orientation.w - 1), 0.01);
    rclcpp::Rate(10).sleep();
  }

  for (size_t ind = 0; ind < 36; ind += 7) {
    odom.pose.covariance[ind] = 1e-6;
  }

  // Slowly move the position, and hope that the differential position keeps up
  // changed the spinnin and timing as per ros2
  rclcpp::Rate loopRate(20);
  for (size_t i = 0; i < 100; ++i) {
    odom.pose.pose.position.x += 0.01;
    odom.pose.pose.position.y += 0.02;
    odom.pose.pose.position.z -= 0.03;

    odom.header.stamp = node_->now();
    odomPub->publish(odom);
    rclcpp::spin_some(node_);
    loopRate.sleep();
  }
  rclcpp::spin_some(node_);

  EXPECT_LT(::fabs(filtered_.pose.pose.position.x - 1), 0.2);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y - 2), 0.4);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.z + 3), 0.6);

  resetFilter(node_);
}

TEST(InterfacesTest, PoseDifferentialIO) {
  // node handle is created as per ros2
  auto node_ =
    rclcpp::Node::make_shared("InterfacesTest_PoseDifferentialIO_testcase");
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 5;
  // publish and subscribe calls have been changed as per ros2
  auto posePub =
    node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/pose_input1", custom_qos_profile);

  auto filteredSub = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", filterCallback);

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.pose.pose.position.x = 20.0;
  pose.pose.pose.position.y = 10.0;
  pose.pose.pose.position.z = -40.0;

  pose.pose.pose.orientation.w = 1;

  pose.pose.covariance[0] = 2.0;
  pose.pose.covariance[7] = 2.0;
  pose.pose.covariance[14] = 2.0;
  pose.pose.covariance[21] = 0.2;
  pose.pose.covariance[28] = 0.2;
  pose.pose.covariance[35] = 0.2;

  pose.header.frame_id = "odom";

  // No guaranteeing that the zero state
  // we're expecting to see here isn't just
  // a result of zeroing it out previously,
  // so check 10 times in succession.
  size_t zeroCount = 0;
  while (zeroCount++ < 10) {
    pose.header.stamp = node_->now();
    posePub->publish(pose);
    rclcpp::spin_some(node_);

    EXPECT_LT(::fabs(filtered_.pose.pose.position.x), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.position.y), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.position.z), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.orientation.x), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.orientation.y), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.orientation.z), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.orientation.w - 1), 0.01);

    rclcpp::Rate(10).sleep();
  }

  // ...but only if we give the measurement a tiny covariance
  for (size_t ind = 0; ind < 36; ind += 7) {
    pose.pose.covariance[ind] = 1e-6;
  }

  // Issue this location repeatedly, and see if we get
  // a final reported position of (1, 2, -3)
  // changed the spinnin and timing as per ros2
  rclcpp::Rate loopRate(20);
  for (size_t i = 0; i < 100; ++i) {
    pose.pose.pose.position.x += 0.01;
    pose.pose.pose.position.y += 0.02;
    pose.pose.pose.position.z -= 0.03;

    pose.header.stamp = node_->now();
    posePub->publish(pose);
    rclcpp::spin_some(node_);
    loopRate.sleep();
  }
  rclcpp::spin_some(node_);

  EXPECT_LT(::fabs(filtered_.pose.pose.position.x - 1), 0.2);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y - 2), 0.4);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.z + 3), 0.6);

  resetFilter(node_);
}

TEST(InterfacesTest, ImuDifferentialIO) {
  // node handle is created as per ros2
  auto node_ =
    rclcpp::Node::make_shared("InterfacesTest_ImuDifferentialIO_testcase");
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 5;
  // publish and subscribe calls have been changed as per ros2
  auto imu0Pub = node_->create_publisher<sensor_msgs::msg::Imu>(
    "/imu_input0", custom_qos_profile);
  auto imu1Pub = node_->create_publisher<sensor_msgs::msg::Imu>(
    "/imu_input1", custom_qos_profile);
  auto imuPub = node_->create_publisher<sensor_msgs::msg::Imu>(
    "/imu_input3", custom_qos_profile);

  auto filteredSub = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", filterCallback);

  sensor_msgs::msg::Imu imu;
  imu.header.frame_id = "base_link";
  tf2::Quaternion quat;
  const double roll = M_PI / 2.0;
  const double pitch = -M_PI;
  const double yaw = -M_PI / 4.0;
  quat.setRPY(roll, pitch, yaw);
  imu.orientation = tf2::toMsg(quat);

  imu.orientation_covariance[0] = 0.02;
  imu.orientation_covariance[4] = 0.02;
  imu.orientation_covariance[8] = 0.02;

  imu.angular_velocity_covariance[0] = 0.02;
  imu.angular_velocity_covariance[4] = 0.02;
  imu.angular_velocity_covariance[8] = 0.02;

  size_t setCount = 0;
  while (setCount++ < 10) {
    imu.header.stamp = node_->now();
    imu0Pub->publish(imu);  // Use this to move the absolute orientation
    imu1Pub->publish(imu);  // Use this to keep velocities at 0
    rclcpp::spin_some(node_);
    rclcpp::Rate(10).sleep();
  }

  size_t zeroCount = 0;
  while (zeroCount++ < 10) {
    imu.header.stamp = node_->now();
    imuPub->publish(imu);
    rclcpp::spin_some(node_);
    rclcpp::Rate(10).sleep();
  }

  double rollFinal = roll;
  double pitchFinal = pitch;
  double yawFinal = yaw;

  // Move the orientation slowly, and see if we can push it to 0
  // changed the spinnin and timing as per ros2
  rclcpp::Rate loopRate(20);
  for (size_t i = 0; i < 100; ++i) {
    yawFinal -= 0.01 * (3.0 * M_PI / 4.0);

    quat.setRPY(rollFinal, pitchFinal, yawFinal);

    imu.orientation = tf2::toMsg(quat);
    imu.header.stamp = node_->now();
    imuPub->publish(imu);
    rclcpp::spin_some(node_);

    loopRate.sleep();
  }
  rclcpp::spin_some(node_);

  // Move the orientation slowly, and see if we can push it to 0
  for (size_t i = 0; i < 100; ++i) {
    rollFinal += 0.01 * (M_PI / 2.0);

    quat.setRPY(rollFinal, pitchFinal, yawFinal);

    imu.orientation = tf2::toMsg(quat);
    imu.header.stamp = node_->now();
    imuPub->publish(imu);
    rclcpp::spin_some(node_);

    loopRate.sleep();
  }
  rclcpp::spin_some(node_);

  tf2::fromMsg(filtered_.pose.pose.orientation, quat);
  tf2::Matrix3x3 mat(quat);
  mat.getRPY(rollFinal, pitchFinal, yawFinal);

  EXPECT_LT(::fabs(rollFinal), 0.2);
  EXPECT_LT(::fabs(pitchFinal), 0.2);
  EXPECT_LT(::fabs(yawFinal), 0.2);

  resetFilter(node_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  // Give ekf_localization_node time to initialize
  rclcpp::Rate(0.5).sleep();

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
