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
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "gtest/gtest.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

nav_msgs::msg::Odometry filtered_;
bool state_updated_;

rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom0_pub_;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom1_pub_;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom2_pub_;
rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose0_pub_;
rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose1_pub_;
rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist0_pub_;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu0_pub_;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu1_pub_;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu2_pub_;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu3_pub_;
rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;

void filterCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  filtered_ = *msg;
  state_updated_ = true;
}

void resetFilter()
{
  // Force any callbacks to fire in the UKF
  rclcpp::spin_some(node_);

  auto reset_request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto result = reset_client_->async_send_request(reset_request);
  rclcpp::spin_until_future_complete(node_, result, 5s);  // Wait for the result

  // Reset the output message
  filtered_ = nav_msgs::msg::Odometry();
  filtered_.pose.pose.orientation.w = 1.0;
}

TEST(InterfacesTest, OdomPoseBasicIO) {
  state_updated_ = false;

  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 20.0;
  odom.pose.pose.position.y = 10.0;
  odom.pose.pose.position.z = -40.0;

  odom.pose.covariance[0] = 2.0;
  odom.pose.covariance[7] = 2.0;
  odom.pose.covariance[14] = 2.0;

  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  rclcpp::Rate loop_rate(50);
  for (size_t i = 0; i < 50; ++i) {
    odom.header.stamp = node_->now();
    odom0_pub_->publish(odom);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  // Now check the values from the callback
  EXPECT_LT(std::abs(filtered_.pose.pose.position.x - odom.pose.pose.position.x), 0.01);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.y), 0.01);  // Y is not fused
  EXPECT_LT(std::abs(filtered_.pose.pose.position.z - odom.pose.pose.position.z), 0.01);

  EXPECT_LT(filtered_.pose.covariance[0], 0.5);
  EXPECT_LT(filtered_.pose.covariance[7], 0.25);  // Y is not fused
  EXPECT_LT(filtered_.pose.covariance[14], 0.6);

  resetFilter();
}

TEST(InterfacesTest, OdomTwistBasicIO) {
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
  rclcpp::Rate loop_rate(20);
  for (size_t i = 0; i < 400; ++i) {
    odom.header.stamp = node_->now();
    odom2_pub_->publish(odom);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  EXPECT_LT(std::abs(filtered_.twist.twist.linear.x - odom.twist.twist.linear.x), 0.1);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.x - 100.0), 2.0);

  resetFilter();

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 5.0;

  for (size_t i = 0; i < 400; ++i) {
    odom.header.stamp = node_->now();
    odom2_pub_->publish(odom);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  EXPECT_LT(std::abs(filtered_.twist.twist.linear.y - odom.twist.twist.linear.y), 0.1);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.y - 100.0), 2.0);

  resetFilter();

  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 5.0;

  for (size_t i = 0; i < 400; ++i) {
    odom.header.stamp = node_->now();
    odom2_pub_->publish(odom);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  EXPECT_LT(std::abs(filtered_.twist.twist.linear.z - odom.twist.twist.linear.z), 0.1);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.z - 100.0), 2.0);

  resetFilter();

  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.linear.x = 1.0;
  odom.twist.twist.angular.z = (M_PI / 2) / (100.0 * 0.05);

  for (size_t i = 0; i < 100; ++i) {
    odom.header.stamp = node_->now();
    odom2_pub_->publish(odom);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  EXPECT_LT(std::abs(filtered_.twist.twist.linear.x - odom.twist.twist.linear.x), 0.1);
  EXPECT_LT(std::abs(filtered_.twist.twist.angular.z - odom.twist.twist.angular.z), 0.1);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.x - filtered_.pose.pose.position.y), 0.5);

  resetFilter();

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.angular.z = 0.0;
  odom.twist.twist.angular.x = -(M_PI / 2) / (100.0 * 0.05);

  // First, roll the vehicle on its side
  for (size_t i = 0; i < 100; ++i) {
    odom.header.stamp = node_->now();
    odom2_pub_->publish(odom);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = (M_PI / 2) / (100.0 * 0.05);

  // Now, pitch it down (positive pitch velocity in vehicle frame)
  for (size_t i = 0; i < 100; ++i) {
    odom.header.stamp = node_->now();
    odom2_pub_->publish(odom);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.linear.x = 3.0;

  // We should now be on our side and facing -Y. Move forward in
  // the vehicle frame X direction, and make sure Y decreases in
  // the world frame.
  for (size_t i = 0; i < 100; ++i) {
    odom.header.stamp = node_->now();
    odom2_pub_->publish(odom);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  EXPECT_LT(std::abs(filtered_.twist.twist.linear.x - odom.twist.twist.linear.x), 0.1);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.y + 15), 1.0);

  resetFilter();
}

TEST(InterfacesTest, PoseBasicIO) {
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
  rclcpp::Rate loop_rate(50);
  for (size_t i = 0; i < 50; ++i) {
    pose.header.stamp = node_->now();
    pose0_pub_->publish(pose);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  // Now check the values from the callback
  EXPECT_LT(std::abs(filtered_.pose.pose.position.x - pose.pose.pose.position.x), 0.1);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.y), 0.1);  // Y is not fused
  EXPECT_LT(std::abs(filtered_.pose.pose.position.z - pose.pose.pose.position.z), 0.1);
  EXPECT_LT(filtered_.pose.covariance[0], 0.5);
  EXPECT_LT(filtered_.pose.covariance[7], 0.25);  // Y is not fused
  EXPECT_LT(filtered_.pose.covariance[14], 0.5);

  resetFilter();
}

TEST(InterfacesTest, TwistBasicIO) {
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
  rclcpp::Rate loop_rate(20);
  for (size_t i = 0; i < 400; ++i) {
    twist.header.stamp = node_->now();
    twist0_pub_->publish(twist);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  EXPECT_LT(std::abs(filtered_.twist.twist.linear.x - twist.twist.twist.linear.x), 0.1);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.x - 100.0), 2.0);

  resetFilter();

  twist.twist.twist.linear.x = 0.0;
  twist.twist.twist.linear.y = 5.0;

  for (size_t i = 0; i < 400; ++i) {
    twist.header.stamp = node_->now();
    twist0_pub_->publish(twist);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  EXPECT_LT(
    std::abs(filtered_.twist.twist.linear.y - twist.twist.twist.linear.y),
    0.1);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.y - 100.0), 2.0);

  resetFilter();

  twist.twist.twist.linear.y = 0.0;
  twist.twist.twist.linear.z = 5.0;

  for (size_t i = 0; i < 400; ++i) {
    twist.header.stamp = node_->now();
    twist0_pub_->publish(twist);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  EXPECT_LT(
    std::abs(filtered_.twist.twist.linear.z - twist.twist.twist.linear.z),
    0.1);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.z - 100.0), 2.0);

  resetFilter();

  twist.twist.twist.linear.z = 0.0;
  twist.twist.twist.linear.x = 1.0;
  twist.twist.twist.angular.z = (M_PI / 2) / (100.0 * 0.05);
  for (size_t i = 0; i < 100; ++i) {
    twist.header.stamp = node_->now();
    twist0_pub_->publish(twist);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  EXPECT_LT(std::abs(filtered_.twist.twist.linear.x - twist.twist.twist.linear.x), 0.1);
  EXPECT_LT(std::abs(filtered_.twist.twist.angular.z - twist.twist.twist.angular.z), 0.1);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.x - filtered_.pose.pose.position.y), 0.5);

  resetFilter();

  twist.twist.twist.linear.x = 0.0;
  twist.twist.twist.angular.z = 0.0;
  twist.twist.twist.angular.x = -(M_PI / 2) / (100.0 * 0.05);

  // First, roll the vehicle on its side
  for (size_t i = 0; i < 100; ++i) {
    twist.header.stamp = node_->now();
    twist0_pub_->publish(twist);
    rclcpp::spin_some(node_);

    loop_rate.sleep();
  }

  twist.twist.twist.angular.x = 0.0;
  twist.twist.twist.angular.y = (M_PI / 2) / (100.0 * 0.05);

  // Now, pitch it down (positive pitch velocity in vehicle frame)
  for (size_t i = 0; i < 100; ++i) {
    twist.header.stamp = node_->now();
    twist0_pub_->publish(twist);
    rclcpp::spin_some(node_);

    loop_rate.sleep();
  }

  twist.twist.twist.angular.y = 0.0;
  twist.twist.twist.linear.x = 3.0;

  // We should now be on our side and facing -Y. Move forward in
  // the vehicle frame X direction, and make sure Y decreases in
  // the world frame.
  for (size_t i = 0; i < 100; ++i) {
    twist.header.stamp = node_->now();
    twist0_pub_->publish(twist);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  EXPECT_LT(std::abs(filtered_.twist.twist.linear.x - twist.twist.twist.linear.x), 0.1);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.y + 15), 1.0);

  resetFilter();
}

TEST(InterfacesTest, ImuPoseBasicIO) {
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
  rclcpp::Rate loop_rate1(50);
  for (size_t i = 0; i < 50; ++i) {
    imu.header.stamp = node_->now();
    imu0_pub_->publish(imu);
    loop_rate1.sleep();
    rclcpp::spin_some(node_);
  }

  // Now check the values from the callback
  tf2::fromMsg(filtered_.pose.pose.orientation, quat);
  tf2::Matrix3x3 mat(quat);
  double r, p, y;
  mat.getRPY(r, p, y);
  EXPECT_LT(std::abs(r - M_PI / 4), 0.1);
  EXPECT_LT(std::abs(p + M_PI / 4), 0.1);
  EXPECT_LT(std::abs(y - M_PI / 2), 0.1);

  EXPECT_LT(filtered_.pose.covariance[21], 0.5);
  EXPECT_LT(filtered_.pose.covariance[28], 0.25);
  EXPECT_LT(filtered_.pose.covariance[35], 0.5);

  resetFilter();

  state_updated_ = false;

  // Test to see if the orientation data is ignored when we set the
  // first covariance value to -1
  sensor_msgs::msg::Imu imuIgnore;
  imuIgnore.orientation.x = 0.1;
  imuIgnore.orientation.y = 0.2;
  imuIgnore.orientation.z = 0.3;
  imuIgnore.orientation.w = 0.4;
  imuIgnore.orientation_covariance[0] = -1;

  rclcpp::Rate loop_rate2(50);
  for (size_t i = 0; i < 50; ++i) {
    imuIgnore.header.stamp = node_->now();
    imu0_pub_->publish(imuIgnore);
    loop_rate2.sleep();
    rclcpp::spin_some(node_);
    EXPECT_FALSE(state_updated_);
  }

  tf2::fromMsg(filtered_.pose.pose.orientation, quat);
  mat.setRotation(quat);
  mat.getRPY(r, p, y);

  EXPECT_LT(std::abs(r), 1e-3);
  EXPECT_LT(std::abs(p), 1e-3);
  EXPECT_LT(std::abs(y), 1e-3);

  resetFilter();
}

TEST(InterfacesTest, ImuTwistBasicIO) {
  sensor_msgs::msg::Imu imu;
  tf2::Quaternion quat;
  imu.angular_velocity.x = (M_PI / 2.0);

  for (size_t ind = 0; ind < 9; ind += 4) {
    imu.angular_velocity_covariance[ind] = 1e-6;
  }

  imu.header.frame_id = "base_link";
  rclcpp::Rate loop_rate(50);
  for (size_t i = 0; i < 50; ++i) {
    imu.header.stamp = node_->now();
    imu1_pub_->publish(imu);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  // Now check the values from the callback
  tf2::fromMsg(filtered_.pose.pose.orientation, quat);
  tf2::Matrix3x3 mat(quat);
  double r, p, y;
  mat.getRPY(r, p, y);

  // Tolerances may seem loose, but the initial state covariances
  // are tiny, so the filter is sluggish to accept velocity data
  EXPECT_LT(std::abs(r - M_PI / 2.0), 0.7);
  EXPECT_LT(std::abs(p), 0.1);
  EXPECT_LT(std::abs(y), 0.1);

  EXPECT_LT(filtered_.twist.covariance[21], 1e-3);
  EXPECT_LT(filtered_.pose.covariance[21], 0.1);

  resetFilter();

  imu.angular_velocity.x = 0.0;
  imu.angular_velocity.y = -(M_PI / 3.0);

  // Make sure the pose reset worked. Test will timeout
  // if this fails.
  for (size_t i = 0; i < 50; ++i) {
    imu.header.stamp = node_->now();
    imu1_pub_->publish(imu);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }
  rclcpp::spin_some(node_);

  // Now check the values from the callback
  tf2::fromMsg(filtered_.pose.pose.orientation, quat);
  tf2::Quaternion expected_quat(tf2::Vector3(0., 1., 0.), -M_PI / 3.0);
  EXPECT_LT(std::abs(tf2Angle(expected_quat.getAxis(), quat.getAxis())), 0.1);
  EXPECT_LT(std::abs(expected_quat.getAngle() - quat.getAngle()), 0.7);
  EXPECT_LT(filtered_.twist.covariance[28], 1e-3);
  EXPECT_LT(filtered_.pose.covariance[28], 0.1);

  resetFilter();

  imu.angular_velocity.y = 0;
  imu.angular_velocity.z = M_PI / 4.0;

  // Make sure the pose reset worked. Test will timeout if this fails.
  for (size_t i = 0; i < 50; ++i) {
    imu.header.stamp = node_->now();
    imu1_pub_->publish(imu);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  // Now check the values from the callback
  tf2::fromMsg(filtered_.pose.pose.orientation, quat);
  mat.setRotation(quat);
  mat.getRPY(r, p, y);
  EXPECT_LT(std::abs(r), 0.1);
  EXPECT_LT(std::abs(p), 0.1);
  EXPECT_LT(std::abs(y - M_PI / 4.0), 0.7);

  EXPECT_LT(filtered_.twist.covariance[35], 1e-3);
  EXPECT_LT(filtered_.pose.covariance[35], 0.12);

  resetFilter();

  // Test to see if the angular velocity data is ignored when we set the
  // first covariance value to -1
  sensor_msgs::msg::Imu imuIgnore;
  imuIgnore.angular_velocity.x = 100;
  imuIgnore.angular_velocity.y = 100;
  imuIgnore.angular_velocity.z = 100;
  imuIgnore.angular_velocity_covariance[0] = -1;

  for (size_t i = 0; i < 50; ++i) {
    imuIgnore.header.stamp = node_->now();
    imu1_pub_->publish(imuIgnore);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  tf2::fromMsg(filtered_.pose.pose.orientation, quat);
  mat.setRotation(quat);
  mat.getRPY(r, p, y);
  EXPECT_LT(std::abs(r), 1e-3);
  EXPECT_LT(std::abs(p), 1e-3);
  EXPECT_LT(std::abs(y), 1e-3);

  resetFilter();
}

TEST(InterfacesTest, ImuAccBasicIO) {
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
  rclcpp::Rate loop_rate(50);
  for (size_t i = 0; i < 50; ++i) {
    imu.header.stamp = node_->now();
    imu2_pub_->publish(imu);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  EXPECT_LT(std::abs(filtered_.twist.twist.linear.x - 1.0), 0.40);
  EXPECT_LT(std::abs(filtered_.twist.twist.linear.y + 1.0), 0.40);
  EXPECT_LT(std::abs(filtered_.twist.twist.linear.z - 1.0), 0.40);

  imu.linear_acceleration.x = 0.0;
  imu.linear_acceleration.y = 0.0;
  imu.linear_acceleration.z = 0.0;

  // Now move for another second, and see where we end up
  for (size_t i = 0; i < 50; ++i) {
    imu.header.stamp = node_->now();
    imu2_pub_->publish(imu);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  EXPECT_LT(std::abs(filtered_.pose.pose.position.x - 1.8), 0.6);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.y + 1.8), 0.6);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.z - 1.8), 0.6);

  resetFilter();

  state_updated_ = false;

  // Test to see if the linear acceleration data is ignored when we set the
  // first covariance value to -1
  sensor_msgs::msg::Imu imuIgnore;
  imuIgnore.linear_acceleration.x = 1000;
  imuIgnore.linear_acceleration.y = 1000;
  imuIgnore.linear_acceleration.z = 1000;
  imuIgnore.linear_acceleration_covariance[0] = -1;

  for (size_t i = 0; i < 50; ++i) {
    imuIgnore.header.stamp = node_->now();
    imu2_pub_->publish(imuIgnore);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
    EXPECT_FALSE(state_updated_);
  }

  EXPECT_LT(std::abs(filtered_.pose.pose.position.x), 1e-3);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.y), 1e-3);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.z), 1e-3);

  resetFilter();
}

TEST(InterfacesTest, OdomDifferentialIO) {
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
    odom1_pub_->publish(odom);
    rclcpp::spin_some(node_);

    EXPECT_LT(std::abs(filtered_.pose.pose.position.x), 0.01);
    EXPECT_LT(std::abs(filtered_.pose.pose.position.y), 0.01);
    EXPECT_LT(std::abs(filtered_.pose.pose.position.z), 0.01);
    EXPECT_LT(std::abs(filtered_.pose.pose.orientation.x), 0.01);
    EXPECT_LT(std::abs(filtered_.pose.pose.orientation.y), 0.01);
    EXPECT_LT(std::abs(filtered_.pose.pose.orientation.z), 0.01);
    EXPECT_LT(std::abs(filtered_.pose.pose.orientation.w - 1), 0.01);
    rclcpp::Rate(10).sleep();
  }

  for (size_t ind = 0; ind < 36; ind += 7) {
    odom.pose.covariance[ind] = 1e-6;
  }

  // Slowly move the position, and hope that the differential position keeps up
  rclcpp::Rate loop_rate(20);
  for (size_t i = 0; i < 100; ++i) {
    odom.pose.pose.position.x += 0.01;
    odom.pose.pose.position.y += 0.02;
    odom.pose.pose.position.z -= 0.03;

    odom.header.stamp = node_->now();
    odom1_pub_->publish(odom);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  EXPECT_LT(std::abs(filtered_.pose.pose.position.x - 1), 0.2);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.y - 2), 0.4);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.z + 3), 0.6);

  resetFilter();
}

TEST(InterfacesTest, PoseDifferentialIO) {
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

  // No guaranteeing that the zero state we're expecting to see here isn't just a result of zeroing
  // it out previously, so check 10 times in succession.
  size_t zeroCount = 0;
  while (zeroCount++ < 10) {
    pose.header.stamp = node_->now();
    pose1_pub_->publish(pose);
    rclcpp::spin_some(node_);

    EXPECT_LT(std::abs(filtered_.pose.pose.position.x), 0.01);
    EXPECT_LT(std::abs(filtered_.pose.pose.position.y), 0.01);
    EXPECT_LT(std::abs(filtered_.pose.pose.position.z), 0.01);
    EXPECT_LT(std::abs(filtered_.pose.pose.orientation.x), 0.01);
    EXPECT_LT(std::abs(filtered_.pose.pose.orientation.y), 0.01);
    EXPECT_LT(std::abs(filtered_.pose.pose.orientation.z), 0.01);
    EXPECT_LT(std::abs(filtered_.pose.pose.orientation.w - 1), 0.01);

    rclcpp::Rate(10).sleep();
  }

  for (size_t ind = 0; ind < 36; ind += 7) {
    pose.pose.covariance[ind] = 1e-6;
  }

  // Issue this location repeatedly, and see if we get a final reported position of (1, 2, -3)
  rclcpp::Rate loop_rate(20);
  for (size_t i = 0; i < 100; ++i) {
    pose.pose.pose.position.x += 0.01;
    pose.pose.pose.position.y += 0.02;
    pose.pose.pose.position.z -= 0.03;

    pose.header.stamp = node_->now();
    pose1_pub_->publish(pose);
    loop_rate.sleep();
    rclcpp::spin_some(node_);
  }

  EXPECT_LT(std::abs(filtered_.pose.pose.position.x - 1), 0.2);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.y - 2), 0.4);
  EXPECT_LT(std::abs(filtered_.pose.pose.position.z + 3), 0.6);

  resetFilter();
}

TEST(InterfacesTest, ImuDifferentialIO) {
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
    imu0_pub_->publish(imu);  // Use this to move the absolute orientation
    imu1_pub_->publish(imu);  // Use this to keep velocities at 0
    rclcpp::spin_some(node_);
    rclcpp::Rate(10).sleep();
  }

  size_t zeroCount = 0;
  while (zeroCount++ < 10) {
    imu.header.stamp = node_->now();
    imu3_pub_->publish(imu);
    rclcpp::spin_some(node_);
    rclcpp::Rate(10).sleep();
  }

  double rollFinal = roll;
  double pitchFinal = pitch;
  double yawFinal = yaw;

  // Move the orientation slowly, and see if we can push it to 0
  rclcpp::Rate loop_rate(20);
  for (size_t i = 0; i < 100; ++i) {
    yawFinal -= 0.01 * (3.0 * M_PI / 4.0);

    quat.setRPY(rollFinal, pitchFinal, yawFinal);

    imu.orientation = tf2::toMsg(quat);
    imu.header.stamp = node_->now();
    imu3_pub_->publish(imu);
    rclcpp::spin_some(node_);

    loop_rate.sleep();
  }

  // Move the orientation slowly, and see if we can push it to 0
  for (size_t i = 0; i < 100; ++i) {
    rollFinal += 0.01 * (M_PI / 2.0);

    quat.setRPY(rollFinal, pitchFinal, yawFinal);

    imu.orientation = tf2::toMsg(quat);
    imu.header.stamp = node_->now();
    imu3_pub_->publish(imu);
    rclcpp::spin_some(node_);

    loop_rate.sleep();
  }

  tf2::fromMsg(filtered_.pose.pose.orientation, quat);
  tf2::Matrix3x3 mat(quat);
  mat.getRPY(rollFinal, pitchFinal, yawFinal);

  EXPECT_LT(std::abs(rollFinal), 0.2);
  EXPECT_LT(std::abs(pitchFinal), 0.2);
  EXPECT_LT(std::abs(yawFinal), 0.2);

  resetFilter();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  node_ = rclcpp::Node::make_shared("test_se_node_interfaces");

  odom0_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
    "odom_input0", rclcpp::SensorDataQoS());
  odom1_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
    "odom_input1", rclcpp::SensorDataQoS());
  odom2_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
    "odom_input2", rclcpp::SensorDataQoS());

  pose0_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose_input0", rclcpp::SensorDataQoS());
  pose1_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose_input1", rclcpp::SensorDataQoS());

  twist0_pub_ = node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_input0", rclcpp::SensorDataQoS());

  imu0_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
    "imu_input0", rclcpp::SensorDataQoS());
  imu1_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
    "imu_input1", rclcpp::SensorDataQoS());
  imu2_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
    "imu_input2", rclcpp::SensorDataQoS());
  imu3_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
    "imu_input3", rclcpp::SensorDataQoS());

  auto filtered_sub = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", rclcpp::QoS(1), filterCallback);

  reset_client_ = node_->create_client<std_srvs::srv::Empty>("reset");

  if (!reset_client_->wait_for_service(10s)) {
    RCLCPP_ERROR(node_->get_logger(), "Reset service not available after waiting");
    return 1;
  }

  int attempts = 0;
  while (
    odom0_pub_->get_subscription_count() == 0 ||
    odom1_pub_->get_subscription_count() == 0 ||
    odom2_pub_->get_subscription_count() == 0 ||
    pose0_pub_->get_subscription_count() == 0 ||
    pose1_pub_->get_subscription_count() == 0 ||
    twist0_pub_->get_subscription_count() == 0 ||
    imu0_pub_->get_subscription_count() == 0 ||
    imu1_pub_->get_subscription_count() == 0 ||
    imu2_pub_->get_subscription_count() == 0 ||
    imu3_pub_->get_subscription_count() == 0 ||
    filtered_sub->get_publisher_count() == 0)
  {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(),
      *(node_->get_clock()),
      1.0,
      "Waiting for all publishers and subscriptions to start...");

    rclcpp::Rate(1.0).sleep();

    if (++attempts == 10) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Could not establish necessary publishers and subscribers.");
      return 1;
    }
  }

  int ret = RUN_ALL_TESTS();

  odom0_pub_.reset();
  odom1_pub_.reset();
  odom2_pub_.reset();
  pose0_pub_.reset();
  pose1_pub_.reset();
  twist0_pub_.reset();
  imu0_pub_.reset();
  imu1_pub_.reset();
  imu2_pub_.reset();
  imu3_pub_.reset();
  filtered_sub.reset();
  reset_client_.reset();
  node_.reset();

  rclcpp::shutdown();
  return ret;
}
