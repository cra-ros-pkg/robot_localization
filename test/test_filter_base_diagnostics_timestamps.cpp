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

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "robot_localization/filter_base.hpp"
#include "robot_localization/filter_common.hpp"
#include "robot_localization/srv/set_pose.hpp"

namespace robot_localization
{

/*
  Convenience functions to get valid messages.
*/

geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr getValidPose()
{
  auto pose_msg =
    std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  pose_msg->header.frame_id = "base_link";
  pose_msg->pose.pose.position.x = 1;
  pose_msg->pose.pose.orientation.w = 1;
  for (size_t i = 0; i < 6; i++) {
    pose_msg->pose.covariance[i * 6 + i] = 1;
  }
  return pose_msg;
}

geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr getValidTwist()
{
  auto twist_msg =
    std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
  twist_msg->header.frame_id = "base_link";
  for (size_t i = 0; i < 6; i++) {
    twist_msg->twist.covariance[i * 6 + i] = 1;
  }
  return twist_msg;
}

sensor_msgs::msg::Imu::SharedPtr getValidImu()
{
  auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
  imu_msg->header.frame_id = "base_link";
  imu_msg->orientation.w = 1;
  for (size_t i = 0; i < 3; i++) {
    imu_msg->orientation_covariance[i * 3 + i] = 1;
    imu_msg->angular_velocity_covariance[i * 3 + i] = 1;
    imu_msg->linear_acceleration_covariance[i * 3 + i] = 1;
  }
  return imu_msg;
}

nav_msgs::msg::Odometry::SharedPtr getValidOdometry()
{
  auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
  odom_msg->header.frame_id = "odom";
  odom_msg->child_frame_id = "base_link";
  odom_msg->pose = getValidPose()->pose;
  odom_msg->twist = getValidTwist()->twist;
  return odom_msg;
}

/*
  Helper class to handle the setup and message publishing for the testcases.
  It provides convenience to send valid messages with a specified timestamp.
  All diagnostic messages are stored into the public diagnostics attribute.
*/
class DiagnosticsHelper
{
private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    twist_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> pose_msg_;
  std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> twist_msg_;
  std::shared_ptr<nav_msgs::msg::Odometry> odom_msg_;
  std::shared_ptr<sensor_msgs::msg::Imu> imu_msg_;

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostic_sub_;
  rclcpp::Client<robot_localization::srv::SetPose>::SharedPtr set_pose_;

public:
  std::vector<diagnostic_msgs::msg::DiagnosticArray> diagnostics;
  rclcpp::Node::SharedPtr node_;

  DiagnosticsHelper()
  {
    node_ = rclcpp::Node::make_shared("test_filter_base_diagnostics");

    // ready the valid messages.
    pose_msg_ = getValidPose();
    twist_msg_ = getValidTwist();
    odom_msg_ = getValidOdometry();
    imu_msg_ = getValidImu();

    // Create a publisher with a custom Quality of Service profile.
    // subscribe to diagnostics and create publishers for the odometry messages.
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      "/example/odom", rclcpp::SensorDataQoS());
    pose_pub_ =
      node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/example/pose", rclcpp::SensorDataQoS());
    twist_pub_ =
      node_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/example/twist", rclcpp::SensorDataQoS());
    imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
      "/example/imu/data", rclcpp::SensorDataQoS());

    rclcpp::SystemDefaultsQoS qos = rclcpp::SystemDefaultsQoS();
    diagnostic_sub_ =
      node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", qos.keep_all(),
      [&](diagnostic_msgs::msg::DiagnosticArray::UniquePtr msg) {
        diagnostics.push_back(*msg);
      });

    set_pose_ =
      node_->create_client<robot_localization::srv::SetPose>("set_pose");
  }

  void publishMessages(rclcpp::Time t)
  {
    odom_msg_->header.stamp = t;
    odom_pub_->publish(*odom_msg_);

    pose_msg_->header.stamp = t;
    pose_pub_->publish(*pose_msg_);

    twist_msg_->header.stamp = t;
    twist_pub_->publish(*twist_msg_);

    imu_msg_->header.stamp = t;
    imu_pub_->publish(*imu_msg_);
  }

  void setPose(rclcpp::Time t)
  {
    auto setPoseRequest =
      std::make_shared<robot_localization::srv::SetPose::Request>();
    setPoseRequest->pose.header.frame_id = "base_link";
    setPoseRequest->pose.pose = getValidPose()->pose;
    setPoseRequest->pose.header.stamp = t;
    set_pose_->async_send_request(setPoseRequest);
  }
};

}  // namespace robot_localization

/*
  First test; we run for a bit; then send messagse with an empty timestamp.
  Then we check if the diagnostics showed a warning.
*/
TEST(FilterBaseDiagnosticsTest, EmptyTimestamps) {
  robot_localization::DiagnosticsHelper dh_;

  // keep track of which diagnostic messages are detected.
  bool received_warning_imu = false;
  bool received_warning_odom = false;
  bool received_warning_twist = false;
  bool received_warning_pose = false;

  // For about a second, send correct messages.
  rclcpp::Rate loopRate(10);
  for (size_t i = 0; i < 10; ++i) {
    rclcpp::spin_some(dh_.node_);
    dh_.publishMessages((dh_.node_)->now());
    loopRate.sleep();
  }

  rclcpp::spin_some(dh_.node_);

  // create an empty timestamp and send all messages with this empty timestamp.
  static uint32_t empty_sec = 0;
  builtin_interfaces::msg::Time msg;
  msg.sec = empty_sec;
  msg.nanosec = empty_sec;
  rclcpp::Time empty = msg;

  dh_.publishMessages(empty);
  rclcpp::spin_some(dh_.node_);

  // The filter runs and sends the diagnostics every second.
  // Just run this for two seconds to ensure we get all the diagnostic message.
  for (size_t i = 0; i < 20; ++i) {
    rclcpp::spin_some(dh_.node_);
    loopRate.sleep();
  }

  /*
    Now the diagnostic messages have to be investigated to see whether they
    contain our warning.
  */
  for (size_t i = 0; i < dh_.diagnostics.size(); i++) {
    for (size_t status_index = 0;
      status_index < dh_.diagnostics[i].status.size(); status_index++)
    {
      for (size_t key = 0;
        key < dh_.diagnostics[i].status[status_index].values.size(); key++)
      {
        diagnostic_msgs::msg::KeyValue kv =
          dh_.diagnostics[i].status[status_index].values[key];
        // Now the keys can be checked to see whether we found our warning.
        if (kv.key == "imu0_timestamp") {
          received_warning_imu = true;
        }
        if (kv.key == "odom0_timestamp") {
          received_warning_odom = true;
        }
        if (kv.key == "twist0_timestamp") {
          received_warning_twist = true;
        }
        if (kv.key == "pose0_timestamp") {
          received_warning_pose = true;
        }
      }
    }
  }

  EXPECT_TRUE(received_warning_imu);
  EXPECT_TRUE(received_warning_odom);
  EXPECT_TRUE(received_warning_twist);
  EXPECT_TRUE(received_warning_pose);
}

TEST(FilterBaseDiagnosticsTest, TimestampsBeforeSetPose) {
  robot_localization::DiagnosticsHelper dh_;

  // keep track of which diagnostic messages are detected.
  bool received_warning_imu = false;
  bool received_warning_odom = false;
  bool received_warning_twist = false;
  bool received_warning_pose = false;

  // For about a second, send correct messages.
  rclcpp::Rate loopRate(10);
  for (size_t i = 0; i < 10; ++i) {
    rclcpp::spin_some(dh_.node_);
    dh_.publishMessages((dh_.node_)->now());
    loopRate.sleep();
  }
  rclcpp::spin_some(dh_.node_);

  rclcpp::Time curr = (dh_.node_)->now();
  // Set the pose to the current timestamp.
  dh_.setPose(curr);

  rclcpp::spin_some(dh_.node_);

  // wait for 1 sec to make synchronize setPose msg & before msg
  sleep(1);
  // send messages from one second before that pose reset.
  dh_.publishMessages(curr - rclcpp::Duration(1, 0));

  // The filter runs and sends the diagnostics every second.
  // Just run this for two seconds to ensure we get all the diagnostic message.
  for (size_t i = 0; i < 20; ++i) {
    rclcpp::spin_some(dh_.node_);
    loopRate.sleep();
  }
  /*
    Now the diagnostic messages have to be investigated to see whether they
    contain our warning.
  */
  for (size_t i = 0; i < dh_.diagnostics.size(); i++) {
    for (size_t status_index = 0;
      status_index < dh_.diagnostics[i].status.size(); status_index++)
    {
      for (size_t key = 0;
        key < dh_.diagnostics[i].status[status_index].values.size(); key++)
      {
        diagnostic_msgs::msg::KeyValue kv =
          dh_.diagnostics[i].status[status_index].values[key];
        // Now the keys can be checked to see whether we found our warning.

        if (kv.key == "imu0_timestamp") {
          received_warning_imu = true;
        }
        if (kv.key == "odom0_timestamp") {
          received_warning_odom = true;
        }
        if (kv.key == "twist0_timestamp") {
          received_warning_twist = true;
        }
        if (kv.key == "pose0_timestamp") {
          received_warning_pose = true;
        }
      }
    }
  }
  EXPECT_TRUE(received_warning_imu);
  EXPECT_TRUE(received_warning_odom);
  EXPECT_TRUE(received_warning_twist);
  EXPECT_TRUE(received_warning_pose);
}

TEST(FilterBaseDiagnosticsTest, TimestampsBeforePrevious) {
  robot_localization::DiagnosticsHelper dh_;
  // keep track of which diagnostic messages are detected.
  // we have more things to check here because the messages get split over
  // various callbacks if they pass the check if they predate the set_pose time.
  bool received_warning_imu_accel = false;
  bool received_warning_imu_pose = false;
  bool received_warning_imu_twist = false;
  bool received_warning_odom_twist = false;
  bool received_warning_twist = false;
  bool received_warning_pose = false;

  // For two seconds send correct messages.
  rclcpp::Rate loopRate(20);
  for (size_t i = 0; i < 20; ++i) {
    rclcpp::spin_some(dh_.node_);
    dh_.publishMessages((dh_.node_)->now());
    loopRate.sleep();
  }
  rclcpp::spin_some(dh_.node_);

  // Send message that is one second in the past.
  dh_.publishMessages((dh_.node_)->now() - rclcpp::Duration(1, 0));

  // The filter runs and sends the diagnostics every second.
  // Just run this for two seconds to ensure we get all the diagnostic message.
  for (size_t i = 0; i < 20; ++i) {
    rclcpp::spin_some(dh_.node_);
    loopRate.sleep();
  }

  /*
    Now the diagnostic messages have to be investigated to see whether they
    contain our warning.
  */
  for (size_t i = 0; i < dh_.diagnostics.size(); i++) {
    for (size_t status_index = 0;
      status_index < dh_.diagnostics[i].status.size(); status_index++)
    {
      for (size_t key = 0;
        key < dh_.diagnostics[i].status[status_index].values.size(); key++)
      {
        diagnostic_msgs::msg::KeyValue kv =
          dh_.diagnostics[i].status[status_index].values[key];
        // Now the keys can be checked to see whether we found our warning.

        if (kv.key == "imu0_acceleration_timestamp") {
          received_warning_imu_accel = true;
        }
        if (kv.key == "imu0_pose_timestamp") {
          received_warning_imu_pose = true;
        }
        if (kv.key == "imu0_twist_timestamp") {
          received_warning_imu_twist = true;
        }

        if (kv.key == "odom0_twist_timestamp") {
          received_warning_twist = true;
        }

        if (kv.key == "pose0_timestamp") {
          received_warning_pose = true;
        }
        if (kv.key == "twist0_timestamp") {
          received_warning_odom_twist = true;
        }
      }
    }
  }

  EXPECT_TRUE(received_warning_imu_accel);
  EXPECT_TRUE(received_warning_imu_pose);
  EXPECT_TRUE(received_warning_imu_twist);
  EXPECT_TRUE(received_warning_odom_twist);
  EXPECT_TRUE(received_warning_pose);
  EXPECT_TRUE(received_warning_twist);
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
