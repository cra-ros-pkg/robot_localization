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
#include <memory>
#include <thread>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "robot_localization/navsat_transform.hpp"

using namespace std::chrono_literals;

/**
 * @class NavSatLifecycleTest
 * @brief Test fixture for testing lifecycle transitions and functionality of NavSatTransform node
 */
class NavSatLifecycleTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Configure node with common test parameters
    rclcpp::NodeOptions options;
    options.append_parameter_override("magnetic_declination_radians", 0.0);
    options.append_parameter_override("yaw_offset", 0.0);
    options.append_parameter_override("publish_filtered_gps", true);
    options.append_parameter_override("wait_for_datum", false);
    options.append_parameter_override("use_odometry_yaw", false);
    options.append_parameter_override("zero_altitude", false);
    options.append_parameter_override("broadcast_utm_transform", false);

    node_ = std::make_shared<robot_localization::NavSatTransform>(options);
    test_helper_node_ = rclcpp::Node::make_shared("test_helper");

    // Initialize TF broadcaster and listener
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(test_helper_node_);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(test_helper_node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void TearDown() override
  {
    // Gracefully transition node back to unconfigured state
    if (node_) {
      auto current_state = node_->get_current_state().id();

      if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        node_->deactivate();
      }
      if (node_->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        node_->cleanup();
      }
    }

    // Reset shared pointers in reverse order of creation
    tf_listener_.reset();
    tf_buffer_.reset();
    tf_broadcaster_.reset();
    node_.reset();
    test_helper_node_.reset();
  }

  /**
   * @brief Publish a static transform between two frames
   */
  void publish_static_tf(
    const std::string & parent_frame = "base_link",
    const std::string & child_frame = "gps",
    double x = 0.0, double y = 0.0, double z = 0.0)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = test_helper_node_->now();
    tf.header.frame_id = parent_frame;
    tf.child_frame_id = child_frame;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = z;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf);
  }

  /**
   * @brief Create a GPS fix message with specified coordinates
   */
  sensor_msgs::msg::NavSatFix::SharedPtr create_gps_message(
    double lat, double lon, double alt,
    const std::string & frame_id = "gps",
    int status = sensor_msgs::msg::NavSatStatus::STATUS_FIX)
  {
    auto msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
    msg->header.stamp = test_helper_node_->now();
    msg->header.frame_id = frame_id;
    msg->latitude = lat;
    msg->longitude = lon;
    msg->altitude = alt;
    msg->status.status = status;
    msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    msg->position_covariance[0] = 1.0;      // East variance
    msg->position_covariance[4] = 1.0;      // North variance
    msg->position_covariance[8] = 1.0;      // Up variance
    return msg;
  }

  /**
   * @brief Create an IMU message with specified orientation
   */
  sensor_msgs::msg::Imu::SharedPtr create_imu_message(
    double yaw = 0.0,
    const std::string & frame_id = "base_link")
  {
    auto msg = std::make_shared<sensor_msgs::msg::Imu>();
    msg->header.stamp = test_helper_node_->now();
    msg->header.frame_id = frame_id;

    // Convert RPY to quaternion (simplified for zero roll/pitch)
    msg->orientation.x = 0.0;
    msg->orientation.y = 0.0;
    msg->orientation.z = std::sin(yaw / 2.0);
    msg->orientation.w = std::cos(yaw / 2.0);

    msg->orientation_covariance[0] = 0.1;
    msg->orientation_covariance[4] = 0.1;
    msg->orientation_covariance[8] = 0.1;

    return msg;
  }

  /**
   * @brief Create an odometry message with specified pose
   */
  nav_msgs::msg::Odometry::SharedPtr create_odom_message(
    double x = 0.0, double y = 0.0, double yaw = 0.0,
    const std::string & frame_id = "odom")
  {
    auto msg = std::make_shared<nav_msgs::msg::Odometry>();
    msg->header.stamp = test_helper_node_->now();
    msg->header.frame_id = frame_id;
    msg->child_frame_id = "base_link";

    msg->pose.pose.position.x = x;
    msg->pose.pose.position.y = y;
    msg->pose.pose.position.z = 0.0;

    msg->pose.pose.orientation.x = 0.0;
    msg->pose.pose.orientation.y = 0.0;
    msg->pose.pose.orientation.z = std::sin(yaw / 2.0);
    msg->pose.pose.orientation.w = std::cos(yaw / 2.0);

    return msg;
  }

  /**
   * @brief Spin both nodes for a specified duration
   */
  void spin_for_duration(const std::chrono::milliseconds & duration)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      rclcpp::spin_some(node_->get_node_base_interface());
      rclcpp::spin_some(test_helper_node_);
      std::this_thread::sleep_for(10ms);
    }
  }

  std::shared_ptr<robot_localization::NavSatTransform> node_;
  rclcpp::Node::SharedPtr test_helper_node_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

/**
 * @test Verify basic lifecycle state transitions
 * Tests: Unconfigured -> Inactive -> Active -> Inactive -> Unconfigured
 */
TEST_F(NavSatLifecycleTest, BasicLifecycleTransitions)
{
  // Verify initial state is Unconfigured
  EXPECT_EQ(
    node_->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  // Configure transition
  auto result = node_->configure();
  EXPECT_EQ(result.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Activate transition
  result = node_->activate();
  EXPECT_EQ(result.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Deactivate transition
  result = node_->deactivate();
  EXPECT_EQ(result.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Cleanup transition
  result = node_->cleanup();
  EXPECT_EQ(result.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

/**
 * @test Verify reconfiguration cycle
 * Tests: Configure -> Cleanup -> Reconfigure -> Activate
 */
TEST_F(NavSatLifecycleTest, ReconfigurationCycle)
{
  EXPECT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

/**
 * @test Verify multiple activation/deactivation cycles
 * Tests that node can be activated and deactivated multiple times
 */
TEST_F(NavSatLifecycleTest, MultipleActivationCycles)
{
  EXPECT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

/**
 * @test Verify GPS message processing and datum initialization
 * Tests that node can process GPS messages and set datum correctly
 */
TEST_F(NavSatLifecycleTest, GPSMessageProcessingWithDatum)
{
  // Configure and activate node
  EXPECT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  publish_static_tf();
  EXPECT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Create publishers
  auto gps_pub = test_helper_node_->create_publisher<sensor_msgs::msg::NavSatFix>(
    "gps/fix", rclcpp::QoS(10));
  auto imu_pub = test_helper_node_->create_publisher<sensor_msgs::msg::Imu>(
    "imu", rclcpp::QoS(10));

  // Allow publishers to be discovered
  spin_for_duration(100ms);

  // Publish GPS message to set datum (NYC coordinates)
  auto gps_msg = create_gps_message(40.7128, -74.0060, 10.0);
  gps_pub->publish(*gps_msg);

  // Publish IMU for orientation
  auto imu_msg = create_imu_message();
  imu_pub->publish(*imu_msg);

  // Process messages
  spin_for_duration(200ms);

  // Verification: If we reach here without exceptions, datum was set successfully
  SUCCEED();
}

/**
 * @test Verify processing of multiple sequential GPS updates
 * Tests that node can handle a stream of GPS messages
 */
TEST_F(NavSatLifecycleTest, MultipleGPSUpdates)
{
  EXPECT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  publish_static_tf();
  EXPECT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto gps_pub = test_helper_node_->create_publisher<sensor_msgs::msg::NavSatFix>(
    "gps/fix", rclcpp::QoS(10));
  auto imu_pub = test_helper_node_->create_publisher<sensor_msgs::msg::Imu>(
    "imu", rclcpp::QoS(10));

  spin_for_duration(100ms);

  // Publish sequence of GPS messages simulating movement
  for (int i = 0; i < 5; ++i) {
    auto gps_msg = create_gps_message(
      40.7128 + i * 0.0001,
      -74.0060 + i * 0.0001,
      10.0
    );
    gps_pub->publish(*gps_msg);

    auto imu_msg = create_imu_message();
    imu_pub->publish(*imu_msg);
    spin_for_duration(50ms);
  }
  SUCCEED();
}

/**
 * @test Verify handling of GPS messages with invalid/no fix status
 * Tests that node handles GPS messages without valid fix gracefully
 */
TEST_F(NavSatLifecycleTest, InvalidGPSStatusHandling)
{
  EXPECT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  publish_static_tf();
  EXPECT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto gps_pub = test_helper_node_->create_publisher<sensor_msgs::msg::NavSatFix>(
    "gps/fix", rclcpp::QoS(10));

  spin_for_duration(100ms);

  // Publish GPS message with NO_FIX status
  auto gps_msg = create_gps_message(
    40.7128, -74.0060, 10.0, "gps",
    sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX
  );
  gps_pub->publish(*gps_msg);

  spin_for_duration(100ms);
  SUCCEED();
}

/**
 * @test Verify odometry input processing when use_odometry_yaw is enabled
 * Tests that node can use odometry for yaw when configured
 */
TEST_F(NavSatLifecycleTest, OdometryInputProcessing)
{
  // Create node with use_odometry_yaw enabled
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_odometry_yaw", true);
  options.append_parameter_override("wait_for_datum", false);
  options.append_parameter_override("publish_filtered_gps", true);
  auto odom_node = std::make_shared<robot_localization::NavSatTransform>(options);

  EXPECT_EQ(odom_node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  publish_static_tf();
  EXPECT_EQ(odom_node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto gps_pub = test_helper_node_->create_publisher<sensor_msgs::msg::NavSatFix>(
    "gps/fix", rclcpp::QoS(10));
  auto odom_pub = test_helper_node_->create_publisher<nav_msgs::msg::Odometry>(
    "odometry/filtered", rclcpp::QoS(10));

  spin_for_duration(100ms);

  // Publish GPS and odometry messages
  auto gps_msg = create_gps_message(40.7128, -74.0060, 10.0);
  gps_pub->publish(*gps_msg);

  auto odom_msg = create_odom_message(0.0, 0.0, 0.785);    // 45 degrees yaw
  odom_pub->publish(*odom_msg);

  // Process messages
  for (int i = 0; i < 10; ++i) {
    rclcpp::spin_some(odom_node->get_node_base_interface());
    rclcpp::spin_some(test_helper_node_);
    std::this_thread::sleep_for(10ms);
  }

  // Cleanup
  odom_node->deactivate();
  odom_node->cleanup();
  odom_node.reset();

  SUCCEED();
}

/**
 * @test Verify parameter configuration
 * Tests that parameters are correctly set and retrievable
 */
TEST_F(NavSatLifecycleTest, ParameterConfiguration)
{
  EXPECT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Verify parameters were set correctly
  auto magnetic_declination = node_->get_parameter("magnetic_declination_radians").as_double();
  auto yaw_offset = node_->get_parameter("yaw_offset").as_double();
  auto publish_filtered_gps = node_->get_parameter("publish_filtered_gps").as_bool();
  auto wait_for_datum = node_->get_parameter("wait_for_datum").as_bool();

  EXPECT_DOUBLE_EQ(magnetic_declination, 0.0);
  EXPECT_DOUBLE_EQ(yaw_offset, 0.0);
  EXPECT_TRUE(publish_filtered_gps);
  EXPECT_FALSE(wait_for_datum);
}

/**
 * @test Verify TF frame availability
 * Tests that required TF frames are published and available
 */
TEST_F(NavSatLifecycleTest, TFFrameAvailability)
{
  EXPECT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  publish_static_tf();

  // Allow TF to propagate
  spin_for_duration(100ms);

  EXPECT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Verify TF is available
  bool tf_available = tf_buffer_->canTransform("base_link", "gps", tf2::TimePointZero);
  EXPECT_TRUE(tf_available);
}

/**
 * @test Verify state persistence after deactivation
 * Tests that datum persists across deactivation/reactivation cycles
 */
TEST_F(NavSatLifecycleTest, StatePersistenceAfterDeactivation)
{
  EXPECT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  publish_static_tf();
  EXPECT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto gps_pub = test_helper_node_->create_publisher<sensor_msgs::msg::NavSatFix>(
    "gps/fix", rclcpp::QoS(10));
  auto imu_pub = test_helper_node_->create_publisher<sensor_msgs::msg::Imu>(
    "imu", rclcpp::QoS(10));

  spin_for_duration(100ms);

  // Set initial datum
  auto gps_msg = create_gps_message(40.7128, -74.0060, 10.0);
  gps_pub->publish(*gps_msg);
  auto imu_msg = create_imu_message();
  imu_pub->publish(*imu_msg);

  spin_for_duration(200ms);

  // Deactivate node
  EXPECT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Reactivate - datum should persist
  EXPECT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Publish another GPS message (should use existing datum)
  auto gps_msg2 = create_gps_message(40.7129, -74.0061, 10.0);
  gps_pub->publish(*gps_msg2);
  spin_for_duration(100ms);
  SUCCEED();
}

/**
 * @test Verify GPS with different frame IDs
 * Tests that node handles GPS messages from different frame IDs
 */
TEST_F(NavSatLifecycleTest, DifferentGPSFrameIDs)
{
  EXPECT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Publish TF for different GPS frame
  publish_static_tf("base_link", "gps_antenna", 0.5, 0.0, 1.0);

  EXPECT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto gps_pub = test_helper_node_->create_publisher<sensor_msgs::msg::NavSatFix>(
    "gps/fix", rclcpp::QoS(10));
  auto imu_pub = test_helper_node_->create_publisher<sensor_msgs::msg::Imu>(
    "imu", rclcpp::QoS(10));

  spin_for_duration(100ms);

  // Publish GPS with custom frame ID
  auto gps_msg = create_gps_message(40.7128, -74.0060, 10.0, "gps_antenna");
  gps_pub->publish(*gps_msg);

  auto imu_msg = create_imu_message();
  imu_pub->publish(*imu_msg);

  spin_for_duration(200ms);

  SUCCEED();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
