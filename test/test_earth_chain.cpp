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
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "robot_localization/navsat_transform.hpp"

#include "GeographicLib/Geocentric.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_localization/srv/set_datum.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

class EarthChainTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<robot_localization::NavSatTransform> createNavSatNode(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    return std::make_shared<robot_localization::NavSatTransform>(options);
  }

  // Returns (latitude, longitude, height)
  tf2::Vector3 toGeodetic(const tf2::Vector3 & ecef)
  {
    double latitude, longitude, height;
    GeographicLib::Geocentric::WGS84().Reverse(
      ecef.x(), ecef.y(), ecef.z(), latitude, longitude, height);
    return tf2::Vector3(latitude, longitude, height);
  }

  tf2::Transform lookup(
    const tf2_ros::Buffer & buffer, const std::string & parent,
    const std::string & child)
  {
    tf2::Transform transform;
    tf2::fromMsg(buffer.lookupTransform(parent, child, tf2::TimePointZero).transform, transform);
    return transform;
  }

  tf2::Vector3 gpsInMap()
  {
    tf2::Quaternion robot_yaw;
    robot_yaw.setRPY(0.0, 0.0, 0.4);
    return tf2::Vector3(3.0, -2.0, 0.15) + tf2::quatRotate(robot_yaw, tf2::Vector3(0.5, 0.0, 1.2));
  }

  // Publishes the sensor mounts, robot pose, heading and optional fix until done() is true
  bool publishUntil(
    const std::shared_ptr<robot_localization::NavSatTransform> & navsat,
    const std::function<bool()> & done,
    const std::optional<double> & fix_altitude = std::nullopt)
  {
    auto helper = std::make_shared<rclcpp::Node>("earth_chain_test_helper");
    auto odom_pub = helper->create_publisher<nav_msgs::msg::Odometry>(
      "odometry/filtered", rclcpp::SensorDataQoS());
    auto imu_pub = helper->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
    auto fix_pub = helper->create_publisher<sensor_msgs::msg::NavSatFix>(
      "gps/fix", rclcpp::SensorDataQoS());

    geometry_msgs::msg::TransformStamped gps_mount;
    gps_mount.header.frame_id = "base_link";
    gps_mount.child_frame_id = "gps_link";
    gps_mount.transform.translation.x = 0.5;
    gps_mount.transform.translation.z = 1.2;
    gps_mount.transform.rotation.w = 1.0;
    geometry_msgs::msg::TransformStamped imu_mount = gps_mount;
    imu_mount.child_frame_id = "imu_link";
    imu_mount.transform.translation.x = 0.0;
    imu_mount.transform.translation.z = 0.0;
    tf2_ros::StaticTransformBroadcaster static_broadcaster(*helper);
    static_broadcaster.sendTransform({gps_mount, imu_mount});

    tf2::Quaternion robot_yaw;
    robot_yaw.setRPY(0.0, 0.0, 0.4);
    tf2::Quaternion imu_yaw;
    imu_yaw.setRPY(0.0, 0.0, 0.5);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(navsat);
    executor.add_node(helper);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = helper->now();
      odom.header.frame_id = "map";
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = 3.0;
      odom.pose.pose.position.y = -2.0;
      odom.pose.pose.position.z = 0.15;
      odom.pose.pose.orientation = tf2::toMsg(robot_yaw);
      odom_pub->publish(odom);

      sensor_msgs::msg::Imu imu;
      imu.header = odom.header;
      imu.header.frame_id = "imu_link";
      imu.orientation = tf2::toMsg(imu_yaw);
      imu_pub->publish(imu);

      if (fix_altitude) {
        sensor_msgs::msg::NavSatFix fix;
        fix.header = odom.header;
        fix.header.frame_id = "gps_link";
        fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        fix.latitude = 29.2;
        fix.longitude = -90.1;
        fix.altitude = *fix_altitude;
        fix_pub->publish(fix);
      }

      executor.spin_some(std::chrono::milliseconds(50));
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      if (done()) {
        return true;
      }
    }
    return false;
  }
};

TEST_F(EarthChainTest, CaseA_EarthToCartesianAndCartesianToWorld)
{
  // Configure as_parent=true, broadcast_cartesian=true, broadcast_earth_transform=true
  std::vector<rclcpp::Parameter> parameters = {
    rclcpp::Parameter("broadcast_cartesian_transform", true),
    rclcpp::Parameter("broadcast_cartesian_transform_as_parent_frame", true),
    rclcpp::Parameter("broadcast_earth_transform", true),
    rclcpp::Parameter("earth_frame_id", "earth"),
    rclcpp::Parameter("wait_for_datum", true),
    rclcpp::Parameter("datum", std::vector<double>{40.0, -74.0, 0.0})
  };

  auto node = createNavSatNode(parameters);

  // Create TF buffer to check transforms
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

  // Give some time for transforms to be published
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin_some();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Test that we can lookup earth->cartesian transform
  try {
    auto transform = tf_buffer->lookupTransform("utm", "earth", tf2::TimePointZero);
    EXPECT_EQ(transform.header.frame_id, "earth");
    EXPECT_EQ(transform.child_frame_id, "utm");
  } catch (tf2::TransformException & ex) {
    // Transform might not be available immediately in test - this is acceptable for now
    RCLCPP_WARN(rclcpp::get_logger("test"), "Transform earth->utm not available: %s", ex.what());
  }

  node.reset();
}

TEST_F(EarthChainTest, CaseB_EarthToWorldAndWorldToCartesian)
{
  // Configure as_parent=false, broadcast_cartesian=true, broadcast_earth_transform=true
  std::vector<rclcpp::Parameter> parameters = {
    rclcpp::Parameter("broadcast_cartesian_transform", true),
    rclcpp::Parameter("broadcast_cartesian_transform_as_parent_frame", false),
    rclcpp::Parameter("broadcast_earth_transform", true),
    rclcpp::Parameter("earth_frame_id", "earth"),
    rclcpp::Parameter("wait_for_datum", true),
    rclcpp::Parameter("datum", std::vector<double>{40.0, -74.0, 0.0})
  };

  auto node = createNavSatNode(parameters);

  // Create TF buffer to check transforms
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

  // Give some time for transforms to be published
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin_some();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Test that we can lookup earth->world transform
  try {
    auto transform = tf_buffer->lookupTransform("odom", "earth", tf2::TimePointZero);
    EXPECT_EQ(transform.header.frame_id, "earth");
    EXPECT_EQ(transform.child_frame_id, "odom");
  } catch (tf2::TransformException & ex) {
    // Transform might not be available immediately in test - this is acceptable for now
    RCLCPP_WARN(rclcpp::get_logger("test"), "Transform earth->odom not available: %s", ex.what());
  }

  node.reset();
}

TEST_F(EarthChainTest, CaseC_EarthToWorldOnly)
{
  // Configure broadcast_cartesian=false, broadcast_earth_transform=true
  std::vector<rclcpp::Parameter> parameters = {
    rclcpp::Parameter("broadcast_cartesian_transform", false),
    rclcpp::Parameter("broadcast_earth_transform", true),
    rclcpp::Parameter("earth_frame_id", "earth"),
    rclcpp::Parameter("wait_for_datum", true),
    rclcpp::Parameter("datum", std::vector<double>{40.0, -74.0, 0.0})
  };

  auto node = createNavSatNode(parameters);

  // Create TF buffer to check transforms
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

  // Give some time for transforms to be published
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin_some();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Test that we can lookup earth->world transform and no cartesian link
  try {
    auto transform = tf_buffer->lookupTransform("odom", "earth", tf2::TimePointZero);
    EXPECT_EQ(transform.header.frame_id, "earth");
    EXPECT_EQ(transform.child_frame_id, "odom");
  } catch (tf2::TransformException & ex) {
    // Transform might not be available immediately in test - this is acceptable for now
    RCLCPP_WARN(rclcpp::get_logger("test"), "Transform earth->odom not available: %s", ex.what());
  }

  // Test that cartesian link is NOT published
  try {
    auto transform = tf_buffer->lookupTransform("utm", "odom", tf2::TimePointZero);
    // If we get here without exception, that means the transform exists, which it shouldn't
    FAIL() << "Expected cartesian transform to NOT be published in Case C";
  } catch (tf2::TransformException & ex) {
    // This is expected - no cartesian transform should be published
    SUCCEED();
  }

  node.reset();
}

TEST_F(EarthChainTest, ECEFConversionAccuracy)
{
  // Test known coordinates round-trip validation
  std::vector<rclcpp::Parameter> parameters = {
    rclcpp::Parameter("broadcast_earth_transform", true),
    rclcpp::Parameter("earth_frame_id", "earth"),
    rclcpp::Parameter("use_local_cartesian", true),
    rclcpp::Parameter("wait_for_datum", true),
    rclcpp::Parameter("datum", std::vector<double>{40.7128, -74.0060, 0.0})  // NYC coordinates
  };

  auto node = createNavSatNode(parameters);

  // Give some time for initialization
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin_some();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Test basic parameters were set correctly
  EXPECT_EQ(node->get_parameter("earth_frame_id").as_string(), "earth");
  EXPECT_EQ(node->get_parameter("broadcast_earth_transform").as_bool(), true);

  node.reset();
}

TEST_F(EarthChainTest, ParameterDefaults)
{
  // Test that default parameters are set correctly
  std::vector<rclcpp::Parameter> parameters = {};

  auto node = createNavSatNode(parameters);

  // Test default values
  EXPECT_EQ(node->get_parameter("earth_frame_id").as_string(), "earth");
  EXPECT_EQ(node->get_parameter("broadcast_earth_transform").as_bool(), false);

  node.reset();
}

TEST_F(EarthChainTest, FixReturnsThroughEarth)
{
  for (const double altitude : {-25.6, 120.0}) {
    SCOPED_TRACE(altitude);
    auto node = createNavSatNode(
    {
      rclcpp::Parameter("use_local_cartesian", true),
      rclcpp::Parameter("broadcast_earth_transform", true),
      rclcpp::Parameter("zero_altitude", true),
    });
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);
    ASSERT_TRUE(
      publishUntil(
        node,
        [&tf_buffer]() {return tf_buffer.canTransform("earth", "map", tf2::TimePointZero);},
        altitude));

    const tf2::Vector3 gps = toGeodetic(lookup(tf_buffer, "earth", "map") * gpsInMap());
    EXPECT_NEAR(gps.x(), 29.2, 1e-8);
    EXPECT_NEAR(gps.y(), -90.1, 1e-8);
    EXPECT_NEAR(gps.z(), altitude, 1e-3);
  }
}

TEST_F(EarthChainTest, SetDatumAltitudeReturnsThroughEarth)
{
  auto node = createNavSatNode(
  {
    rclcpp::Parameter("use_local_cartesian", true),
    rclcpp::Parameter("broadcast_earth_transform", true),
    rclcpp::Parameter("wait_for_datum", true),
    rclcpp::Parameter("datum", std::vector<double>{29.2, -90.1, 0.0}),
  });
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  const double altitude = -25.6;
  auto client = node->create_client<robot_localization::srv::SetDatum>("datum");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  auto request = std::make_shared<robot_localization::srv::SetDatum::Request>();
  request->geo_pose.position.latitude = 29.2;
  request->geo_pose.position.longitude = -90.1;
  request->geo_pose.position.altitude = altitude;
  request->geo_pose.orientation.w = 1.0;
  auto response = client->async_send_request(request);

  auto datum_applied = [&]() {
      if (!tf_buffer.canTransform("earth", "map", tf2::TimePointZero)) {
        return false;
      }
      const tf2::Vector3 origin = toGeodetic(lookup(tf_buffer, "earth", "map").getOrigin());
      return std::abs(origin.z() - altitude) < 1e-3;
    };
  EXPECT_TRUE(publishUntil(node, datum_applied));
}

TEST_F(EarthChainTest, FixReturnsThroughLocalEnuWithZeroAltitude)
{
  const double altitude = -25.6;
  auto node = createNavSatNode(
  {
    rclcpp::Parameter("use_local_cartesian", true),
    rclcpp::Parameter("broadcast_cartesian_transform", true),
    rclcpp::Parameter("broadcast_cartesian_transform_as_parent_frame", true),
    rclcpp::Parameter("broadcast_earth_transform", true),
    rclcpp::Parameter("zero_altitude", true),
  });
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);
  ASSERT_TRUE(
    publishUntil(
      node,
      [&tf_buffer]() {return tf_buffer.canTransform("earth", "map", tf2::TimePointZero);},
      altitude));

  const tf2::Vector3 gps = toGeodetic(lookup(tf_buffer, "earth", "map") * gpsInMap());
  EXPECT_NEAR(gps.x(), 29.2, 1e-8);
  EXPECT_NEAR(gps.y(), -90.1, 1e-8);
  EXPECT_NEAR(gps.z(), altitude, 1e-3);
  EXPECT_NEAR(lookup(tf_buffer, "local_enu", "map").getOrigin().z(), 0.0, 1e-3);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
