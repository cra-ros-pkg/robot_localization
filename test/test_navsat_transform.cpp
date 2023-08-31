/*
 * Copyright (c) 2021, Charles River Analytics, Inc.
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

#include "robot_localization/navsat_transform.h"
#include <robot_localization/SetDatum.h>
#include <robot_localization/ToLL.h>
#include <robot_localization/FromLL.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <gtest/gtest.h>

#include <string>

TEST(NavSatTransformUTMJumpTest, UtmTest)
{
  ros::NodeHandle nh;
  ros::ServiceClient set_datum_client = nh.serviceClient<robot_localization::SetDatum>("/datum");
  ros::ServiceClient from_ll_client = nh.serviceClient<robot_localization::FromLL>("/fromLL");

  EXPECT_TRUE(set_datum_client.waitForExistence(ros::Duration(5)));

  // Initialise the navsat_transform node to a UTM zone
  robot_localization::SetDatum set_datum_srv;
  set_datum_srv.request.geo_pose.position.latitude = 1;
  set_datum_srv.request.geo_pose.position.longitude = 4;
  set_datum_srv.request.geo_pose.orientation.w = 1;
  EXPECT_TRUE(set_datum_client.call(set_datum_srv));

  // Let the node figure out its transforms
  ros::Duration(0.2).sleep();

  // Request the GPS point of said point:
  robot_localization::FromLL from_ll_srv;
  from_ll_srv.request.ll_point.latitude = 10;
  from_ll_srv.request.ll_point.longitude = 4.5;
  EXPECT_TRUE(from_ll_client.call(from_ll_srv));
  auto initial_response = from_ll_srv.response;

  // Request GPS point also in that zone
  from_ll_srv.request.ll_point.longitude = 5.5;
  EXPECT_TRUE(from_ll_client.call(from_ll_srv));
  auto same_zone_response = from_ll_srv.response;

  // 1° of longitude is about 111 kilometers in length
  EXPECT_NEAR(initial_response.map_point.x, same_zone_response.map_point.x, 120e3);
  EXPECT_NEAR(initial_response.map_point.y, same_zone_response.map_point.y, 120e3);

  // Request GPS point from neighboring zone (zone crossing is at 6 degrees)
  from_ll_srv.request.ll_point.longitude = 6.5;
  from_ll_client.call(from_ll_srv);
  auto neighbour_zone_response = from_ll_srv.response;

  EXPECT_NEAR(initial_response.map_point.x, neighbour_zone_response.map_point.x, 2*120e3);
  EXPECT_NEAR(initial_response.map_point.y, neighbour_zone_response.map_point.y, 2*120e3);
}

TEST(NavSatTransformUTMJumpTest, UtmServiceTest)
{
  ros::NodeHandle nh;
  ros::ServiceClient set_zone_client = nh.serviceClient<robot_localization::SetUTMZone>("/setUTMZone");
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  EXPECT_TRUE(set_zone_client.waitForExistence(ros::Duration(5)));

  // Publish input topics
  ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("gps/fix", 1, true);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odometry/filtered", 1, true);

  sensor_msgs::NavSatFix gps_msg;
  gps_msg.header.frame_id = "base_link";
  gps_msg.header.stamp = ros::Time::now();
  gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  gps_msg.latitude = 36.0;
  gps_msg.longitude = 0.0;

  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "odom";
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.child_frame_id = "base_link";
  odom_msg.pose.pose.orientation.w = 1;

  // Initialise the navsat_transform node to a UTM zone
  robot_localization::SetUTMZone set_zone_srv;
  set_zone_srv.request.utm_zone = "30U";
  EXPECT_TRUE(set_zone_client.call(set_zone_srv));
  gps_msg.header.stamp = ros::Time::now();
  gps_pub.publish(gps_msg);
  odom_msg.header.stamp = ros::Time::now();
  odom_pub.publish(odom_msg);
  // Let the node figure out its transforms
  ros::Duration(0.3).sleep();

  // Record the initial map transform
  geometry_msgs::TransformStamped initial_transform;
  try
  {
    EXPECT_TRUE(tf_buffer.canTransform("utm", "odom", ros::Time::now(), ros::Duration(3.0)));
    initial_transform = tf_buffer.lookupTransform("utm", "odom", ros::Time::now());
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    FAIL();
  }

  // Set the zone to a neighboring zone
  set_zone_srv.request.utm_zone = "31U";
  EXPECT_TRUE(set_zone_client.call(set_zone_srv));
  tf_buffer.clear();
  gps_msg.header.stamp = ros::Time::now();
  gps_pub.publish(gps_msg);
  odom_msg.header.stamp = ros::Time::now();
  odom_pub.publish(odom_msg);
  // Let the node figure out its transforms
  ros::Duration(0.3).sleep();

  // Check if map transform has changed
  geometry_msgs::TransformStamped new_transform;
  try
  {
    EXPECT_TRUE(tf_buffer.canTransform("utm", "odom", ros::Time::now(), ros::Duration(3.0)));
    new_transform = tf_buffer.lookupTransform("utm", "odom", ros::Time::now());
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    FAIL();
  }

  // Check difference between initial and new transform
  EXPECT_GT(std::abs(initial_transform.transform.translation.x - new_transform.transform.translation.x), 1.0);
  EXPECT_GT(std::abs(initial_transform.transform.translation.x - new_transform.transform.translation.x), 1.0);
  EXPECT_GT(std::abs(initial_transform.transform.rotation.z - new_transform.transform.rotation.z), 0.02);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_navsat_transform");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
