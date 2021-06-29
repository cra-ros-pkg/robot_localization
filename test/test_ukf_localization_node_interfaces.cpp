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

#include "robot_localization/SetPose.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gtest/gtest.h>

#include <iostream>

nav_msgs::Odometry filtered_;

bool stateUpdated_;

void resetFilter()
{
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<robot_localization::SetPose>("/set_pose");

  robot_localization::SetPose setPose;
  setPose.request.pose.pose.pose.orientation.w = 1;
  setPose.request.pose.header.frame_id = "odom";
  for (size_t ind = 0; ind < 36; ind+=7)
  {
    setPose.request.pose.pose.covariance[ind] = 1e-6;
  }

  setPose.request.pose.header.stamp = ros::Time::now();
  client.call(setPose);
  setPose.request.pose.header.seq++;
  ros::spinOnce();
  ros::Duration(0.01).sleep();
  stateUpdated_ = false;

  double deltaX = 0.0;
  double deltaY = 0.0;
  double deltaZ = 0.0;

  while (!stateUpdated_ || ::sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ) > 0.1)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();

    deltaX = filtered_.pose.pose.position.x - setPose.request.pose.pose.pose.position.x;
    deltaY = filtered_.pose.pose.position.y - setPose.request.pose.pose.pose.position.y;
    deltaZ = filtered_.pose.pose.position.z - setPose.request.pose.pose.pose.position.z;
  }

  EXPECT_LT(::sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ), 0.1);
}

void filterCallback(const nav_msgs::OdometryConstPtr &msg)
{
  filtered_ = *msg;
  stateUpdated_ = true;
}

TEST(InterfacesTest, OdomPoseBasicIO)
{
  stateUpdated_ = false;

  ros::NodeHandle nh;
  ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("/odom_input0", 5);
  ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);

  nav_msgs::Odometry odom;
  odom.pose.pose.position.x = 20.0;
  odom.pose.pose.position.y = 10.0;
  odom.pose.pose.position.z = -40.0;

  odom.pose.covariance[0] = 2.0;
  odom.pose.covariance[7] = 2.0;
  odom.pose.covariance[14] = 2.0;

  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  ros::Rate loopRate(50);
  for (size_t i = 0; i < 50; ++i)
  {
    odom.header.stamp = ros::Time::now();
    odomPub.publish(odom);
    ros::spinOnce();

    loopRate.sleep();

    odom.header.seq++;
  }

  // Now check the values from the callback
  EXPECT_LT(::fabs(filtered_.pose.pose.position.x - odom.pose.pose.position.x), 0.01);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y), 0.01);  // Configuration for this variable for this sensor is false
  EXPECT_LT(::fabs(filtered_.pose.pose.position.z - odom.pose.pose.position.z), 0.01);

  EXPECT_LT(filtered_.pose.covariance[0], 0.5);
  EXPECT_LT(filtered_.pose.covariance[7], 0.25);  // Configuration for this variable for this sensor is false
  EXPECT_LT(filtered_.pose.covariance[14], 0.6);

  resetFilter();
}

TEST(InterfacesTest, OdomTwistBasicIO)
{
  ros::NodeHandle nh;
  ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("/odom_input2", 5);
  ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);

  nav_msgs::Odometry odom;
  odom.twist.twist.linear.x = 5.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = 0.0;

  for (size_t ind = 0; ind < 36; ind+=7)
  {
    odom.twist.covariance[ind] = 1e-6;
  }

  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  ros::Rate loopRate(20);
  for (size_t i = 0; i < 400; ++i)
  {
    odom.header.stamp = ros::Time::now();
    odomPub.publish(odom);
    ros::spinOnce();

    loopRate.sleep();

    odom.header.seq++;
  }
  ros::spinOnce();

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - odom.twist.twist.linear.x), 0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.x - 100.0), 2.0);

  resetFilter();

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.linear.y = 5.0;

  loopRate = ros::Rate(20);
  for (size_t i = 0; i < 200; ++i)
  {
    odom.header.stamp = ros::Time::now();
    odomPub.publish(odom);
    ros::spinOnce();

    loopRate.sleep();

    odom.header.seq++;
  }
  ros::spinOnce();

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.y - odom.twist.twist.linear.y), 0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y - 50.0), 1.0);

  resetFilter();

  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 5.0;

  loopRate = ros::Rate(20);
  for (size_t i = 0; i < 100; ++i)
  {
    odom.header.stamp = ros::Time::now();
    odomPub.publish(odom);
    ros::spinOnce();

    loopRate.sleep();

    odom.header.seq++;
  }
  ros::spinOnce();

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.z - odom.twist.twist.linear.z), 0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.z - 25.0), 1.0);

  resetFilter();

  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.linear.x = 1.0;
  odom.twist.twist.angular.z = (M_PI/2) / (100.0 * 0.05);

  loopRate = ros::Rate(20);
  for (size_t i = 0; i < 100; ++i)
  {
    odom.header.stamp = ros::Time::now();
    odomPub.publish(odom);
    ros::spinOnce();

    loopRate.sleep();

    odom.header.seq++;
  }
  ros::spinOnce();

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - odom.twist.twist.linear.x), 0.1);
  EXPECT_LT(::fabs(filtered_.twist.twist.angular.z - odom.twist.twist.angular.z), 0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.x - filtered_.pose.pose.position.y), 0.5);

  resetFilter();

  odom.twist.twist.linear.x = 0.0;
  odom.twist.twist.angular.z = 0.0;
  odom.twist.twist.angular.x = -(M_PI/2) / (100.0 * 0.05);

  // First, roll the vehicle on its side
  loopRate = ros::Rate(20);
  for (size_t i = 0; i < 100; ++i)
  {
    odom.header.stamp = ros::Time::now();
    odomPub.publish(odom);
    ros::spinOnce();

    loopRate.sleep();

    odom.header.seq++;
  }
  ros::spinOnce();

  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = (M_PI/2) / (100.0 * 0.05);

  // Now, pitch it down (positive pitch velocity in vehicle frame)
  loopRate = ros::Rate(20);
  for (size_t i = 0; i < 100; ++i)
  {
    odom.header.stamp = ros::Time::now();
    odomPub.publish(odom);
    ros::spinOnce();

    loopRate.sleep();

    odom.header.seq++;
  }
  ros::spinOnce();

  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.linear.x = 3.0;

  // We should now be on our side and facing -Y. Move forward in
  // the vehicle frame X direction, and make sure Y decreases in
  // the world frame.
  loopRate = ros::Rate(20);
  for (size_t i = 0; i < 100; ++i)
  {
    odom.header.stamp = ros::Time::now();
    odomPub.publish(odom);
    ros::spinOnce();

    loopRate.sleep();

    odom.header.seq++;
  }
  ros::spinOnce();

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - odom.twist.twist.linear.x), 0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y + 15), 1.0);

  resetFilter();
}

TEST(InterfacesTest, PoseBasicIO)
{
  ros::NodeHandle nh;
  ros::Publisher posePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_input0", 5);
  ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);

  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.pose.pose.position.x = 20.0;
  pose.pose.pose.position.y = 10.0;
  pose.pose.pose.position.z = -40.0;
  pose.pose.pose.orientation.x = 0;
  pose.pose.pose.orientation.y = 0;
  pose.pose.pose.orientation.z = 0;
  pose.pose.pose.orientation.w = 1;

  for (size_t ind = 0; ind < 36; ind+=7)
  {
    pose.pose.covariance[ind] = 1e-6;
  }

  pose.header.frame_id = "odom";

  ros::Rate loopRate = ros::Rate(50);
  for (size_t i = 0; i < 50; ++i)
  {
    pose.header.stamp = ros::Time::now();
    posePub.publish(pose);
    ros::spinOnce();

    loopRate.sleep();

    pose.header.seq++;
  }

  // Now check the values from the callback
  EXPECT_LT(::fabs(filtered_.pose.pose.position.x - pose.pose.pose.position.x), 0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y), 0.1);  // Configuration for this variable for this sensor is false
  EXPECT_LT(::fabs(filtered_.pose.pose.position.z - pose.pose.pose.position.z), 0.1);

  EXPECT_LT(filtered_.pose.covariance[0], 0.5);
  EXPECT_LT(filtered_.pose.covariance[7], 0.25);  // Configuration for this variable for this sensor is false
  EXPECT_LT(filtered_.pose.covariance[14], 0.5);

  resetFilter();
}

TEST(InterfacesTest, TwistBasicIO)
{
  ros::NodeHandle nh;
  ros::Publisher twistPub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/twist_input0", 5);
  ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);

  geometry_msgs::TwistWithCovarianceStamped twist;
  twist.twist.twist.linear.x = 5.0;
  twist.twist.twist.linear.y = 0;
  twist.twist.twist.linear.z = 0;
  twist.twist.twist.angular.x = 0;
  twist.twist.twist.angular.y = 0;
  twist.twist.twist.angular.z = 0;

  for (size_t ind = 0; ind < 36; ind+=7)
  {
    twist.twist.covariance[ind] = 1e-6;
  }

  twist.header.frame_id = "base_link";

  ros::Rate loopRate = ros::Rate(20);
  for (size_t i = 0; i < 400; ++i)
  {
    twist.header.stamp = ros::Time::now();
    twistPub.publish(twist);
    ros::spinOnce();

    loopRate.sleep();

    twist.header.seq++;
  }
  ros::spinOnce();

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - twist.twist.twist.linear.x), 0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.x - 100.0), 2.0);

  resetFilter();

  twist.twist.twist.linear.x = 0.0;
  twist.twist.twist.linear.y = 5.0;

  loopRate = ros::Rate(20);
  for (size_t i = 0; i < 200; ++i)
  {
    twist.header.stamp = ros::Time::now();
    twistPub.publish(twist);
    ros::spinOnce();

    loopRate.sleep();

    twist.header.seq++;
  }
  ros::spinOnce();

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.y - twist.twist.twist.linear.y), 0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y - 50.0), 1.0);

  resetFilter();

  twist.twist.twist.linear.y = 0.0;
  twist.twist.twist.linear.z = 5.0;

  loopRate = ros::Rate(20);
  for (size_t i = 0; i < 100; ++i)
  {
    twist.header.stamp = ros::Time::now();
    twistPub.publish(twist);
    ros::spinOnce();

    loopRate.sleep();

    twist.header.seq++;
  }
  ros::spinOnce();

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.z - twist.twist.twist.linear.z), 0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.z - 25.0), 1.0);

  resetFilter();

  twist.twist.twist.linear.z = 0.0;
  twist.twist.twist.linear.x = 1.0;
  twist.twist.twist.angular.z = (M_PI/2) / (100.0 * 0.05);

  loopRate = ros::Rate(20);
  for (size_t i = 0; i < 100; ++i)
  {
    twist.header.stamp = ros::Time::now();
    twistPub.publish(twist);
    ros::spinOnce();

    loopRate.sleep();

    twist.header.seq++;
  }
  ros::spinOnce();

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - twist.twist.twist.linear.x), 0.1);
  EXPECT_LT(::fabs(filtered_.twist.twist.angular.z - twist.twist.twist.angular.z), 0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.x - filtered_.pose.pose.position.y), 0.5);

  resetFilter();

  twist.twist.twist.linear.x = 0.0;
  twist.twist.twist.angular.z = 0.0;
  twist.twist.twist.angular.x = -(M_PI/2) / (100.0 * 0.05);

  // First, roll the vehicle on its side
  loopRate = ros::Rate(20);
  for (size_t i = 0; i < 100; ++i)
  {
    twist.header.stamp = ros::Time::now();
    twistPub.publish(twist);
    ros::spinOnce();

    loopRate.sleep();

    twist.header.seq++;
  }
  ros::spinOnce();

  twist.twist.twist.angular.x = 0.0;
  twist.twist.twist.angular.y = (M_PI/2) / (100.0 * 0.05);

  // Now, pitch it down (positive pitch velocity in vehicle frame)
  loopRate = ros::Rate(20);
  for (size_t i = 0; i < 100; ++i)
  {
    twist.header.stamp = ros::Time::now();
    twistPub.publish(twist);
    ros::spinOnce();

    loopRate.sleep();

    twist.header.seq++;
  }
  ros::spinOnce();

  twist.twist.twist.angular.y = 0.0;
  twist.twist.twist.linear.x = 3.0;

  // We should now be on our side and facing -Y. Move forward in
  // the vehicle frame X direction, and make sure Y decreases in
  // the world frame.
  loopRate = ros::Rate(20);
  for (size_t i = 0; i < 100; ++i)
  {
    twist.header.stamp = ros::Time::now();
    twistPub.publish(twist);
    ros::spinOnce();

    loopRate.sleep();

    twist.header.seq++;
  }
  ros::spinOnce();

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - twist.twist.twist.linear.x), 0.1);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y + 15), 1.0);

  resetFilter();
}

TEST(InterfacesTest, ImuPoseBasicIO)
{
  ros::NodeHandle nh;
  ros::Publisher imuPub = nh.advertise<sensor_msgs::Imu>("/imu_input0", 5);
  ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);

  sensor_msgs::Imu imu;
  tf2::Quaternion quat;
  quat.setRPY(M_PI/4, -M_PI/4, M_PI/2);
  imu.orientation = tf2::toMsg(quat);

  for (size_t ind = 0; ind < 9; ind+=4)
  {
    imu.orientation_covariance[ind] = 1e-6;
  }

  imu.header.frame_id = "base_link";

  // Make sure the pose reset worked. Test will timeout
  // if this fails.
  ros::Rate loopRate(50);
  for (size_t i = 0; i < 50; ++i)
  {
    imu.header.stamp = ros::Time::now();
    imuPub.publish(imu);
    ros::spinOnce();

    loopRate.sleep();

    imu.header.seq++;
  }

  // Now check the values from the callback
  tf2::fromMsg(filtered_.pose.pose.orientation, quat);
  tf2::Matrix3x3 mat(quat);
  double r, p, y;
  mat.getRPY(r, p, y);
  EXPECT_LT(::fabs(r - M_PI/4), 0.1);
  EXPECT_LT(::fabs(p + M_PI/4), 0.1);
  EXPECT_LT(::fabs(y - M_PI/2), 0.1);

  EXPECT_LT(filtered_.pose.covariance[21], 0.5);
  EXPECT_LT(filtered_.pose.covariance[28], 0.25);
  EXPECT_LT(filtered_.pose.covariance[35], 0.5);

  resetFilter();

  // Test to see if the orientation data is ignored when we set the
  // first covariance value to -1
  sensor_msgs::Imu imuIgnore;
  imuIgnore.orientation.x = 0.1;
  imuIgnore.orientation.y = 0.2;
  imuIgnore.orientation.z = 0.3;
  imuIgnore.orientation.w = 0.4;
  imuIgnore.orientation_covariance[0] = -1;

  loopRate = ros::Rate(50);
  for (size_t i = 0; i < 50; ++i)
  {
    imuIgnore.header.stamp = ros::Time::now();
    imuPub.publish(imuIgnore);
    loopRate.sleep();
    ros::spinOnce();

    imuIgnore.header.seq++;
  }

  tf2::fromMsg(filtered_.pose.pose.orientation, quat);
  mat.setRotation(quat);
  mat.getRPY(r, p, y);
  EXPECT_LT(::fabs(r), 1e-3);
  EXPECT_LT(::fabs(p), 1e-3);
  EXPECT_LT(::fabs(y), 1e-3);

  resetFilter();
}

TEST(InterfacesTest, ImuTwistBasicIO)
{
  ros::NodeHandle nh;
  ros::Publisher imuPub = nh.advertise<sensor_msgs::Imu>("/imu_input1", 5);
  ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);

  sensor_msgs::Imu imu;
  tf2::Quaternion quat;
  imu.angular_velocity.x = (M_PI / 2.0);

  for (size_t ind = 0; ind < 9; ind+=4)
  {
    imu.angular_velocity_covariance[ind] = 1e-6;
  }

  imu.header.frame_id = "base_link";

  ros::Rate loopRate(50);
  for (size_t i = 0; i < 50; ++i)
  {
    imu.header.stamp = ros::Time::now();
    imuPub.publish(imu);
    loopRate.sleep();
    ros::spinOnce();

    imu.header.seq++;
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

  resetFilter();

  imu.angular_velocity.x = 0.0;
  imu.angular_velocity.y = -(M_PI / 2.0);

  // Make sure the pose reset worked. Test will timeout
  // if this fails.
  loopRate = ros::Rate(50);
  for (size_t i = 0; i < 50; ++i)
  {
    imu.header.stamp = ros::Time::now();
    imuPub.publish(imu);
    loopRate.sleep();
    ros::spinOnce();

    imu.header.seq++;
  }

  // Now check the values from the callback
  tf2::fromMsg(filtered_.pose.pose.orientation, quat);

  tf2::Quaternion expected_quat(tf2::Vector3(0., 1., 0.), -M_PI/2.0);

  EXPECT_LT(std::abs(tf2Angle(expected_quat.getAxis(), quat.getAxis())), 0.1);
  EXPECT_LT(std::abs(expected_quat.getAngle() - quat.getAngle()), 0.7);

  EXPECT_LT(filtered_.twist.covariance[28], 1e-3);
  EXPECT_LT(filtered_.pose.covariance[28], 0.1);

  resetFilter();

  imu.angular_velocity.y = 0;
  imu.angular_velocity.z = M_PI / 4.0;

  // Make sure the pose reset worked. Test will timeout
  // if this fails.
  loopRate = ros::Rate(50);
  for (size_t i = 0; i < 50; ++i)
  {
    imu.header.stamp = ros::Time::now();
    imuPub.publish(imu);
    loopRate.sleep();
    ros::spinOnce();

    imu.header.seq++;
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

  resetFilter();

  // Test to see if the angular velocity data is ignored when we set the
  // first covariance value to -1
  sensor_msgs::Imu imuIgnore;
  imuIgnore.angular_velocity.x = 100;
  imuIgnore.angular_velocity.y = 100;
  imuIgnore.angular_velocity.z = 100;
  imuIgnore.angular_velocity_covariance[0] = -1;

  loopRate = ros::Rate(50);
  for (size_t i = 0; i < 50; ++i)
  {
    imuIgnore.header.stamp = ros::Time::now();
    imuPub.publish(imuIgnore);
    loopRate.sleep();
    ros::spinOnce();

    imuIgnore.header.seq++;
  }

  tf2::fromMsg(filtered_.pose.pose.orientation, quat);
  mat.setRotation(quat);
  mat.getRPY(r, p, y);
  EXPECT_LT(::fabs(r), 1e-3);
  EXPECT_LT(::fabs(p), 1e-3);
  EXPECT_LT(::fabs(y), 1e-3);

  resetFilter();
}

TEST(InterfacesTest, ImuAccBasicIO)
{
  ros::NodeHandle nh;
  ros::Publisher imuPub = nh.advertise<sensor_msgs::Imu>("/imu_input2", 5);
  ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);

  sensor_msgs::Imu imu;
  imu.header.frame_id = "base_link";
  imu.linear_acceleration_covariance[0] = 1e-6;
  imu.linear_acceleration_covariance[4] = 1e-6;
  imu.linear_acceleration_covariance[8] = 1e-6;

  imu.linear_acceleration.x = 1;
  imu.linear_acceleration.y = -1;
  imu.linear_acceleration.z = 1;

  // Move with constant acceleration for 1 second,
  // then check our velocity.
  ros::Rate loopRate(50);
  for (size_t i = 0; i < 50; ++i)
  {
    imu.header.stamp = ros::Time::now();
    imuPub.publish(imu);
    loopRate.sleep();
    ros::spinOnce();

    imu.header.seq++;
  }

  EXPECT_LT(::fabs(filtered_.twist.twist.linear.x - 1.0), 0.4);
  EXPECT_LT(::fabs(filtered_.twist.twist.linear.y + 1.0), 0.4);
  EXPECT_LT(::fabs(filtered_.twist.twist.linear.z - 1.0), 0.4);

  imu.linear_acceleration.x = 0.0;
  imu.linear_acceleration.y = 0.0;
  imu.linear_acceleration.z = 0.0;

  // Now move for another second, and see where we
  // end up
  loopRate = ros::Rate(50);
  for (size_t i = 0; i < 50; ++i)
  {
    imu.header.stamp = ros::Time::now();
    imuPub.publish(imu);
    loopRate.sleep();
    ros::spinOnce();

    imu.header.seq++;
  }

  EXPECT_LT(::fabs(filtered_.pose.pose.position.x - 1.2), 0.4);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y + 1.2), 0.4);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.z - 1.2), 0.4);

  resetFilter();

  // Test to see if the linear acceleration data is ignored when we set the
  // first covariance value to -1
  sensor_msgs::Imu imuIgnore;
  imuIgnore.linear_acceleration.x = 1000;
  imuIgnore.linear_acceleration.y = 1000;
  imuIgnore.linear_acceleration.z = 1000;
  imuIgnore.linear_acceleration_covariance[0] = -1;

  loopRate = ros::Rate(50);
  for (size_t i = 0; i < 50; ++i)
  {
    imuIgnore.header.stamp = ros::Time::now();
    imuPub.publish(imuIgnore);
    loopRate.sleep();
    ros::spinOnce();

    imuIgnore.header.seq++;
  }

  EXPECT_LT(::fabs(filtered_.pose.pose.position.x), 1e-3);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y), 1e-3);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.z), 1e-3);

  resetFilter();
}

TEST(InterfacesTest, OdomDifferentialIO)
{
  ros::NodeHandle nh;
  ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("/odom_input1", 5);
  ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);

  nav_msgs::Odometry odom;
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
  while (zeroCount++ < 10)
  {
    odom.header.stamp = ros::Time::now();
    odomPub.publish(odom);
    ros::spinOnce();

    EXPECT_LT(::fabs(filtered_.pose.pose.position.x), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.position.y), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.position.z), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.orientation.x), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.orientation.y), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.orientation.z), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.orientation.w - 1), 0.01);

    ros::Duration(0.1).sleep();

    odom.header.seq++;
  }

  for (size_t ind = 0; ind < 36; ind+=7)
  {
    odom.pose.covariance[ind] = 1e-6;
  }

  // Slowly move the position, and hope that the
  // differential position keeps up
  ros::Rate loopRate(20);
  for (size_t i = 0; i < 100; ++i)
  {
    odom.pose.pose.position.x += 0.01;
    odom.pose.pose.position.y += 0.02;
    odom.pose.pose.position.z -= 0.03;

    odom.header.stamp = ros::Time::now();
    odomPub.publish(odom);
    ros::spinOnce();

    loopRate.sleep();

    odom.header.seq++;
  }
  ros::spinOnce();

  EXPECT_LT(::fabs(filtered_.pose.pose.position.x - 1), 0.2);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y - 2), 0.4);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.z + 3), 0.6);

  resetFilter();
}

TEST(InterfacesTest, PoseDifferentialIO)
{
  ros::NodeHandle nh;
  ros::Publisher posePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_input1", 5);
  ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);

  geometry_msgs::PoseWithCovarianceStamped pose;
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
  while (zeroCount++ < 10)
  {
    pose.header.stamp = ros::Time::now();
    posePub.publish(pose);
    ros::spinOnce();

    EXPECT_LT(::fabs(filtered_.pose.pose.position.x), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.position.y), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.position.z), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.orientation.x), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.orientation.y), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.orientation.z), 0.01);
    EXPECT_LT(::fabs(filtered_.pose.pose.orientation.w - 1), 0.01);

    ros::Duration(0.1).sleep();

    pose.header.seq++;
  }

  // ...but only if we give the measurement a tiny covariance
  for (size_t ind = 0; ind < 36; ind+=7)
  {
    pose.pose.covariance[ind] = 1e-6;
  }

  // Issue this location repeatedly, and see if we get
  // a final reported position of (1, 2, -3)
  ros::Rate loopRate(20);
  for (size_t i = 0; i < 100; ++i)
  {
    pose.pose.pose.position.x += 0.01;
    pose.pose.pose.position.y += 0.02;
    pose.pose.pose.position.z -= 0.03;

    pose.header.stamp = ros::Time::now();
    posePub.publish(pose);
    ros::spinOnce();

    loopRate.sleep();

    pose.header.seq++;
  }
  ros::spinOnce();

  EXPECT_LT(::fabs(filtered_.pose.pose.position.x - 1), 0.2);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.y - 2), 0.4);
  EXPECT_LT(::fabs(filtered_.pose.pose.position.z + 3), 0.6);

  resetFilter();
}

TEST(InterfacesTest, ImuDifferentialIO)
{
  ros::NodeHandle nh;
  ros::Publisher imu0Pub = nh.advertise<sensor_msgs::Imu>("/imu_input0", 5);
  ros::Publisher imu1Pub = nh.advertise<sensor_msgs::Imu>("/imu_input1", 5);
  ros::Publisher imuPub = nh.advertise<sensor_msgs::Imu>("/imu_input3", 5);
  ros::Subscriber filteredSub = nh.subscribe("/odometry/filtered", 1, &filterCallback);

  sensor_msgs::Imu imu;
  imu.header.frame_id = "base_link";
  tf2::Quaternion quat;
  const double roll = M_PI/2.0;
  const double pitch = -M_PI;
  const double yaw = -M_PI/4.0;
  quat.setRPY(roll, pitch, yaw);
  imu.orientation = tf2::toMsg(quat);

  imu.orientation_covariance[0] = 0.02;
  imu.orientation_covariance[4] = 0.02;
  imu.orientation_covariance[8] = 0.02;

  imu.angular_velocity_covariance[0] = 0.02;
  imu.angular_velocity_covariance[4] = 0.02;
  imu.angular_velocity_covariance[8] = 0.02;

  ros::Duration(0.1).sleep();
  ros::spinOnce();

  size_t setCount = 0;
  while (setCount++ < 1000)
  {
    imu.header.stamp = ros::Time::now();
    imu0Pub.publish(imu);  // Use this to move the absolute orientation
    imu1Pub.publish(imu);  // Use this to keep velocities at 0
    ros::spinOnce();

    ros::Duration(0.001).sleep();

    imu.header.seq++;
  }

  size_t zeroCount = 0;
  while (zeroCount++ < 10)
  {
    imu.header.stamp = ros::Time::now();
    imuPub.publish(imu);
    ros::spinOnce();

    ros::Duration(0.1).sleep();

    imu.header.seq++;
  }

  double rollFinal = roll;
  double pitchFinal = pitch;
  double yawFinal = yaw;

  // Move the orientation slowly, and see if we
  // can push it to 0
  ros::Rate loopRate(20);
  for (size_t i = 0; i < 100; ++i)
  {
    yawFinal -= 0.01 * (3.0 * M_PI/4.0);

    quat.setRPY(rollFinal, pitchFinal, yawFinal);

    imu.orientation = tf2::toMsg(quat);
    imu.header.stamp = ros::Time::now();
    imuPub.publish(imu);
    ros::spinOnce();

    loopRate.sleep();

    imu.header.seq++;
  }
  ros::spinOnce();

  // Move the orientation slowly, and see if we
  // can push it to 0
  loopRate = ros::Rate(20);
  for (size_t i = 0; i < 100; ++i)
  {
    rollFinal += 0.01 * (M_PI/2.0);

    quat.setRPY(rollFinal, pitchFinal, yawFinal);

    imu.orientation = tf2::toMsg(quat);
    imu.header.stamp = ros::Time::now();
    imuPub.publish(imu);
    ros::spinOnce();

    loopRate.sleep();

    imu.header.seq++;
  }
  ros::spinOnce();

  tf2::fromMsg(filtered_.pose.pose.orientation, quat);
  tf2::Matrix3x3 mat(quat);
  mat.getRPY(rollFinal, pitchFinal, yawFinal);
  EXPECT_LT(::fabs(rollFinal), 0.2);
  EXPECT_LT(::fabs(pitchFinal), 0.2);
  EXPECT_LT(::fabs(yawFinal), 0.2);

  resetFilter();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "ukf_navigation_node-test-interfaces");
  ros::Time::init();

  // Give ukf_localization_node time to initialize
  ros::Duration(2.0).sleep();

  return RUN_ALL_TESTS();
}
