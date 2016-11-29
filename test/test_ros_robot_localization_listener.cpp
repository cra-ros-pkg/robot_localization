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

#include "robot_localization/ros_robot_localization_listener.h"
#include "robot_localization/filter_common.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <ros/ros.h>

#include <gtest/gtest.h>

RobotLocalization::RosRobotLocalizationListener* g_listener;
ros::Publisher odom_pub;
ros::Publisher accel_pub;

TEST( LocalizationListenerTest, testGetStateBeforeReceivingMessages )
{
  Eigen::VectorXd state(RobotLocalization::STATE_SIZE);
  Eigen::MatrixXd covariance(RobotLocalization::STATE_SIZE,RobotLocalization::STATE_SIZE);

  ros::Time time(0);
  std::string base_frame("base_link");

  EXPECT_FALSE(g_listener->getState(time, base_frame, state, covariance));
}

TEST(LocalizationListenerTest, testGetStateOfBaseLink )
{
  Eigen::VectorXd state(RobotLocalization::STATE_SIZE);
  Eigen::MatrixXd covariance(RobotLocalization::STATE_SIZE,RobotLocalization::STATE_SIZE);

  double x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az;
  x = y = z = roll = pitch = yaw = vy = vz = vroll = vpitch = vyaw = ax = ay = az = 0.0;
  vx = 1.0;
  vroll = M_PI/4.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, 0);

  ros::Time time1(1000);
  ros::Time time2(1001);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = time1;
  odom_msg.header.frame_id = "map";
  odom_msg.child_frame_id = "base_link";
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.y = z;
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();

  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.linear.z = vz;
  odom_msg.twist.twist.angular.x = vroll;
  odom_msg.twist.twist.angular.y = vpitch;
  odom_msg.twist.twist.angular.z = vyaw;

  geometry_msgs::AccelWithCovarianceStamped accel_msg;
  accel_msg.header.stamp = time1;
  accel_msg.header.frame_id = "base_link";
  accel_msg.accel.accel.linear.x = ax;
  accel_msg.accel.accel.linear.y = ay;
  accel_msg.accel.accel.linear.z = az;
  accel_msg.accel.accel.angular.x = 0;
  accel_msg.accel.accel.angular.y = 0;
  accel_msg.accel.accel.angular.z = 0;

  EXPECT_EQ("/test/odometry",odom_pub.getTopic());
  EXPECT_EQ("/test/accel",accel_pub.getTopic());

  EXPECT_EQ(1, odom_pub.getNumSubscribers());
  EXPECT_EQ(1, accel_pub.getNumSubscribers());

  odom_pub.publish(odom_msg);
  accel_pub.publish(accel_msg);

  ROS_INFO_STREAM("Publishing odom:\n" << odom_msg.pose.pose << "\n" << odom_msg.twist.twist);
  ROS_INFO_STREAM("Publishing accel:\n" << accel_msg.accel.accel);

  ros::Duration(0.1).sleep();

  ros::spinOnce();

  std::string base_frame("base_link");

  EXPECT_TRUE(g_listener->getState(time1, base_frame, state, covariance));

  ROS_INFO_STREAM("\ngetState at time = " << time1 << " returned:\n" <<
                  " - state\n" <<
                  " x           y           z        roll       pitch         yaw          vx          vy          vz       vroll      vpitch        vyaw          ax          ay          az\n" <<
                  state << "\n"
                  " - covariance\n" <<
                  covariance
                  );

  state.setZero();
  covariance.setZero();

  g_listener->getState(time2, base_frame, state, covariance);

  ROS_INFO_STREAM("\ngetState at time = " << time2 << " returned:\n" <<
                  " - state\n" <<
                  " x           y           z        roll       pitch         yaw          vx          vy          vz       vroll      vpitch        vyaw          ax          ay          az\n" <<
                  state << "\n"
                  " - covariance\n" <<
                  covariance
                  );

  EXPECT_DOUBLE_EQ(1.0,state(RobotLocalization::StateMemberX));
  EXPECT_DOUBLE_EQ(0.0,state(RobotLocalization::StateMemberY));
  EXPECT_DOUBLE_EQ(0.0,state(RobotLocalization::StateMemberZ));

  EXPECT_FLOAT_EQ(M_PI/4,state(RobotLocalization::StateMemberRoll));
  EXPECT_FLOAT_EQ(0.0,state(RobotLocalization::StateMemberPitch));
  EXPECT_FLOAT_EQ(0.0,state(RobotLocalization::StateMemberYaw));

  EXPECT_DOUBLE_EQ(M_PI/4.0,state(RobotLocalization::StateMemberVroll));
  EXPECT_DOUBLE_EQ(0.0,state(RobotLocalization::StateMemberVpitch));
  EXPECT_DOUBLE_EQ(0.0,state(RobotLocalization::StateMemberVyaw));
}

TEST( LocalizationListenerTest, GetStateOfRelatedFrame )
{
  Eigen::VectorXd state(RobotLocalization::STATE_SIZE);
  Eigen::MatrixXd covariance(RobotLocalization::STATE_SIZE,RobotLocalization::STATE_SIZE);

  tf2_ros::StaticTransformBroadcaster transform_broadcaster;

  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "base_link";
  transformStamped.child_frame_id = "sensor";
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 1.0;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI/2); // Fixed axes rpy, so new z points in the original negative x-direction and y in the original negative z.
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  transform_broadcaster.sendTransform(transformStamped);

  ros::Duration(0.1).sleep();

  state.setZero();
  covariance.setZero();

  ros::Time time1(1000);
  ros::Time time2(1001);

  std::string sensor_frame("sensor");

  EXPECT_TRUE(g_listener->getState(time1, sensor_frame, state, covariance) );

  ROS_WARN_STREAM("\ngetState for " << sensor_frame << " at time = " << time1 << " returned:\n" <<
                  " - state\n" <<
                  " x           y           z        roll       pitch         yaw          vx          vy          vz       vroll      vpitch        vyaw          ax          ay          az\n" <<
                  state << "\n"
                  " - covariance\n" <<
                  covariance
                  );

  EXPECT_FLOAT_EQ(0.0,state(RobotLocalization::StateMemberX));
  EXPECT_FLOAT_EQ(1.0,state(RobotLocalization::StateMemberY));
  EXPECT_FLOAT_EQ(0.0,state(RobotLocalization::StateMemberZ));

  EXPECT_FLOAT_EQ(0.0,state(RobotLocalization::StateMemberRoll));
  EXPECT_FLOAT_EQ(0.0,state(RobotLocalization::StateMemberPitch));
  EXPECT_FLOAT_EQ(M_PI/2,state(RobotLocalization::StateMemberYaw));

  EXPECT_TRUE( 1e-12 > state(RobotLocalization::StateMemberVx));
  EXPECT_FLOAT_EQ(-1.0, state(RobotLocalization::StateMemberVy));
  EXPECT_FLOAT_EQ(M_PI/4.0,state(RobotLocalization::StateMemberVz));

  EXPECT_TRUE( 1e-12 > state(RobotLocalization::StateMemberVroll));
  EXPECT_FLOAT_EQ(-M_PI/4.0,state(RobotLocalization::StateMemberVpitch));
  EXPECT_FLOAT_EQ(0.0,state(RobotLocalization::StateMemberVyaw));

  EXPECT_TRUE(g_listener->getState(time2, sensor_frame, state, covariance));

  ROS_WARN_STREAM("\n\ngetState for " << sensor_frame << " at time = " << time2 << " returned:\n" <<
                  " - state\n" <<
                  " x           y           z        roll       pitch         yaw          vx          vy          vz       vroll      vpitch        vyaw          ax          ay          az\n" <<
                  state << "\n"
                  " - covariance\n" <<
                  covariance
                  );

  EXPECT_FLOAT_EQ(1.0,state(RobotLocalization::StateMemberX));
  EXPECT_FLOAT_EQ(sqrt(2)/2.0,state(RobotLocalization::StateMemberY));
  EXPECT_FLOAT_EQ(sqrt(2)/2.0,state(RobotLocalization::StateMemberZ));

  EXPECT_TRUE( 1e-12 > state(RobotLocalization::StateMemberRoll));
  EXPECT_TRUE( 1e-12 > fabs(-M_PI/4.0 - state(RobotLocalization::StateMemberPitch)));
  EXPECT_FLOAT_EQ(M_PI/2,state(RobotLocalization::StateMemberYaw));

  EXPECT_TRUE( 1e-12 > state(RobotLocalization::StateMemberVx));
  EXPECT_FLOAT_EQ(-1.0, state(RobotLocalization::StateMemberVy));
  EXPECT_FLOAT_EQ(M_PI/4, state(RobotLocalization::StateMemberVz));

  EXPECT_TRUE( 1e-12 > state(RobotLocalization::StateMemberVroll));
  EXPECT_FLOAT_EQ(-M_PI/4.0,state(RobotLocalization::StateMemberVpitch));
  EXPECT_FLOAT_EQ(0,state(RobotLocalization::StateMemberVyaw));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_robot_localization_estimator");

  g_listener = new RobotLocalization::RosRobotLocalizationListener("test");

  ros::NodeHandle nh;
  odom_pub = nh.advertise<nav_msgs::Odometry>("/test/odometry", 1);
  accel_pub = nh.advertise<geometry_msgs::AccelWithCovarianceStamped>("/test/accel", 1);

  testing::InitGoogleTest(&argc, argv);

  int res = RUN_ALL_TESTS();

  delete g_listener;

  return res;
}
