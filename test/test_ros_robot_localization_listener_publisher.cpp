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

#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <string>

#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_robot_localization_listener_publisher");

  ros::NodeHandle nh;
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odometry/filtered", 1);
  ros::Publisher accel_pub = nh.advertise<geometry_msgs::AccelWithCovarianceStamped>("accel/filtered", 1);
  tf2_ros::StaticTransformBroadcaster transform_broadcaster;

  ros::Time end_time = ros::Time::now() + ros::Duration(10);
  while (ros::ok() && ros::Time::now() < end_time)
  {
    ros::Time time1(1000);
    double x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az;
    x = y = z = roll = pitch = yaw = vy = vz = vroll = vpitch = vyaw = ax = ay = az = 0.0;
    vx = 1.0;
    vroll = M_PI/4.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);

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

    odom_pub.publish(odom_msg);
    accel_pub.publish(accel_msg);

    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "sensor";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 1.0;
    transformStamped.transform.translation.z = 0.0;
    {
      tf2::Quaternion q;
      q.setRPY(0, 0, M_PI/2);
      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();
      transformStamped.transform.rotation.w = q.w();

      transform_broadcaster.sendTransform(transformStamped);
    }

    ros::Duration(0.1).sleep();
  }

  return 0;
}
