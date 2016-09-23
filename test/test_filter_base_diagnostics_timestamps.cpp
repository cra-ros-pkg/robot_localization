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

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>

#include "robot_localization/filter_base.h"
#include "robot_localization/filter_common.h"
#include "robot_localization/SetPose.h"


#include <diagnostic_msgs/DiagnosticArray.h>

#include <gtest/gtest.h>

#include <vector>

namespace RobotLocalization
{

/*
  Convenience functions to get valid messages.
*/

geometry_msgs::PoseWithCovarianceStamped getValidPose()
{
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.frame_id = "base_link";
  pose_msg.pose.pose.position.x = 1;
  pose_msg.pose.pose.orientation.w = 1;
  for (size_t i = 0; i < 6 ; i++)
  {
    pose_msg.pose.covariance[i*6 + i] = 1;
  }
  return pose_msg;
}

geometry_msgs::TwistWithCovarianceStamped getValidTwist()
{
  geometry_msgs::TwistWithCovarianceStamped twist_msg;
  twist_msg.header.frame_id = "base_link";
  for (size_t i = 0; i < 6 ; i++)
  {
    twist_msg.twist.covariance[i*6 + i] = 1;
  }
  return twist_msg;
}


sensor_msgs::Imu getValidImu()
{
  sensor_msgs::Imu imu_msg;
  imu_msg.header.frame_id = "base_link";
  imu_msg.orientation.w = 1;
  for (size_t i = 0; i < 3 ; i++)
  {
    imu_msg.orientation_covariance[i * 3 + i] = 1;
    imu_msg.angular_velocity_covariance[i * 3 + i] = 1;
    imu_msg.linear_acceleration_covariance[i * 3 + i] = 1;
  }
  return imu_msg;
}

nav_msgs::Odometry getValidOdometry()
{
  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";
  odom_msg.pose = getValidPose().pose;
  odom_msg.twist = getValidTwist().twist;
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
  ros::Publisher odom_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher twist_pub_;
  ros::Publisher imu_pub_;

  geometry_msgs::PoseWithCovarianceStamped pose_msg_;
  geometry_msgs::TwistWithCovarianceStamped twist_msg_;
  nav_msgs::Odometry odom_msg_;
  sensor_msgs::Imu imu_msg_;

  ros::Subscriber diagnostic_sub_;
  ros::ServiceClient set_pose_;

 public:
  std::vector< diagnostic_msgs::DiagnosticArray > diagnostics;

  DiagnosticsHelper()
  {
    ros::NodeHandle nh;
    ros::NodeHandle nhLocal("~");

    // ready the valid messages.
    pose_msg_ = getValidPose();
    twist_msg_ = getValidTwist();
    odom_msg_ = getValidOdometry();
    imu_msg_ = getValidImu();

    // subscribe to diagnostics and create publishers for the odometry messages.
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("example/odom", 10);
    pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("example/pose", 10);
    twist_pub_ = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("example/twist", 10);
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("example/imu/data", 10);

    diagnostic_sub_ = nh.subscribe("/diagnostics", 10, &DiagnosticsHelper::diagnosticCallback, this);
    set_pose_ = nh.serviceClient<robot_localization::SetPose>("/set_pose");
  }

  void diagnosticCallback(const diagnostic_msgs::DiagnosticArrayPtr &msg)
  {
    diagnostics.push_back(*msg);
  }

  void publishMessages(ros::Time t)
  {
    odom_msg_.header.stamp = t;
    odom_msg_.header.seq++;
    odom_pub_.publish(odom_msg_);

    pose_msg_.header.stamp = t;
    pose_msg_.header.seq++;
    pose_pub_.publish(pose_msg_);

    twist_msg_.header.stamp = t;
    twist_msg_.header.seq++;
    twist_pub_.publish(twist_msg_);

    imu_msg_.header.stamp = t;
    imu_msg_.header.seq++;
    imu_pub_.publish(imu_msg_);
  }

  void setPose(ros::Time t)
  {
    robot_localization::SetPose pose_;
    pose_.request.pose = getValidPose();
    pose_.request.pose.header.stamp = t;
    set_pose_.call(pose_);
  }
};

}  // namespace RobotLocalization

/*
  First test; we run for a bit; then send messagse with an empty timestamp.
  Then we check if the diagnostics showed a warning.
*/
TEST(FilterBaseDiagnosticsTest, EmptyTimestamps)
{
  RobotLocalization::DiagnosticsHelper dh_;

  // keep track of which diagnostic messages are detected.
  bool received_warning_imu = false;
  bool received_warning_odom = false;
  bool received_warning_twist = false;
  bool received_warning_pose = false;

  // For about a second, send correct messages.
  ros::Rate loopRate(10);
  for (size_t i = 0; i < 10; ++i)
  {
    ros::spinOnce();
    dh_.publishMessages(ros::Time::now());
    loopRate.sleep();
  }

  ros::spinOnce();

  // create an empty timestamp and send all messages with this empty timestamp.
  ros::Time empty;
  empty.fromSec(0);

  dh_.publishMessages(empty);

  ros::spinOnce();

  // The filter runs and sends the diagnostics every second.
  // Just run this for two seconds to ensure we get all the diagnostic message.
  for (size_t i = 0; i < 20; ++i)
  {
    ros::spinOnce();
    loopRate.sleep();
  }

  /*
    Now the diagnostic messages have to be investigated to see whether they contain our warning.
  */
  for (size_t i=0; i < dh_.diagnostics.size(); i++)
  {
    for (size_t status_index=0; status_index < dh_.diagnostics[i].status.size(); status_index++)
    {
      for (size_t key=0; key < dh_.diagnostics[i].status[status_index].values.size(); key++)
      {
        diagnostic_msgs::KeyValue kv = dh_.diagnostics[i].status[status_index].values[key];
        // Now the keys can be checked to see whether we found our warning.

        if (kv.key == "imu0_timestamp")
        {
          received_warning_imu = true;
        }
        if (kv.key == "odom0_timestamp")
        {
          received_warning_odom = true;
        }
        if (kv.key == "twist0_timestamp")
        {
          received_warning_twist = true;
        }
        if (kv.key == "pose0_timestamp")
        {
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

TEST(FilterBaseDiagnosticsTest, TimestampsBeforeSetPose)
{
  RobotLocalization::DiagnosticsHelper dh_;

  // keep track of which diagnostic messages are detected.
  bool received_warning_imu = false;
  bool received_warning_odom = false;
  bool received_warning_twist = false;
  bool received_warning_pose = false;

  // For about a second, send correct messages.
  ros::Rate loopRate(10);
  for (size_t i = 0; i < 10; ++i)
  {
    ros::spinOnce();
    dh_.publishMessages(ros::Time::now());
    loopRate.sleep();
  }
  ros::spinOnce();

  // Set the pose to the current timestamp.
  dh_.setPose(ros::Time::now());
  ros::spinOnce();

  // send messages from one second before that pose reset.
  dh_.publishMessages(ros::Time::now() - ros::Duration(1));

  // The filter runs and sends the diagnostics every second.
  // Just run this for two seconds to ensure we get all the diagnostic message.
  for (size_t i = 0; i < 20; ++i)
  {
    ros::spinOnce();
    loopRate.sleep();
  }

  /*
    Now the diagnostic messages have to be investigated to see whether they contain our warning.
  */
  for (size_t i=0; i < dh_.diagnostics.size(); i++)
  {
    for (size_t status_index=0; status_index < dh_.diagnostics[i].status.size(); status_index++)
    {
      for (size_t key=0; key < dh_.diagnostics[i].status[status_index].values.size(); key++)
      {
        diagnostic_msgs::KeyValue kv = dh_.diagnostics[i].status[status_index].values[key];
        // Now the keys can be checked to see whether we found our warning.

        if (kv.key == "imu0_timestamp")
        {
          received_warning_imu = true;
        }
        if (kv.key == "odom0_timestamp")
        {
          received_warning_odom = true;
        }
        if (kv.key == "twist0_timestamp")
        {
          received_warning_twist = true;
        }
        if (kv.key == "pose0_timestamp")
        {
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

TEST(FilterBaseDiagnosticsTest, TimestampsBeforePrevious)
{
  RobotLocalization::DiagnosticsHelper dh_;

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
  ros::Rate loopRate(10);
  for (size_t i = 0; i < 20; ++i)
  {
    ros::spinOnce();
    dh_.publishMessages(ros::Time::now());
    loopRate.sleep();
  }
  ros::spinOnce();

  // Send message that is one second in the past.
  dh_.publishMessages(ros::Time::now() - ros::Duration(1));

  // The filter runs and sends the diagnostics every second.
  // Just run this for two seconds to ensure we get all the diagnostic message.
  for (size_t i = 0; i < 20; ++i)
  {
    ros::spinOnce();
    loopRate.sleep();
  }

  /*
    Now the diagnostic messages have to be investigated to see whether they contain our warning.
  */
  for (size_t i=0; i < dh_.diagnostics.size(); i++)
  {
    for (size_t status_index=0; status_index < dh_.diagnostics[i].status.size(); status_index++)
    {
      for (size_t key=0; key < dh_.diagnostics[i].status[status_index].values.size(); key++)
      {
        diagnostic_msgs::KeyValue kv = dh_.diagnostics[i].status[status_index].values[key];
        // Now the keys can be checked to see whether we found our warning.

        if (kv.key == "imu0_acceleration_timestamp")
        {
          received_warning_imu_accel = true;
        }
        if (kv.key == "imu0_pose_timestamp")
        {
          received_warning_imu_pose = true;
        }
        if (kv.key == "imu0_twist_timestamp")
        {
          received_warning_imu_twist = true;
        }

        if (kv.key == "odom0_twist_timestamp")
        {
          received_warning_twist = true;
        }

        if (kv.key == "pose0_timestamp")
        {
          received_warning_pose = true;
        }
        if (kv.key == "twist0_timestamp")
        {
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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "filter_base_diagnostics_timestamps-test-interfaces");
  ros::Time::init();

  // Give ekf_localization_node time to initialize
  ros::Duration(2.0).sleep();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
