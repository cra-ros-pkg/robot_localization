/*
 * Copyright (c) 2014, Charles River Analytics, Inc.
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
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//! @brief Used for calculating the utm_frame->odom_frame transform
nav_msgs::Odometry *latestUtmMsg_;
nav_msgs::Odometry *latestOdomMessage_;
sensor_msgs::Imu *latestImuMsg_;

//! @brief TF listener used for converting
tf::TransformListener *tfListener_;

//! @brief The utm_frame->odom_frame transform object
tf::StampedTransform *odomUtmTransform_;

//! @brief Used for updating the transform so as to
//! remove altitude from the UTM measurements.
tf::Pose originalOdomPose_;

//! @brief Used for updating the transform so as to
//! remove altitude from the UTM measurements.
tf::Pose originalUtmPose_;

//! @brief Parameter that specifies the magnetic decliation for the robot's
//! environment.
double magneticDeclination_;

//! @brief Stores the roll we need to compute the transform
double utmOdomTfRoll_;

//! @brief Stores the pitch we need to compute the transform
double utmOdomTfPitch_;

//! @brief Stores the yaw we need to compute the transform
double utmOdomTfYaw_;

//! @brief Whether or not the GPS fix is usable
bool hasFix_;

//! @brief Whether or not we've computed a good heading
bool transformGood_;

//! @brief On level ground, your IMU should read 0 roll. If
//! it doesn't, this (parameterized) value gives the offset
double rollOffset_;

//! @brief On level ground, your IMU should read 0 pitch. If
//! it doesn't, this (parameterized) value gives the offset
double pitchOffset_;

//! @brief Your IMU should read 0 when facing *magnetic* north. If it
//! doesn't, this (parameterized) value gives the offset (NOTE: if you
//! have a magenetic declination, use the parameter setting for that).
double yawOffset_;

//! @brief If this parameter is true, we continually update the transform
//! using the current altitude from the UTM message. This allows users to
//! receive a (nearly) zero Z measurement when they use the transform.
bool zeroAltitude_;

//! @brief Computes the transform from the odometry message's frame_id
//! to the UTM message's frame_id
//!
//! The transform is computed from the odom frame to the UTM frame so that
//! we don't accidentally attempt to give a second parent to the odom frame
//! in the tf tree. Users may already have a parent for that frame_id, and
//! we don't want to stomp on it.
//!
void computeOdomUtmTransform()
{
  // Only do this if:
  // 1. We haven't computed the odom_frame->utm_frame transform before
  // 2. We've received the messages we need already
  // 3. We have good GPS data
  if(!transformGood_ &&
     latestUtmMsg_ != NULL &&
     latestImuMsg_ != NULL &&
     latestOdomMessage_ != NULL &&
     hasFix_)
  {
    if(!std::isnan(latestUtmMsg_->pose.pose.position.x) &&
       !std::isnan(latestUtmMsg_->pose.pose.position.y) &&
       !std::isnan(latestUtmMsg_->pose.pose.position.z))
    {
      ROS_INFO_STREAM("Computing initial " << latestOdomMessage_->header.frame_id << "->" << latestUtmMsg_->header.frame_id << " transform");

      // Now that we have what we need, create the transform so it will be broadcast
      if(odomUtmTransform_ == NULL)
      {
        odomUtmTransform_ = new tf::StampedTransform();
      }

      // Get the IMU's current RPY values. Need the raw values (for yaw, anyway).
      tf::Matrix3x3 mat(tf::Quaternion(latestImuMsg_->orientation.x,
                                       latestImuMsg_->orientation.y,
                                       latestImuMsg_->orientation.z,
                                       latestImuMsg_->orientation.w));

      // Convert to RPY
      mat.getRPY(utmOdomTfRoll_, utmOdomTfPitch_, utmOdomTfYaw_);

      ROS_INFO_STREAM("Latest IMU orientation was: (" << std::fixed << utmOdomTfRoll_ << ", " << utmOdomTfPitch_ << ", " << utmOdomTfYaw_ << ")");

      // Compute the final yaw value that corrects for the difference between the
      // IMU's heading and the UTM grid's belief of where 0 heading should be (i.e.,
      // along the x-axis)
      utmOdomTfYaw_ += (magneticDeclination_ + yawOffset_ + (M_PI / 2.0));
      utmOdomTfPitch_ += pitchOffset_;
      utmOdomTfRoll_ += rollOffset_;

      ROS_INFO_STREAM("Corrected for magnetic declination of " << std::fixed << magneticDeclination_ <<
                      ", user-specified offset of " << yawOffset_ << ", and fixed offset of " << (M_PI / 2.0) <<
                      ". Transform heading factor is now " << utmOdomTfYaw_);

      // Convert to tf-friendly structures
      tf::Quaternion quat;
      quat.setRPY(utmOdomTfRoll_, utmOdomTfPitch_, utmOdomTfYaw_);
      tf::Vector3 pos;
      tf::pointMsgToTF(latestUtmMsg_->pose.pose.position, pos);

      // Put the transform together
      odomUtmTransform_->frame_id_ = latestOdomMessage_->header.frame_id;
      odomUtmTransform_->child_frame_id_ = latestUtmMsg_->header.frame_id;
      odomUtmTransform_->setRotation(quat);
      odomUtmTransform_->setOrigin(pos);

      ROS_INFO_STREAM("Before correcttion, " << odomUtmTransform_->frame_id_  << "->" <<
                      odomUtmTransform_->child_frame_id_ << " transform is: " << std::fixed <<
                      "\nPosition: (" << odomUtmTransform_->getOrigin().getX() << ", " <<
                                         odomUtmTransform_->getOrigin().getY() << ", " <<
                                         odomUtmTransform_->getOrigin().getZ() << ")" <<
                      "\nOrientation: (" << utmOdomTfRoll_ << ", " <<
                                            utmOdomTfPitch_ << ", " <<
                                            utmOdomTfYaw_ << ")");

      // If we started in a location without GPS (or with poor GPS), then we could be at some non-zero
      // (x, y) location in the odomFrame_ frame. For that reason, we need to move this transform to the
      // origin. To do that, we need to figure out what our odometry origin (the inverse
      // of our position) is in the UTM frame.

      // Convert the pose to a tf object
      tf::Pose odomPose;
      tf::poseMsgToTF(latestOdomMessage_->pose.pose, odomPose);

      // The transform order will be orig_odom_pos * orig_utm_pos_inverse * cur_utm_pos
      // Store these values so we can change and re-compose the transform later.
      originalOdomPose_ = odomPose;
      originalUtmPose_ = *odomUtmTransform_;
      odomUtmTransform_->mult(originalOdomPose_, originalUtmPose_.inverse());

      double odomRoll;
      double odomPitch;
      double odomYaw;
      double utmRoll;
      double utmPitch;
      double utmYaw;

      mat.setRotation(odomPose.getRotation());
      mat.getRPY(odomRoll, odomPitch, odomYaw);

      ROS_INFO_STREAM("Latest " << latestOdomMessage_->header.frame_id << " pose is: " << std::fixed <<
                      "\nPosition: (" << odomPose.getOrigin().getX() << ", " <<
                                         odomPose.getOrigin().getY() << ", " <<
                                         odomPose.getOrigin().getZ() << ")" <<
                      "\nOrientation: (" << odomRoll << ", " <<
                                            odomPitch << ", " <<
                                            odomYaw << ")");

      mat.setRotation(odomUtmTransform_->getRotation());
      mat.getRPY(utmRoll, utmPitch, utmYaw);

      ROS_INFO_STREAM(odomUtmTransform_->frame_id_  << "->" << odomUtmTransform_->child_frame_id_ <<
                       " transform is now: " << std::fixed <<
                       "\nPosition: (" << odomUtmTransform_->getOrigin().getX() << ", " <<
                                          odomUtmTransform_->getOrigin().getY() << ", " <<
                                          odomUtmTransform_->getOrigin().getZ() << ")" <<
                       "\nOrientation: (" << utmRoll << ", " <<
                                             utmPitch << ", " <<
                                             utmYaw << ")");

      transformGood_ = true;
    }
  }
}

//! @brief Callback for the UTM data
//!
void utmCallback(const nav_msgs::OdometryConstPtr& msg)
{
  if(latestUtmMsg_ == NULL)
  {
    ROS_INFO("Received initial UTM message");

    latestUtmMsg_ = new nav_msgs::Odometry();
  }

  *latestUtmMsg_ = *msg;

  if(zeroAltitude_ && odomUtmTransform_ != NULL)
  {
    // Grab the original transforms, update the z position
    // in the original UTM transform, and then put it back
    // together.
    tf::Vector3 origin = originalUtmPose_.getOrigin();
    origin.setZ(msg->pose.pose.position.z);
    originalUtmPose_.setOrigin(origin);

    odomUtmTransform_->mult(originalOdomPose_, originalUtmPose_.inverse());
  }
}

//! @brief Callback for the IMU data
//!
void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  if(latestImuMsg_ == NULL)
  {
    ROS_INFO("Received initial IMU message");

    latestImuMsg_ = new sensor_msgs::Imu();
  }

  *latestImuMsg_ = *msg;
}

//! @brief Callback for the odom data
//!
void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  if(latestOdomMessage_ == NULL)
  {
    ROS_INFO("Received initial relay odometry message");

    latestOdomMessage_ = new nav_msgs::Odometry();
  }

  *latestOdomMessage_ = *msg;
}

//! @brief Callback for the GPS fix data
//!
void gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg)
{
  hasFix_ = (msg->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX &&
             !std::isnan(msg->altitude) &&
             !std::isnan(msg->latitude) &&
             !std::isnan(msg->longitude));
}

//! @brief Waits until a transform is ready, then
//! broadcasts it at a specified rate
//!
int main(int argc, char **argv)
{
  ros::init(argc, argv, "utm_transform_node");

  ros::Time::init();

  ros::NodeHandle nh;
  ros::NodeHandle nhPriv("~");

  tf::TransformBroadcaster broadcaster;

  latestUtmMsg_ = NULL;
  latestImuMsg_ = NULL;
  latestOdomMessage_ = NULL;
  odomUtmTransform_ = NULL;
  tfListener_ = new tf::TransformListener();

  magneticDeclination_ = 0;
  utmOdomTfYaw_ = 0;
  rollOffset_ = 0;
  pitchOffset_ = 0;
  yawOffset_ = 0;
  hasFix_ = false;
  transformGood_ = false;
  double frequency = 0;

  // Subscribe to the messages we need
  ros::Subscriber utmSub = nh.subscribe("gps/gps_utm", 10, &utmCallback);
  ros::Subscriber imuSub = nh.subscribe("imu/data", 10, &imuCallback);
  ros::Subscriber odomSub = nh.subscribe("odometry/filtered", 10, &odomCallback);
  ros::Subscriber gpsFixSub = nh.subscribe("gps/fix", 10, &gpsFixCallback);

  // Load the parameters we need
  nhPriv.getParam("magnetic_declination_radians", magneticDeclination_);
  nhPriv.getParam("roll_offset", rollOffset_);
  nhPriv.getParam("pitch_offset", pitchOffset_);
  nhPriv.getParam("yaw_offset", yawOffset_);
  nhPriv.param("zero_altitude", zeroAltitude_, false);
  nhPriv.param("frequency", frequency, 50.0);

  ros::Rate rate(frequency);
  while(ros::ok())
  {
    ros::spinOnce();

    if(!transformGood_)
    {
      computeOdomUtmTransform();
    }
    else
    {
      odomUtmTransform_->stamp_ = ros::Time::now();
      broadcaster.sendTransform(*odomUtmTransform_);
    }

    rate.sleep();
  }

  delete latestUtmMsg_;
  delete latestImuMsg_;
  delete latestOdomMessage_;
  delete odomUtmTransform_;

  return 0;
}

