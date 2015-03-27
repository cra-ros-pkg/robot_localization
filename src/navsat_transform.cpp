/*
 * Copyright (c) 2015, Charles River Analytics, Inc.
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
#include "robot_localization/filter_common.h"

#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>

// This header is from the gps_common package
// by Ken Tossell. We use it to convert the
// lat/lon data into UTM data.
#include <gps_common/conversions.h>

namespace RobotLocalization
{
  NavSatTransform::NavSatTransform() :
    magneticDeclination_(0.0),
    utmOdomTfYaw_(0.0),
    yawOffset_(0.0),
    broadcastUtmTransform_(false),
    hasOdom_(false),
    hasGps_(false),
    hasImu_(false),
    transformGood_(false),
    gpsUpdated_(false),
    odomUpdated_(false),
    publishGps_(false),
    worldFrameId_("odom"),
    utmZone_("")
 {
    latestUtmCovariance_.resize(POSE_SIZE, POSE_SIZE);
  }

  NavSatTransform::~NavSatTransform()
  {

  }

  void NavSatTransform::odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    tf::poseMsgToTF(msg->pose.pose, latestWorldPose_);
    worldFrameId_ = msg->header.frame_id;
    hasOdom_ = true;
    odomUpdateTime_ = msg->header.stamp;
    odomUpdated_ = true;
  }

  void NavSatTransform::gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg)
  {
    hasGps_ = (msg->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX &&
               !std::isnan(msg->altitude) &&
               !std::isnan(msg->latitude) &&
               !std::isnan(msg->longitude));

    if(hasGps_)
    {
      double utmX = 0;
      double utmY = 0;
      gps_common::LLtoUTM(msg->latitude, msg->longitude, utmY, utmX, utmZone_);
      latestUtmPose_.setOrigin(tf::Vector3(utmX, utmY, msg->altitude));
      latestUtmCovariance_.setZero();

      // Copy the measurement's covariance matrix so that we can rotate it later
      for (size_t i = 0; i < POSITION_SIZE; i++)
      {
        for (size_t j = 0; j < POSITION_SIZE; j++)
        {
          latestUtmCovariance_(i, j) = msg->position_covariance[POSITION_SIZE * i + j];
        }
      }

      gpsUpdateTime_ = msg->header.stamp;
      gpsUpdated_ = true;
    }
  }

  void NavSatTransform::imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    tf::quaternionMsgToTF(msg->orientation, latestOrientation_);
    hasImu_ = true;
  }

  void NavSatTransform::computeTransform()
  {
    // Only do this if:
    // 1. We haven't computed the odom_frame->utm_frame transform before
    // 2. We've received the data we need
    if(!transformGood_ &&
       hasOdom_ &&
       hasGps_ &&
       hasImu_)
    {
      // Get the IMU's current RPY values. Need the raw values (for yaw, anyway).
      tf::Matrix3x3 mat(latestOrientation_);

      // Convert to RPY
      double imuRoll;
      double imuPitch;
      double imuYaw;
      mat.getRPY(imuRoll, imuPitch, imuYaw);

      // Compute the final yaw value that corrects for the difference between the
      // IMU's heading and the UTM grid's belief of where 0 heading should be (i.e.,
      // along the x-axis)
      imuYaw += (magneticDeclination_ + yawOffset_ + (M_PI / 2.0));

      ROS_INFO_STREAM("Corrected for magnetic declination of " << std::fixed << magneticDeclination_ <<
                      ", user-specified offset of " << yawOffset_ << ", and fixed offset of " << (M_PI / 2.0) <<
                      ". Transform heading factor is now " << imuYaw);

      // Convert to tf-friendly structures
      tf::Quaternion imuQuat;
      imuQuat.setRPY(0.0, 0.0, imuYaw);

      // The transform order will be orig_odom_pos * orig_utm_pos_inverse * cur_utm_pos.
      // Doing it this way will allow us to cope with having non-zero odometry position
      // when we get our first GPS message.
      tf::Pose utmPoseWithOrientation;
      utmPoseWithOrientation.setOrigin(latestUtmPose_.getOrigin());
      utmPoseWithOrientation.setRotation(imuQuat);
      utmWorldTransform_.mult(latestWorldPose_, utmPoseWithOrientation.inverse());

      utmWorldTransInverse_ = utmWorldTransform_.inverse();

      double roll = 0;
      double pitch = 0;
      double yaw = 0;
      mat.setRotation(latestWorldPose_.getRotation());
      mat.getRPY(roll, pitch, yaw);

      ROS_INFO_STREAM("Latest world frame pose is: " << std::fixed <<
                      "\nPosition: (" << latestWorldPose_.getOrigin().getX() << ", " <<
                                         latestWorldPose_.getOrigin().getY() << ", " <<
                                         latestWorldPose_.getOrigin().getZ() << ")" <<
                      "\nOrientation: (" << roll << ", " <<
                                            pitch << ", " <<
                                            yaw << ")");

      mat.setRotation(utmWorldTransform_.getRotation());
      mat.getRPY(roll, pitch, yaw);

      ROS_INFO_STREAM("World frame->utm transform is " << std::fixed <<
                       "\nPosition: (" << utmWorldTransform_.getOrigin().getX() << ", " <<
                                          utmWorldTransform_.getOrigin().getY() << ", " <<
                                          utmWorldTransform_.getOrigin().getZ() << ")" <<
                       "\nOrientation: (" << roll << ", " <<
                                             pitch << ", " <<
                                             yaw << ")");

      transformGood_ = true;
    }
  }

  bool NavSatTransform::prepareGpsOdometry(nav_msgs::Odometry &gpsOdom)
  {
    bool newData = false;

    if(transformGood_ && gpsUpdated_)
    {
      tf::Pose transformedUtm;

      transformedUtm.mult(utmWorldTransform_, latestUtmPose_);
      transformedUtm.setRotation(tf::Quaternion::getIdentity());

      // Rotate the covariance as well
      tf::Matrix3x3 rot(utmWorldTransform_.getRotation());
      Eigen::MatrixXd rot6d(POSE_SIZE, POSE_SIZE);
      rot6d.setIdentity();

      for(size_t rInd = 0; rInd < POSITION_SIZE; ++rInd)
      {
        rot6d(rInd, 0) = rot.getRow(rInd).getX();
        rot6d(rInd, 1) = rot.getRow(rInd).getY();
        rot6d(rInd, 2) = rot.getRow(rInd).getZ();
        rot6d(rInd+POSITION_SIZE, 3) = rot.getRow(rInd).getX();
        rot6d(rInd+POSITION_SIZE, 4) = rot.getRow(rInd).getY();
        rot6d(rInd+POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
      }

      // Rotate the covariance
      latestUtmCovariance_ = rot6d * latestUtmCovariance_.eval() * rot6d.transpose();

      // Now fill out the message. Set the orientation to the identity.
      tf::poseTFToMsg(transformedUtm, gpsOdom.pose.pose);
      gpsOdom.pose.pose.position.z = (zeroAltitude_ ? 0.0 : gpsOdom.pose.pose.position.z);

      // Copy the measurement's covariance matrix so that we can rotate it later
      for (size_t i = 0; i < POSE_SIZE; i++)
      {
        for (size_t j = 0; j < POSE_SIZE; j++)
        {
          gpsOdom.pose.covariance[POSE_SIZE * i + j] = latestUtmCovariance_(i, j);
        }
      }

      gpsOdom.header.frame_id = worldFrameId_;
      gpsOdom.header.stamp = gpsUpdateTime_;

      // Mark this GPS as used
      gpsUpdated_ = false;
      newData = true;
    }

    return newData;
  }

  bool NavSatTransform::prepareFilteredGps(sensor_msgs::NavSatFix &filteredGps)
  {
    bool newData = false;

    if(transformGood_ && odomUpdated_)
    {
      tf::Pose odomAsUtm;

      odomAsUtm.mult(utmWorldTransInverse_, latestWorldPose_);
      odomAsUtm.setRotation(tf::Quaternion::getIdentity());

      // Rotate the covariance as well
      tf::Matrix3x3 rot(utmWorldTransInverse_.getRotation());
      Eigen::MatrixXd rot6d(POSE_SIZE, POSE_SIZE);
      rot6d.setIdentity();

      for(size_t rInd = 0; rInd < POSITION_SIZE; ++rInd)
      {
        rot6d(rInd, 0) = rot.getRow(rInd).getX();
        rot6d(rInd, 1) = rot.getRow(rInd).getY();
        rot6d(rInd, 2) = rot.getRow(rInd).getZ();
        rot6d(rInd+POSITION_SIZE, 3) = rot.getRow(rInd).getX();
        rot6d(rInd+POSITION_SIZE, 4) = rot.getRow(rInd).getY();
        rot6d(rInd+POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
      }

      // Rotate the covariance
      latestOdomCovariance_ = rot6d * latestOdomCovariance_.eval() * rot6d.transpose();

      // Now convert the data back to lat/long and place into the message
      gps_common::UTMtoLL(odomAsUtm.getOrigin().getY(), odomAsUtm.getOrigin().getX(), utmZone_, filteredGps.latitude, filteredGps.longitude);
      filteredGps.altitude = odomAsUtm.getOrigin().getZ();

      // Copy the measurement's covariance matrix back
      for (size_t i = 0; i < POSITION_SIZE; i++)
      {
        for (size_t j = 0; j < POSITION_SIZE; j++)
        {
          filteredGps.position_covariance[POSITION_SIZE * i + j] = latestUtmCovariance_(i, j);
        }
      }

      filteredGps.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
      filteredGps.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
      filteredGps.header.frame_id = "gps";
      filteredGps.header.stamp = gpsUpdateTime_;

      // Mark this GPS as used
      odomUpdated_ = false;
      newData = true;
    }

    return newData;
  }

  void NavSatTransform::run()
  {
    ros::Time::init();

    double frequency = 10;

    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    // Subscribe to the messages we need
    ros::Subscriber odomSub = nh.subscribe("odometry/filtered", 1, &NavSatTransform::odomCallback, this);
    ros::Subscriber gpsSub = nh.subscribe("gps/fix", 1, &NavSatTransform::gpsFixCallback, this);
    ros::Subscriber imuSub = nh.subscribe("imu/data", 1, &NavSatTransform::imuCallback, this);

    // Load the parameters we need
    nhPriv.getParam("magnetic_declination_radians", magneticDeclination_);
    nhPriv.getParam("yaw_offset", yawOffset_);
    nhPriv.param("broadcast_utm_transform", broadcastUtmTransform_, false);
    nhPriv.param("zero_altitude", zeroAltitude_, false);
    nhPriv.param("publish_filtered_gps", publishGps_, false);
    nhPriv.param("frequency", frequency, 10.0);

    ros::Publisher gpsOdomPub = nh.advertise<nav_msgs::Odometry>("odometry/gps", 10);
    ros::Publisher filteredGpsPub;

    if(publishGps_)
    {
      filteredGpsPub = nh.advertise<sensor_msgs::NavSatFix>("gps/filtered", 10);
    }

    tf::TransformBroadcaster utmBroadcaster;
    tf::StampedTransform utmTransformStamped;
    utmTransformStamped.child_frame_id_ = "utm";

    ros::Rate rate(frequency);
    while(ros::ok())
    {
      ros::spinOnce();

      if(!transformGood_)
      {
        computeTransform();

        if(transformGood_)
        {
          // Once we have the transform, we don't need the IMU
          imuSub.shutdown();
        }
      }
      else
      {
        nav_msgs::Odometry gpsOdom;
        if(prepareGpsOdometry(gpsOdom))
        {
          gpsOdomPub.publish(gpsOdom);
        }

        if(publishGps_)
        {
          sensor_msgs::NavSatFix odomGps;
          if(prepareFilteredGps(odomGps))
          {
            filteredGpsPub.publish(odomGps);
          }
        }

        // Send out the UTM transform in case anyone
        // else would like to use it.
        if(transformGood_ && broadcastUtmTransform_)
        {
          utmTransformStamped.setData(utmWorldTransform_);
          utmTransformStamped.frame_id_ = worldFrameId_;
          utmTransformStamped.stamp_ = ros::Time::now();
          utmBroadcaster.sendTransform(utmTransformStamped);
        }
      }

      rate.sleep();
    }

  }
}
