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
#include "robot_localization/navsat_conversions.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <XmlRpcException.h>

namespace RobotLocalization
{
  NavSatTransform::NavSatTransform() :
    magneticDeclination_(0.0),
    utmOdomTfYaw_(0.0),
    yawOffset_(0.0),
    broadcastUtmTransform_(false),
    hasTransformOdom_(false),
    hasTransformGps_(false),
    hasTransformImu_(false),
    transformGood_(false),
    gpsUpdated_(false),
    odomUpdated_(false),
    publishGps_(false),
    useOdometryYaw_(false),
    useManualDatum_(false),
    zeroAltitude_(false),
    worldFrameId_("odom"),
    utmZone_("")
 {
    latestUtmCovariance_.resize(POSE_SIZE, POSE_SIZE);
  }

  NavSatTransform::~NavSatTransform()
  {

  }

  bool NavSatTransform::datumCallback(robot_localization::SetDatum::Request& request,
                                      robot_localization::SetDatum::Response&)
  {
    transformGood_ = false;

    sensor_msgs::NavSatFix *fix = new sensor_msgs::NavSatFix();
    fix->latitude = request.geo_pose.position.latitude;
    fix->longitude = request.geo_pose.position.longitude;
    fix->altitude = request.geo_pose.position.altitude;
    fix->header.stamp = ros::Time::now();
    fix->position_covariance[0] = 0.1;
    fix->position_covariance[4] = 0.1;
    fix->position_covariance[8] = 0.1;
    fix->position_covariance_type = sensor_msgs::NavSatStatus::STATUS_FIX;
    sensor_msgs::NavSatFixConstPtr fixPtr(fix);
    setTransformGps(fixPtr);

    sensor_msgs::Imu *imu = new sensor_msgs::Imu();
    imu->orientation = request.geo_pose.orientation;
    sensor_msgs::ImuConstPtr imuPtr(imu);
    imuCallback(imuPtr);

    nav_msgs::Odometry *odom = new nav_msgs::Odometry();
    odom->pose.pose.orientation.x = 0;
    odom->pose.pose.orientation.y = 0;
    odom->pose.pose.orientation.z = 0;
    odom->pose.pose.orientation.w = 1;
    odom->pose.pose.position.x = 0;
    odom->pose.pose.position.y = 0;
    odom->pose.pose.position.z = 0;
    odom->header.frame_id = worldFrameId_;
    nav_msgs::OdometryConstPtr odomPtr(odom);
    setTransformOdometry(odomPtr);

    return true;
  }

  void NavSatTransform::odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    if(!transformGood_ && !useManualDatum_)
    {
      setTransformOdometry(msg);
    }

    tf2::fromMsg(msg->pose.pose, latestWorldPose_);
    odomUpdateTime_ = msg->header.stamp;
    odomUpdated_ = true;
  }

  void NavSatTransform::gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg)
  {
    // Make sure the GPS data is usable
    bool goodGps = (msg->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX &&
                    !std::isnan(msg->altitude) &&
                    !std::isnan(msg->latitude) &&
                    !std::isnan(msg->longitude));

    if(goodGps)
    {
      // If we haven't computed the transform yet, then
      // store this message as the initial GPS data to use
      if(!transformGood_ && !useManualDatum_)
      {
        setTransformGps(msg);
      }

      double utmX = 0;
      double utmY = 0;
      std::string utmZoneTmp;
      NavsatConversions::LLtoUTM(msg->latitude, msg->longitude, utmY, utmX, utmZoneTmp);
      latestUtmPose_.setOrigin(tf2::Vector3(utmX, utmY, msg->altitude));
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

  void NavSatTransform::setTransformGps(const sensor_msgs::NavSatFixConstPtr& msg)
  {
    double utmX = 0;
    double utmY = 0;
    NavsatConversions::LLtoUTM(msg->latitude, msg->longitude, utmY, utmX, utmZone_);

    ROS_INFO_STREAM("Datum (latitude, longitude, altitude) is (" << std::fixed << msg->latitude << ", " << msg->longitude << ", " << msg->altitude << ")");
    ROS_INFO_STREAM("Datum UTM coordinate is (" << std::fixed << utmX << ", " << utmY << ")");

    transformUtmPose_.setOrigin(tf2::Vector3(utmX, utmY, msg->altitude));
    transformUtmPose_.setRotation(tf2::Quaternion::getIdentity());
    hasTransformGps_ = true;
  }

  void NavSatTransform::setTransformOdometry(const nav_msgs::OdometryConstPtr& msg)
  {
    tf2::fromMsg(msg->pose.pose, transformWorldPose_);
    worldFrameId_ = msg->header.frame_id;
    hasTransformOdom_ = true;

    ROS_INFO_STREAM("Initial odometry position is (" << std::fixed << 
                    transformWorldPose_.getOrigin().getX() << ", " << 
                    transformWorldPose_.getOrigin().getY() << ", " << 
                    transformWorldPose_.getOrigin().getZ() << ")");

    // Users can optionally use the (potentially fused) heading from
    // the odometry source, which may have multiple fused sources of
    // heading data, and so would act as a better heading for the
    // UTM->world_frame transform.
    if(!transformGood_ && useOdometryYaw_ && !useManualDatum_)
    {
      sensor_msgs::Imu imu;
      imu.orientation = msg->pose.pose.orientation;
      imu.header.frame_id = msg->child_frame_id;
      sensor_msgs::ImuConstPtr imuPtr(&imu);
      imuCallback(imuPtr);
    }
  }

  void NavSatTransform::imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    /* This method only gets called if we don't yet have the
     * IMU data (the subscriber gets shut down once we compute
     * the transform), so we can assumed that every IMU message
     * that comes here is meant to be used for that purpose. */
    tf2::fromMsg(msg->orientation, transformOrientation_);
    hasTransformImu_ = true;

    double roll, pitch, yaw;
    tf2::Matrix3x3 mat;
    mat.setRotation(transformOrientation_);
    mat.getRPY(roll, pitch, yaw);
    ROS_INFO_STREAM("Initial orientation roll, pitch, yaw is (" << 
                    roll << ", " << pitch << ", " << yaw << ")");
  }

  void NavSatTransform::computeTransform()
  {
    // Only do this if:
    // 1. We haven't computed the odom_frame->utm_frame transform before
    // 2. We've received the data we need
    if(!transformGood_ &&
       hasTransformOdom_ &&
       hasTransformGps_ &&
       hasTransformImu_)
    {
      // Get the IMU's current RPY values. Need the raw values (for yaw, anyway).
      tf2::Matrix3x3 mat(transformOrientation_);

      // Convert to RPY
      double imuRoll;
      double imuPitch;
      double imuYaw;
      mat.getRPY(imuRoll, imuPitch, imuYaw);

      /* The IMU's heading was likely originally reported w.r.t. magnetic north.
       * However, all the nodes in robot_localization assume that orientation data,
       * including that reported by IMUs, is reported in an ENU frame, with a 0 yaw
       * value being reported when facing east and increasing counter-clockwise (i.e.,
       * towards north). Conveniently, this aligns with the UTM grid, where X is east
       * and Y is north. However, we have two additional considerations:
       *   1. The IMU may have its non-ENU frame data transformed to ENU, but there's
       *      a possibility that its data has not been corrected for magnetic
       *      declination. We need to account for this. A positive magnetic
       *      declination is counter-clockwise in an ENU frame. Therefore, if
       *      we have a magnetic declination of N radians, then when the sensor
       *      is facing a heading of N, it reports 0. Therefore, we need to add
       *      the declination angle.
       *   2. To account for any other offsets that may not be accounted for by the
       *      IMU driver or any interim processing node, we expose a yaw offset that
       *      lets users work with navsat_transform_node.
       */
      imuYaw += (magneticDeclination_ + yawOffset_);

      ROS_INFO_STREAM("Corrected for magnetic declination of " << std::fixed << magneticDeclination_ <<
                      " and user-specified offset of " << yawOffset_ << ". Transform heading factor is now " << imuYaw);

      // Convert to tf-friendly structures
      tf2::Quaternion imuQuat;
      imuQuat.setRPY(0.0, 0.0, imuYaw);

      // The transform order will be orig_odom_pos * orig_utm_pos_inverse * cur_utm_pos.
      // Doing it this way will allow us to cope with having non-zero odometry position
      // when we get our first GPS message.
      tf2::Transform utmPoseWithOrientation;
      utmPoseWithOrientation.setOrigin(transformUtmPose_.getOrigin());
      utmPoseWithOrientation.setRotation(imuQuat);
      utmWorldTransform_.mult(transformWorldPose_, utmPoseWithOrientation.inverse());

      utmWorldTransInverse_ = utmWorldTransform_.inverse();

      double roll = 0;
      double pitch = 0;
      double yaw = 0;
      mat.setRotation(latestWorldPose_.getRotation());
      mat.getRPY(roll, pitch, yaw);

      ROS_INFO_STREAM("Transform world frame pose is: " << std::fixed <<
                      "\nPosition: (" << transformWorldPose_.getOrigin().getX() << ", " <<
                                         transformWorldPose_.getOrigin().getY() << ", " <<
                                         transformWorldPose_.getOrigin().getZ() << ")" <<
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

      // Send out the (static) UTM transform in case anyone else would like to use it.
      if(broadcastUtmTransform_)
      {
        geometry_msgs::TransformStamped utmTransformStamped;
        utmTransformStamped.header.stamp = ros::Time::now();
        utmTransformStamped.header.frame_id = worldFrameId_;
        utmTransformStamped.child_frame_id = "utm";
        utmTransformStamped.transform = tf2::toMsg(utmWorldTransform_);
        utmBroadcaster_.sendTransform(utmTransformStamped);
      }
    }
  }

  bool NavSatTransform::prepareGpsOdometry(nav_msgs::Odometry &gpsOdom)
  {
    bool newData = false;

    if(transformGood_ && gpsUpdated_)
    {
      tf2::Transform transformedUtm;

      transformedUtm.mult(utmWorldTransform_, latestUtmPose_);
      transformedUtm.setRotation(tf2::Quaternion::getIdentity());

      // Rotate the covariance as well
      tf2::Matrix3x3 rot(utmWorldTransform_.getRotation());
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
      tf2::toMsg(transformedUtm, gpsOdom.pose.pose);
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
      tf2::Transform odomAsUtm;

      odomAsUtm.mult(utmWorldTransInverse_, latestWorldPose_);
      odomAsUtm.setRotation(tf2::Quaternion::getIdentity());

      // Rotate the covariance as well
      tf2::Matrix3x3 rot(utmWorldTransInverse_.getRotation());
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
      NavsatConversions::UTMtoLL(odomAsUtm.getOrigin().getY(), odomAsUtm.getOrigin().getX(), utmZone_, filteredGps.latitude, filteredGps.longitude);
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
      filteredGps.header.stamp = odomUpdateTime_;

      // Mark this GPS as used
      odomUpdated_ = false;
      newData = true;
    }

    return newData;
  }

  void NavSatTransform::run()
  {
    ros::Time::init();

    double frequency = 10.0;
    double delay = 0.0;

    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");

    // Load the parameters we need
    nhPriv.getParam("magnetic_declination_radians", magneticDeclination_);
    nhPriv.param("yaw_offset", yawOffset_, 0.0);
    nhPriv.param("broadcast_utm_transform", broadcastUtmTransform_, false);
    nhPriv.param("zero_altitude", zeroAltitude_, false);
    nhPriv.param("publish_filtered_gps", publishGps_, false);
    nhPriv.param("use_odometry_yaw", useOdometryYaw_, false);
    nhPriv.param("wait_for_datum", useManualDatum_, false);
    nhPriv.param("frequency", frequency, 10.0);
    nhPriv.param("delay", delay, 0.0);

    // Subscribe to the messages and services we need
    ros::ServiceServer datumServ = nh.advertiseService("datum", &NavSatTransform::datumCallback, this);

    if(useManualDatum_ && nhPriv.hasParam("datum"))
    {
      XmlRpc::XmlRpcValue datumConfig;

      try
      {
        double datumLat;
        double datumLon;
        double datumYaw;

        nhPriv.getParam("datum", datumConfig);

        ROS_ASSERT(datumConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(datumConfig.size() == 4);

        useManualDatum_ = true;

        std::ostringstream ostr;
        ostr << datumConfig[0] << " " << datumConfig[1] << " " << datumConfig[2] << " " << datumConfig[3];
        std::istringstream istr(ostr.str());
        istr >> datumLat >> datumLon >> datumYaw >> worldFrameId_;

        robot_localization::SetDatum::Request request;
        request.geo_pose.position.latitude = datumLat;
        request.geo_pose.position.longitude = datumLon;
        request.geo_pose.position.altitude = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, datumYaw);
        request.geo_pose.orientation = tf2::toMsg(quat);
        robot_localization::SetDatum::Response response;
        datumCallback(request, response);
      }
      catch (XmlRpc::XmlRpcException &e)
      {
        ROS_ERROR_STREAM("ERROR reading sensor config: " << e.getMessage() <<
                         " for process_noise_covariance (type: " << datumConfig.getType() << ")");
      }
    }

    ros::Subscriber odomSub = nh.subscribe("odometry/filtered", 1, &NavSatTransform::odomCallback, this);
    ros::Subscriber gpsSub = nh.subscribe("gps/fix", 1, &NavSatTransform::gpsFixCallback, this);
    ros::Subscriber imuSub;

    if(!useOdometryYaw_ && !useManualDatum_)
    {
      imuSub = nh.subscribe("imu/data", 1, &NavSatTransform::imuCallback, this);
    }

    ros::Publisher gpsOdomPub = nh.advertise<nav_msgs::Odometry>("odometry/gps", 10);
    ros::Publisher filteredGpsPub;

    if(publishGps_)
    {
      filteredGpsPub = nh.advertise<sensor_msgs::NavSatFix>("gps/filtered", 10);
    }

    // Sleep for the parameterized amount of time, to give
    // other nodes time to start up (not always necessary)
    ros::Duration startDelay(delay);
    startDelay.sleep();

    ros::Rate rate(frequency);
    while(ros::ok())
    {
      ros::spinOnce();

      if(!transformGood_)
      {
        computeTransform();

        if(transformGood_ && !useOdometryYaw_ && !useManualDatum_)
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
      }

      rate.sleep();
    }
  }
}
