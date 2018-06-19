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

#include "robot_localization/navsat_transform.h"
#include "robot_localization/filter_common.h"
#include "robot_localization/filter_utilities.h"
#include "robot_localization/navsat_conversions.h"
#include "robot_localization/ros_filter_utilities.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <XmlRpcException.h>

#include <string>

namespace RobotLocalization
{
  NavSatTransform::NavSatTransform() :
    magnetic_declination_(0.0),
    utm_odom_tf_yaw_(0.0),
    yaw_offset_(0.0),
    transform_timeout_(ros::Duration(0)),
    broadcast_utm_transform_(false),
    broadcast_utm_transform_as_parent_frame_(false),
    has_transform_odom_(false),
    has_transform_gps_(false),
    has_transform_imu_(false),
    transform_good_(false),
    gps_frame_id_(""),
    gps_updated_(false),
    odom_updated_(false),
    publish_gps_(false),
    use_odometry_yaw_(false),
    use_manual_datum_(false),
    zero_altitude_(false),
    world_frame_id_("odom"),
    base_link_frame_id_("base_link"),
    utm_zone_(""),
    tf_listener_(tf_buffer_)
  {
    latest_utm_covariance_.resize(POSE_SIZE, POSE_SIZE);
    latest_odom_covariance_.resize(POSE_SIZE, POSE_SIZE);
  }

  NavSatTransform::~NavSatTransform()
  {
  }

  void NavSatTransform::run()
  {
    ROS_INFO("Waiting for valid clock time...");
    ros::Time::waitForValid();
    ROS_INFO("Valid clock time received. Starting node.");

    double frequency = 10.0;
    double delay = 0.0;
    double transform_timeout = 0.0;

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    // Load the parameters we need
    nh_priv.getParam("magnetic_declination_radians", magnetic_declination_);
    nh_priv.param("yaw_offset", yaw_offset_, 0.0);
    nh_priv.param("broadcast_utm_transform", broadcast_utm_transform_, false);
    nh_priv.param("broadcast_utm_transform_as_parent_frame", broadcast_utm_transform_as_parent_frame_, false);
    nh_priv.param("zero_altitude", zero_altitude_, false);
    nh_priv.param("publish_filtered_gps", publish_gps_, false);
    nh_priv.param("use_odometry_yaw", use_odometry_yaw_, false);
    nh_priv.param("wait_for_datum", use_manual_datum_, false);
    nh_priv.param("frequency", frequency, 10.0);
    nh_priv.param("delay", delay, 0.0);
    nh_priv.param("transform_timeout", transform_timeout, 0.0);
    transform_timeout_.fromSec(transform_timeout);

    // Subscribe to the messages and services we need
    ros::ServiceServer datum_srv = nh.advertiseService("datum", &NavSatTransform::datumCallback, this);

    if (use_manual_datum_ && nh_priv.hasParam("datum"))
    {
      XmlRpc::XmlRpcValue datum_config;

      try
      {
        double datum_lat;
        double datum_lon;
        double datum_yaw;

        nh_priv.getParam("datum", datum_config);

        // Handle datum specification. Users should always specify a baseLinkFrameId_ in the
        // datum config, but we had a release where it wasn't used, so we'll maintain compatibility.
        ROS_ASSERT(datum_config.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(datum_config.size() >= 3);

        if (datum_config.size() > 3)
        {
          ROS_WARN_STREAM("Deprecated datum parameter configuration detected. Only the first three parameters "
              "(latitude, longitude, yaw) will be used. frame_ids will be derived from odometry and navsat inputs.");
        }

        std::ostringstream ostr;
        ostr << std::setprecision(20) << datum_config[0] << " " << datum_config[1] << " " << datum_config[2];
        std::istringstream istr(ostr.str());
        istr >> datum_lat >> datum_lon >> datum_yaw;

        // Try to resolve tf_prefix
        std::string tf_prefix = "";
        std::string tf_prefix_path = "";
        if (nh_priv.searchParam("tf_prefix", tf_prefix_path))
        {
          nh_priv.getParam(tf_prefix_path, tf_prefix);
        }

        // Append the tf prefix in a tf2-friendly manner
        FilterUtilities::appendPrefix(tf_prefix, world_frame_id_);
        FilterUtilities::appendPrefix(tf_prefix, base_link_frame_id_);

        robot_localization::SetDatum::Request request;
        request.geo_pose.position.latitude = datum_lat;
        request.geo_pose.position.longitude = datum_lon;
        request.geo_pose.position.altitude = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, datum_yaw);
        request.geo_pose.orientation = tf2::toMsg(quat);
        robot_localization::SetDatum::Response response;
        datumCallback(request, response);
      }
      catch (XmlRpc::XmlRpcException &e)
      {
        ROS_ERROR_STREAM("ERROR reading sensor config: " << e.getMessage() <<
                         " for process_noise_covariance (type: " << datum_config.getType() << ")");
      }
    }

    ros::Subscriber odom_sub = nh.subscribe("odometry/filtered", 1, &NavSatTransform::odomCallback, this);
    ros::Subscriber gps_sub = nh.subscribe("gps/fix", 1, &NavSatTransform::gpsFixCallback, this);
    ros::Subscriber imu_sub;

    if (!use_odometry_yaw_ && !use_manual_datum_)
    {
      imu_sub = nh.subscribe("imu/data", 1, &NavSatTransform::imuCallback, this);
    }

    ros::Publisher gps_odom_pub = nh.advertise<nav_msgs::Odometry>("odometry/gps", 10);
    ros::Publisher filtered_gps_pub;

    if (publish_gps_)
    {
      filtered_gps_pub = nh.advertise<sensor_msgs::NavSatFix>("gps/filtered", 10);
    }

    // Sleep for the parameterized amount of time, to give
    // other nodes time to start up (not always necessary)
    ros::Duration start_delay(delay);
    start_delay.sleep();

    ros::Rate rate(frequency);
    while (ros::ok())
    {
      ros::spinOnce();

      if (!transform_good_)
      {
        computeTransform();

        if (transform_good_ && !use_odometry_yaw_ && !use_manual_datum_)
        {
          // Once we have the transform, we don't need the IMU
          imu_sub.shutdown();
        }
      }
      else
      {
        nav_msgs::Odometry gps_odom;
        if (prepareGpsOdometry(gps_odom))
        {
          gps_odom_pub.publish(gps_odom);
        }

        if (publish_gps_)
        {
          sensor_msgs::NavSatFix odom_gps;
          if (prepareFilteredGps(odom_gps))
          {
            filtered_gps_pub.publish(odom_gps);
          }
        }
      }

      rate.sleep();
    }
  }

  void NavSatTransform::computeTransform()
  {
    // Only do this if:
    // 1. We haven't computed the odom_frame->utm_frame transform before
    // 2. We've received the data we need
    if (!transform_good_ &&
        has_transform_odom_ &&
        has_transform_gps_ &&
        has_transform_imu_)
    {
      // The UTM pose we have is given at the location of the GPS sensor on the robot. We need to get the UTM pose of
      // the robot's origin.
      tf2::Transform transform_utm_pose_corrected;
      if (!use_manual_datum_)
      {
        getRobotOriginUtmPose(transform_utm_pose_, transform_utm_pose_corrected, ros::Time(0));
      }
      else
      {
        transform_utm_pose_corrected = transform_utm_pose_;
      }

      // Get the IMU's current RPY values. Need the raw values (for yaw, anyway).
      tf2::Matrix3x3 mat(transform_orientation_);

      // Convert to RPY
      double imu_roll;
      double imu_pitch;
      double imu_yaw;
      mat.getRPY(imu_roll, imu_pitch, imu_yaw);

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
      imu_yaw += (magnetic_declination_ + yaw_offset_);

      ROS_INFO_STREAM("Corrected for magnetic declination of " << std::fixed << magnetic_declination_ <<
                      " and user-specified offset of " << yaw_offset_ << "." <<
                      " Transform heading factor is now " << imu_yaw);

      // Convert to tf-friendly structures
      tf2::Quaternion imu_quat;
      imu_quat.setRPY(0.0, 0.0, imu_yaw);

      // The transform order will be orig_odom_pos * orig_utm_pos_inverse * cur_utm_pos.
      // Doing it this way will allow us to cope with having non-zero odometry position
      // when we get our first GPS message.
      tf2::Transform utm_pose_with_orientation;
      utm_pose_with_orientation.setOrigin(transform_utm_pose_corrected.getOrigin());
      utm_pose_with_orientation.setRotation(imu_quat);

      utm_world_transform_.mult(transform_world_pose_, utm_pose_with_orientation.inverse());

      utm_world_trans_inverse_ = utm_world_transform_.inverse();

      ROS_INFO_STREAM("Transform world frame pose is: " << transform_world_pose_);
      ROS_INFO_STREAM("World frame->utm transform is " << utm_world_transform_);

      transform_good_ = true;

      // Send out the (static) UTM transform in case anyone else would like to use it.
      if (broadcast_utm_transform_)
      {
        geometry_msgs::TransformStamped utm_transform_stamped;
        utm_transform_stamped.header.stamp = ros::Time::now();
        utm_transform_stamped.header.frame_id = (broadcast_utm_transform_as_parent_frame_ ? "utm" : world_frame_id_);
        utm_transform_stamped.child_frame_id = (broadcast_utm_transform_as_parent_frame_ ? world_frame_id_ : "utm");
        utm_transform_stamped.transform = (broadcast_utm_transform_as_parent_frame_ ?
                                             tf2::toMsg(utm_world_trans_inverse_) : tf2::toMsg(utm_world_transform_));
        utm_transform_stamped.transform.translation.z = (zero_altitude_ ?
                                                           0.0 : utm_transform_stamped.transform.translation.z);
        utm_broadcaster_.sendTransform(utm_transform_stamped);
      }
    }
  }

  bool NavSatTransform::datumCallback(robot_localization::SetDatum::Request& request,
                                      robot_localization::SetDatum::Response&)
  {
    // If we get a service call with a manual datum, even if we already computed the transform using the robot's
    // initial pose, then we want to assume that we are using a datum from now on, and we want other methods to
    // not attempt to transform the values we are specifying here.
    use_manual_datum_ = true;

    transform_good_ = false;

    sensor_msgs::NavSatFix *fix = new sensor_msgs::NavSatFix();
    fix->latitude = request.geo_pose.position.latitude;
    fix->longitude = request.geo_pose.position.longitude;
    fix->altitude = request.geo_pose.position.altitude;
    fix->header.stamp = ros::Time::now();
    fix->position_covariance[0] = 0.1;
    fix->position_covariance[4] = 0.1;
    fix->position_covariance[8] = 0.1;
    fix->position_covariance_type = sensor_msgs::NavSatStatus::STATUS_FIX;
    sensor_msgs::NavSatFixConstPtr fix_ptr(fix);
    setTransformGps(fix_ptr);

    nav_msgs::Odometry *odom = new nav_msgs::Odometry();
    odom->pose.pose.orientation.x = 0;
    odom->pose.pose.orientation.y = 0;
    odom->pose.pose.orientation.z = 0;
    odom->pose.pose.orientation.w = 1;
    odom->pose.pose.position.x = 0;
    odom->pose.pose.position.y = 0;
    odom->pose.pose.position.z = 0;
    odom->header.frame_id = world_frame_id_;
    odom->child_frame_id = base_link_frame_id_;
    nav_msgs::OdometryConstPtr odom_ptr(odom);
    setTransformOdometry(odom_ptr);

    sensor_msgs::Imu *imu = new sensor_msgs::Imu();
    imu->orientation = request.geo_pose.orientation;
    imu->header.frame_id = base_link_frame_id_;
    sensor_msgs::ImuConstPtr imu_ptr(imu);
    imuCallback(imu_ptr);

    return true;
  }

  void NavSatTransform::getRobotOriginUtmPose(const tf2::Transform &gps_utm_pose,
                                              tf2::Transform &robot_utm_pose,
                                              const ros::Time &transform_time)
  {
    robot_utm_pose.setIdentity();

    // Get linear offset from origin for the GPS
    tf2::Transform offset;
    bool can_transform = RosFilterUtilities::lookupTransformSafe(tf_buffer_,
                                                                 base_link_frame_id_,
                                                                 gps_frame_id_,
                                                                 transform_time,
                                                                 ros::Duration(transform_timeout_),
                                                                 offset);

    if (can_transform)
    {
      // Get the orientation we'll use for our UTM->world transform
      tf2::Quaternion utm_orientation = transform_orientation_;
      tf2::Matrix3x3 mat(utm_orientation);

      // Add the offsets
      double roll;
      double pitch;
      double yaw;
      mat.getRPY(roll, pitch, yaw);
      yaw += (magnetic_declination_ + yaw_offset_);
      utm_orientation.setRPY(roll, pitch, yaw);

      // Rotate the GPS linear offset by the orientation
      // Zero out the orientation, because the GPS orientation is meaningless, and if it's non-zero, it will make the
      // the computation of robot_utm_pose erroneous.
      offset.setOrigin(tf2::quatRotate(utm_orientation, offset.getOrigin()));
      offset.setRotation(tf2::Quaternion::getIdentity());

      // Update the initial pose
      robot_utm_pose = offset.inverse() * gps_utm_pose;
    }
    else
    {
      if (gps_frame_id_ != "")
      {
        ROS_WARN_STREAM_ONCE("Unable to obtain " << base_link_frame_id_ << "->" << gps_frame_id_ <<
          " transform. Will assume navsat device is mounted at robot's origin");
      }

      robot_utm_pose = gps_utm_pose;
    }
  }

  void NavSatTransform::getRobotOriginWorldPose(const tf2::Transform &gps_odom_pose,
                                                tf2::Transform &robot_odom_pose,
                                                const ros::Time &transform_time)
  {
    robot_odom_pose.setIdentity();

    // Remove the offset from base_link
    tf2::Transform gps_offset_rotated;
    bool can_transform = RosFilterUtilities::lookupTransformSafe(tf_buffer_,
                                                                 base_link_frame_id_,
                                                                 gps_frame_id_,
                                                                 transform_time,
                                                                 transform_timeout_,
                                                                 gps_offset_rotated);

    if (can_transform)
    {
      tf2::Transform robot_orientation;
      can_transform = RosFilterUtilities::lookupTransformSafe(tf_buffer_,
                                                              world_frame_id_,
                                                              base_link_frame_id_,
                                                              transform_time,
                                                              transform_timeout_,
                                                              robot_orientation);

      if (can_transform)
      {
        // Zero out rotation because we don't care about the orientation of the
        // GPS receiver relative to base_link
        gps_offset_rotated.setOrigin(tf2::quatRotate(robot_orientation.getRotation(), gps_offset_rotated.getOrigin()));
        gps_offset_rotated.setRotation(tf2::Quaternion::getIdentity());
        robot_odom_pose = gps_offset_rotated.inverse() * gps_odom_pose;
      }
      else
      {
        ROS_WARN_STREAM_THROTTLE(5.0, "Could not obtain " << world_frame_id_ << "->" << base_link_frame_id_ <<
          " transform. Will not remove offset of navsat device from robot's origin.");
      }
    }
    else
    {
      ROS_WARN_STREAM_THROTTLE(5.0, "Could not obtain " << base_link_frame_id_ << "->" << gps_frame_id_ <<
        " transform. Will not remove offset of navsat device from robot's origin.");
    }
  }

  void NavSatTransform::gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg)
  {
    gps_frame_id_ = msg->header.frame_id;

    if (gps_frame_id_.empty())
    {
      ROS_WARN_STREAM_ONCE("NavSatFix message has empty frame_id. Will assume navsat device is mounted at robot's "
        "origin.");
    }

    // Make sure the GPS data is usable
    bool good_gps = (msg->status.status != sensor_msgs::NavSatStatus::STATUS_NO_FIX &&
                     !std::isnan(msg->altitude) &&
                     !std::isnan(msg->latitude) &&
                     !std::isnan(msg->longitude));

    if (good_gps)
    {
      // If we haven't computed the transform yet, then
      // store this message as the initial GPS data to use
      if (!transform_good_ && !use_manual_datum_)
      {
        setTransformGps(msg);
      }

      double utmX = 0;
      double utmY = 0;
      std::string utm_zone_tmp;
      NavsatConversions::LLtoUTM(msg->latitude, msg->longitude, utmY, utmX, utm_zone_tmp);
      latest_utm_pose_.setOrigin(tf2::Vector3(utmX, utmY, msg->altitude));
      latest_utm_covariance_.setZero();

      // Copy the measurement's covariance matrix so that we can rotate it later
      for (size_t i = 0; i < POSITION_SIZE; i++)
      {
        for (size_t j = 0; j < POSITION_SIZE; j++)
        {
          latest_utm_covariance_(i, j) = msg->position_covariance[POSITION_SIZE * i + j];
        }
      }

      gps_update_time_ = msg->header.stamp;
      gps_updated_ = true;
    }
  }

  void NavSatTransform::imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    // We need the baseLinkFrameId_ from the odometry message, so
    // we need to wait until we receive it.
    if (has_transform_odom_)
    {
      /* This method only gets called if we don't yet have the
       * IMU data (the subscriber gets shut down once we compute
       * the transform), so we can assumed that every IMU message
       * that comes here is meant to be used for that purpose. */
      tf2::fromMsg(msg->orientation, transform_orientation_);

      // Correct for the IMU's orientation w.r.t. base_link
      tf2::Transform target_frame_trans;
      bool can_transform = RosFilterUtilities::lookupTransformSafe(tf_buffer_,
                                                                   base_link_frame_id_,
                                                                   msg->header.frame_id,
                                                                   msg->header.stamp,
                                                                   transform_timeout_,
                                                                   target_frame_trans);

      if (can_transform)
      {
        double roll_offset = 0;
        double pitch_offset = 0;
        double yaw_offset = 0;
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        RosFilterUtilities::quatToRPY(target_frame_trans.getRotation(), roll_offset, pitch_offset, yaw_offset);
        RosFilterUtilities::quatToRPY(transform_orientation_, roll, pitch, yaw);

        ROS_DEBUG_STREAM("Initial orientation is " << transform_orientation_);

        // Apply the offset (making sure to bound them), and throw them in a vector
        tf2::Vector3 rpy_angles(FilterUtilities::clampRotation(roll - roll_offset),
                                FilterUtilities::clampRotation(pitch - pitch_offset),
                                FilterUtilities::clampRotation(yaw - yaw_offset));

        // Now we need to rotate the roll and pitch by the yaw offset value.
        // Imagine a case where an IMU is mounted facing sideways. In that case
        // pitch for the IMU's world frame is roll for the robot.
        tf2::Matrix3x3 mat;
        mat.setRPY(0.0, 0.0, yaw_offset);
        rpy_angles = mat * rpy_angles;
        transform_orientation_.setRPY(rpy_angles.getX(), rpy_angles.getY(), rpy_angles.getZ());

        ROS_DEBUG_STREAM("Initial corrected orientation roll, pitch, yaw is (" <<
                         rpy_angles.getX() << ", " << rpy_angles.getY() << ", " << rpy_angles.getZ() << ")");

        has_transform_imu_ = true;
      }
    }
  }

  void NavSatTransform::odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    world_frame_id_ = msg->header.frame_id;
    base_link_frame_id_ = msg->child_frame_id;

    if (!transform_good_ && !use_manual_datum_)
    {
      setTransformOdometry(msg);
    }

    tf2::fromMsg(msg->pose.pose, latest_world_pose_);
    latest_odom_covariance_.setZero();
    for (size_t row = 0; row < POSE_SIZE; ++row)
    {
      for (size_t col = 0; col < POSE_SIZE; ++col)
      {
        latest_odom_covariance_(row, col) = msg->pose.covariance[row * POSE_SIZE + col];
      }
    }

    odom_update_time_ = msg->header.stamp;
    odom_updated_ = true;
  }


  bool NavSatTransform::prepareFilteredGps(sensor_msgs::NavSatFix &filtered_gps)
  {
    bool new_data = false;

    if (transform_good_ && odom_updated_)
    {
      tf2::Transform odom_as_utm;

      odom_as_utm.mult(utm_world_trans_inverse_, latest_world_pose_);
      odom_as_utm.setRotation(tf2::Quaternion::getIdentity());

      // Rotate the covariance as well
      tf2::Matrix3x3 rot(utm_world_trans_inverse_.getRotation());
      Eigen::MatrixXd rot_6d(POSE_SIZE, POSE_SIZE);
      rot_6d.setIdentity();

      for (size_t rInd = 0; rInd < POSITION_SIZE; ++rInd)
      {
        rot_6d(rInd, 0) = rot.getRow(rInd).getX();
        rot_6d(rInd, 1) = rot.getRow(rInd).getY();
        rot_6d(rInd, 2) = rot.getRow(rInd).getZ();
        rot_6d(rInd+POSITION_SIZE, 3) = rot.getRow(rInd).getX();
        rot_6d(rInd+POSITION_SIZE, 4) = rot.getRow(rInd).getY();
        rot_6d(rInd+POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
      }

      // Rotate the covariance
      latest_odom_covariance_ = rot_6d * latest_odom_covariance_.eval() * rot_6d.transpose();

      // Now convert the data back to lat/long and place into the message
      NavsatConversions::UTMtoLL(odom_as_utm.getOrigin().getY(),
                                 odom_as_utm.getOrigin().getX(),
                                 utm_zone_,
                                 filtered_gps.latitude,
                                 filtered_gps.longitude);
      filtered_gps.altitude = odom_as_utm.getOrigin().getZ();

      // Copy the measurement's covariance matrix back
      for (size_t i = 0; i < POSITION_SIZE; i++)
      {
        for (size_t j = 0; j < POSITION_SIZE; j++)
        {
          filtered_gps.position_covariance[POSITION_SIZE * i + j] = latest_odom_covariance_(i, j);
        }
      }

      filtered_gps.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN;
      filtered_gps.status.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
      filtered_gps.header.frame_id = "gps";
      filtered_gps.header.stamp = odom_update_time_;

      // Mark this GPS as used
      odom_updated_ = false;
      new_data = true;
    }

    return new_data;
  }

  bool NavSatTransform::prepareGpsOdometry(nav_msgs::Odometry &gps_odom)
  {
    bool new_data = false;

    if (transform_good_ && gps_updated_ && odom_updated_)
    {
      tf2::Transform transformed_utm_gps;

      transformed_utm_gps.mult(utm_world_transform_, latest_utm_pose_);
      transformed_utm_gps.setRotation(tf2::Quaternion::getIdentity());

      // Set header information stamp because we would like to know the robot's position at that timestamp
      gps_odom.header.frame_id = world_frame_id_;
      gps_odom.header.stamp = gps_update_time_;

      // Want the pose of the vehicle origin, not the GPS
      tf2::Transform transformed_utm_robot;
      getRobotOriginWorldPose(transformed_utm_gps, transformed_utm_robot, gps_odom.header.stamp);

      // Rotate the covariance as well
      tf2::Matrix3x3 rot(utm_world_transform_.getRotation());
      Eigen::MatrixXd rot_6d(POSE_SIZE, POSE_SIZE);
      rot_6d.setIdentity();

      for (size_t rInd = 0; rInd < POSITION_SIZE; ++rInd)
      {
        rot_6d(rInd, 0) = rot.getRow(rInd).getX();
        rot_6d(rInd, 1) = rot.getRow(rInd).getY();
        rot_6d(rInd, 2) = rot.getRow(rInd).getZ();
        rot_6d(rInd+POSITION_SIZE, 3) = rot.getRow(rInd).getX();
        rot_6d(rInd+POSITION_SIZE, 4) = rot.getRow(rInd).getY();
        rot_6d(rInd+POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
      }

      // Rotate the covariance
      latest_utm_covariance_ = rot_6d * latest_utm_covariance_.eval() * rot_6d.transpose();

      // Now fill out the message. Set the orientation to the identity.
      tf2::toMsg(transformed_utm_robot, gps_odom.pose.pose);
      gps_odom.pose.pose.position.z = (zero_altitude_ ? 0.0 : gps_odom.pose.pose.position.z);

      // Copy the measurement's covariance matrix so that we can rotate it later
      for (size_t i = 0; i < POSE_SIZE; i++)
      {
        for (size_t j = 0; j < POSE_SIZE; j++)
        {
          gps_odom.pose.covariance[POSE_SIZE * i + j] = latest_utm_covariance_(i, j);
        }
      }

      // Mark this GPS as used
      gps_updated_ = false;
      new_data = true;
    }

    return new_data;
  }

  void NavSatTransform::setTransformGps(const sensor_msgs::NavSatFixConstPtr& msg)
  {
    double utm_x = 0;
    double utm_y = 0;
    NavsatConversions::LLtoUTM(msg->latitude, msg->longitude, utm_y, utm_x, utm_zone_);

    ROS_INFO_STREAM("Datum (latitude, longitude, altitude) is (" << std::fixed << msg->latitude << ", " <<
                    msg->longitude << ", " << msg->altitude << ")");
    ROS_INFO_STREAM("Datum UTM coordinate is (" << std::fixed << utm_x << ", " << utm_y << ")");

    transform_utm_pose_.setOrigin(tf2::Vector3(utm_x, utm_y, msg->altitude));
    transform_utm_pose_.setRotation(tf2::Quaternion::getIdentity());
    has_transform_gps_ = true;
  }

  void NavSatTransform::setTransformOdometry(const nav_msgs::OdometryConstPtr& msg)
  {
    tf2::fromMsg(msg->pose.pose, transform_world_pose_);
    has_transform_odom_ = true;

    ROS_INFO_STREAM("Initial odometry pose is " << transform_world_pose_);

    // Users can optionally use the (potentially fused) heading from
    // the odometry source, which may have multiple fused sources of
    // heading data, and so would act as a better heading for the
    // UTM->world_frame transform.
    if (!transform_good_ && use_odometry_yaw_ && !use_manual_datum_)
    {
      sensor_msgs::Imu *imu = new sensor_msgs::Imu();
      imu->orientation = msg->pose.pose.orientation;
      imu->header.frame_id = msg->child_frame_id;
      imu->header.stamp = msg->header.stamp;
      sensor_msgs::ImuConstPtr imuPtr(imu);
      imuCallback(imuPtr);
    }
  }

}  // namespace RobotLocalization
