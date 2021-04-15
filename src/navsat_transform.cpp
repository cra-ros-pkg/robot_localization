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

#include <robot_localization/filter_common.hpp>
#include <robot_localization/filter_utilities.hpp>
#include <robot_localization/navsat_conversions.hpp>
#include <robot_localization/navsat_transform.hpp>
#include <robot_localization/ros_filter_utilities.hpp>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <utility>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

namespace robot_localization
{
NavSatTransform::NavSatTransform(const rclcpp::NodeOptions & options)
: Node("navsat_transform_node", options),
  base_link_frame_id_("base_link"),
  broadcast_cartesian_transform_(false),
  broadcast_cartesian_transform_as_parent_frame_(false),
  gps_frame_id_(""),
  gps_updated_(false),
  has_transform_gps_(false),
  has_transform_imu_(false),
  has_transform_odom_(false),
  magnetic_declination_(0.0),
  odom_updated_(false),
  publish_gps_(false),
  transform_good_(false),
  transform_timeout_(0ns),
  use_local_cartesian_(false),
  use_manual_datum_(false),
  use_odometry_yaw_(false),
  cartesian_broadcaster_(*this),
  utm_meridian_convergence_(0.0),
  utm_zone_(""),
  world_frame_id_("odom"),
  yaw_offset_(0.0),
  zero_altitude_(false)
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  latest_cartesian_covariance_.resize(POSE_SIZE, POSE_SIZE);
  latest_odom_covariance_.resize(POSE_SIZE, POSE_SIZE);

  double frequency = 10.0;
  double delay = 0.0;
  double transform_timeout = 0.0;

  // Load the parameters we need
  magnetic_declination_ = this->declare_parameter("magnetic_declination_radians", 0.0);
  yaw_offset_ = this->declare_parameter("yaw_offset", 0.0);
  zero_altitude_ = this->declare_parameter("zero_altitude", false);
  publish_gps_ = this->declare_parameter("publish_filtered_gps", true);
  use_odometry_yaw_ = this->declare_parameter("use_odometry_yaw", false);
  use_manual_datum_ = this->declare_parameter("wait_for_datum", false);
  use_local_cartesian_ = this->declare_parameter("use_local_cartesian", false);
  frequency = this->declare_parameter("frequency", frequency);
  delay = this->declare_parameter("delay", delay);
  transform_timeout = this->declare_parameter("transform_timeout", transform_timeout);

  transform_timeout_ = tf2::durationFromSec(transform_timeout);

  broadcast_cartesian_transform_ =
    this->declare_parameter("broadcast_utm_transform", broadcast_cartesian_transform_);

  if (broadcast_cartesian_transform_) {
    RCLCPP_WARN(
      this->get_logger(), "Parameter 'broadcast_utm_transform' has been deprecated. "
      "Please use 'broadcast_cartesian_transform' instead.");
  } else {
    broadcast_cartesian_transform_ =
      this->declare_parameter("broadcast_cartesian_transform", broadcast_cartesian_transform_);
  }

  broadcast_cartesian_transform_as_parent_frame_ =
    this->declare_parameter(
    "broadcast_utm_transform_as_parent_frame_",
    broadcast_cartesian_transform_as_parent_frame_);

  if (broadcast_cartesian_transform_as_parent_frame_) {
    RCLCPP_WARN(
      this->get_logger(), "Parameter 'broadcast_utm_transform_as_parent_frame' has been "
      "deprecated. Please use 'broadcast_cartesian_transform_as_parent_frame' instead.");
  } else {
    broadcast_cartesian_transform_as_parent_frame_ =
      this->declare_parameter(
      "broadcast_cartesian_transform_as_parent_frame",
      broadcast_cartesian_transform_as_parent_frame_);
  }

  datum_srv_ = this->create_service<robot_localization::srv::SetDatum>(
    "datum", std::bind(&NavSatTransform::datumCallback, this, _1, _2));

  to_ll_srv_ = this->create_service<robot_localization::srv::ToLL>(
    "toLL", std::bind(&NavSatTransform::toLLCallback, this, _1, _2));
  from_ll_srv_ = this->create_service<robot_localization::srv::FromLL>(
    "fromLL", std::bind(&NavSatTransform::fromLLCallback, this, _1, _2));

  std::vector<double> datum_vals;
  if (use_manual_datum_) {
    datum_vals = this->declare_parameter("datum", datum_vals);

    double datum_lat = 0.0;
    double datum_lon = 0.0;
    double datum_yaw = 0.0;

    if (datum_vals.size() == 3) {
      datum_lat = datum_vals[0];
      datum_lon = datum_vals[1];
      datum_yaw = datum_vals[2];
    }

    auto request = std::make_shared<robot_localization::srv::SetDatum::Request>();
    request->geo_pose.position.latitude = datum_lat;
    request->geo_pose.position.longitude = datum_lon;
    request->geo_pose.position.altitude = 0.0;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, datum_yaw);
    request->geo_pose.orientation = tf2::toMsg(quat);
    auto response = std::make_shared<robot_localization::srv::SetDatum::Response>();
    datumCallback(request, response);
  }

  auto custom_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odometry/filtered", custom_qos, std::bind(&NavSatTransform::odomCallback, this, _1));

  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "gps/fix", custom_qos, std::bind(&NavSatTransform::gpsFixCallback, this, _1));

  if (!use_odometry_yaw_ && !use_manual_datum_) {
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", custom_qos, std::bind(&NavSatTransform::imuCallback, this, _1));
  }

  gps_odom_pub_ =
    this->create_publisher<nav_msgs::msg::Odometry>("odometry/gps", rclcpp::QoS(10));

  if (publish_gps_) {
    filtered_gps_pub_ =
      this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/filtered", rclcpp::QoS(10));
  }

  // Sleep for the parameterized amount of time, to give
  // other nodes time to start up (not always necessary)
  rclcpp::sleep_for(
    std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::duration<double>(
        delay)));

  auto interval = std::chrono::duration<double>(1.0 / frequency);
  timer_ = this->create_wall_timer(interval, std::bind(&NavSatTransform::transformCallback, this));
}

NavSatTransform::~NavSatTransform() {}

void NavSatTransform::transformCallback()
{
  if (!transform_good_) {
    computeTransform();

    if (transform_good_ && !use_odometry_yaw_ && !use_manual_datum_) {
      // Once we have the transform, we don't need the IMU
      imu_sub_.reset();
    }
  } else {
    auto gps_odom = std::make_unique<nav_msgs::msg::Odometry>();
    if (prepareGpsOdometry(gps_odom.get())) {
      gps_odom_pub_->publish(std::move(gps_odom));
    }

    if (publish_gps_) {
      auto odom_gps = std::make_unique<sensor_msgs::msg::NavSatFix>();
      if (prepareFilteredGps(odom_gps.get())) {
        filtered_gps_pub_->publish(std::move(odom_gps));
      }
    }
  }
}

void NavSatTransform::computeTransform()
{
  // Only do this if:
  // 1. We haven't computed the odom_frame->cartesian_frame transform before
  // 2. We've received the data we need
  if (!transform_good_ && has_transform_odom_ && has_transform_gps_ &&
    has_transform_imu_)
  {
    // The UTM pose we have is given at the location of the GPS sensor on the
    // robot. We need to get the UTM pose of the robot's origin.
    tf2::Transform transform_cartesian_pose_corrected;
    if (!use_manual_datum_) {
      getRobotOriginCartesianPose(
        transform_cartesian_pose_, transform_cartesian_pose_corrected,
        rclcpp::Time(0));
    } else {
      transform_cartesian_pose_corrected = transform_cartesian_pose_;
    }

    // Get the IMU's current RPY values. Need the raw values (for yaw, anyway).
    tf2::Matrix3x3 mat(transform_orientation_);

    // Convert to RPY
    double imu_roll;
    double imu_pitch;
    double imu_yaw;
    mat.getRPY(imu_roll, imu_pitch, imu_yaw);

    /* The IMU's heading was likely originally reported w.r.t. magnetic north.
     * However, all the nodes in robot_localization assume that orientation
     * data, including that reported by IMUs, is reported in an ENU frame, with
     * a 0 yaw value being reported when facing east and increasing
     * counter-clockwise (i.e., towards north). To make the world frame ENU
     * aligned, where X is east and Y is north, we have to take into account
     * three additional considerations:
     *   1. The IMU may have its non-ENU frame data transformed to ENU, but
     *      there's a possibility that its data has not been corrected for
     *      magnetic declination. We need to account for this. A positive
     *      magnetic declination is counter-clockwise in an ENU frame.
     *      Therefore, if we have a magnetic declination of N radians, then when
     *      the sensor is facing a heading of N, it reports 0. Therefore, we
     *      need to add the declination angle.
     *   2. To account for any other offsets that may not be accounted for by
     *      the IMU driver or any interim processing node, we expose a yaw
     *      offset that lets users work with navsat_transform_node.
     *   3. UTM grid isn't aligned with True East\North. To account for the
     *      difference we need to add meridian convergence angle when using UTM.
     *      This value will be 0.0 when use_local_cartesian is TRUE.
     */
    imu_yaw += (magnetic_declination_ + yaw_offset_ +
      utm_meridian_convergence_);

    RCLCPP_INFO(
      this->get_logger(),
      "Corrected for magnetic declination of %g, "
      "user-specified offset of %g and meridian convergence of %g. "
      "Transform heading factor is now %g",
      magnetic_declination_, yaw_offset_, utm_meridian_convergence_, imu_yaw);

    // Convert to tf-friendly structures
    tf2::Quaternion imu_quat;
    imu_quat.setRPY(0.0, 0.0, imu_yaw);

    // The transform order will be orig_odom_pos * orig_cartesian_pos_inverse *
    // cur_cartesian_pos. Doing it this way will allow us to cope with having non-zero
    // odometry position when we get our first GPS message.
    tf2::Transform cartesian_pose_with_orientation;
    cartesian_pose_with_orientation.setOrigin(
      transform_cartesian_pose_corrected.getOrigin());
    cartesian_pose_with_orientation.setRotation(imu_quat);

    // Remove roll and pitch from odometry pose
    // Must be done because roll and pitch is removed from cartesian_pose_with_orientation
    double odom_roll {};
    double odom_pitch {};
    double odom_yaw {};
    tf2::Matrix3x3(transform_world_pose_.getRotation()).getRPY(odom_roll, odom_pitch, odom_yaw);
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0.0, 0.0, odom_yaw);
    tf2::Transform transform_world_pose_yaw_only(transform_world_pose_);
    transform_world_pose_yaw_only.setRotation(odom_quat);

    cartesian_world_transform_.mult(
      transform_world_pose_yaw_only,
      cartesian_pose_with_orientation.inverse());

    cartesian_world_trans_inverse_ = cartesian_world_transform_.inverse();

    transform_good_ = true;

    // Send out the (static) UTM transform in case anyone else would like to use
    // it.
    if (broadcast_cartesian_transform_) {
      geometry_msgs::msg::TransformStamped cartesian_transform_stamped;
      cartesian_transform_stamped.header.stamp = this->now();
      std::string cartesian_frame_id = (use_local_cartesian_ ? "local_enu" : "utm");
      cartesian_transform_stamped.header.frame_id =
        (broadcast_cartesian_transform_as_parent_frame_ ? cartesian_frame_id : world_frame_id_);
      cartesian_transform_stamped.child_frame_id =
        (broadcast_cartesian_transform_as_parent_frame_ ? world_frame_id_ : cartesian_frame_id);
      cartesian_transform_stamped.transform =
        (broadcast_cartesian_transform_as_parent_frame_ ?
        tf2::toMsg(cartesian_world_trans_inverse_) : tf2::toMsg(cartesian_world_transform_));
      cartesian_transform_stamped.transform.translation.z =
        (zero_altitude_ ? 0.0 : cartesian_transform_stamped.transform.translation.z);
      cartesian_broadcaster_.sendTransform(cartesian_transform_stamped);
    }
  }
}

bool NavSatTransform::datumCallback(
  robot_localization::srv::SetDatum::Request::SharedPtr request,
  robot_localization::srv::SetDatum::Response::SharedPtr)
{
  // If we get a service call with a manual datum, even if we already computed
  // the transform using the robot's initial pose, then we want to assume that
  // we are using a datum from now on, and we want other methods to not attempt
  // to transform the values we are specifying here.
  use_manual_datum_ = true;

  transform_good_ = false;

  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = request->geo_pose.position.latitude;
  fix.longitude = request->geo_pose.position.longitude;
  fix.altitude = request->geo_pose.position.altitude;
  fix.header.stamp = this->now();
  fix.position_covariance[0] = 0.1;
  fix.position_covariance[4] = 0.1;
  fix.position_covariance[8] = 0.1;
  fix.position_covariance_type = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  sensor_msgs::msg::NavSatFix::SharedPtr fix_ptr =
    std::make_shared<sensor_msgs::msg::NavSatFix>(fix);
  setTransformGps(fix_ptr);

  nav_msgs::msg::Odometry odom;
  odom.pose.pose.orientation.x = 0;
  odom.pose.pose.orientation.y = 0;
  odom.pose.pose.orientation.z = 0;
  odom.pose.pose.orientation.w = 1;
  odom.pose.pose.position.x = 0;
  odom.pose.pose.position.y = 0;
  odom.pose.pose.position.z = 0;
  odom.header.frame_id = world_frame_id_;
  odom.child_frame_id = base_link_frame_id_;
  nav_msgs::msg::Odometry::SharedPtr odom_ptr =
    std::make_shared<nav_msgs::msg::Odometry>(odom);
  setTransformOdometry(odom_ptr);

  sensor_msgs::msg::Imu imu;
  imu.orientation = request->geo_pose.orientation;
  imu.header.frame_id = base_link_frame_id_;
  sensor_msgs::msg::Imu::SharedPtr imu_ptr =
    std::make_shared<sensor_msgs::msg::Imu>(imu);
  imuCallback(imu_ptr);

  return true;
}

bool NavSatTransform::toLLCallback(
  const std::shared_ptr<robot_localization::srv::ToLL::Request> request,
  std::shared_ptr<robot_localization::srv::ToLL::Response> response)
{
  if (!transform_good_) {
    return false;
  }
  // tf2::Vector3 point;
  // tf2::fromMsg(request.map_point, point);
  tf2::Vector3 point(request->map_point.x, request->map_point.y,
    request->map_point.z);
  mapToLL(
    point, response->ll_point.latitude, response->ll_point.longitude,
    response->ll_point.altitude);

  return true;
}

bool NavSatTransform::fromLLCallback(
  const std::shared_ptr<robot_localization::srv::FromLL::Request> request,
  std::shared_ptr<robot_localization::srv::FromLL::Response> response)
{
  double altitude = request->ll_point.altitude;
  double longitude = request->ll_point.longitude;
  double latitude = request->ll_point.latitude;

  tf2::Transform cartesian_pose;

  double cartesian_x {};
  double cartesian_y {};
  double cartesian_z {};

  if (use_local_cartesian_) {
    gps_local_cartesian_.Forward(
      latitude,
      longitude,
      altitude,
      cartesian_x,
      cartesian_y,
      cartesian_z);
  } else {
    std::string utm_zone_tmp;
    navsat_conversions::LLtoUTM(
      latitude,
      longitude,
      cartesian_y,
      cartesian_x,
      utm_zone_tmp);
  }

  cartesian_pose.setOrigin(tf2::Vector3(cartesian_x, cartesian_y, altitude));

  nav_msgs::msg::Odometry gps_odom;

  if (!transform_good_) {
    return false;
  }

  response->map_point = cartesianToMap(cartesian_pose).pose.pose.position;

  return true;
}

nav_msgs::msg::Odometry NavSatTransform::cartesianToMap(
  const tf2::Transform & cartesian_pose) const
{
  nav_msgs::msg::Odometry gps_odom{};

  tf2::Transform transformed_cartesian_gps{};

  transformed_cartesian_gps.mult(cartesian_world_transform_, cartesian_pose);
  transformed_cartesian_gps.setRotation(tf2::Quaternion::getIdentity());

  // Set header information stamp because we would like to know the robot's
  // position at that timestamp
  gps_odom.header.frame_id = world_frame_id_;
  gps_odom.header.stamp = gps_update_time_;

  // Now fill out the message. Set the orientation to the identity.
  tf2::toMsg(transformed_cartesian_gps, gps_odom.pose.pose);
  gps_odom.pose.pose.position.z = (zero_altitude_ ? 0.0 :
    gps_odom.pose.pose.position.z);

  return gps_odom;
}

void NavSatTransform::mapToLL(
  const tf2::Vector3 & point,
  double & latitude,
  double & longitude,
  double & altitude) const
{
  tf2::Transform odom_as_cartesian{};

  tf2::Transform pose{};
  pose.setOrigin(point);
  pose.setRotation(tf2::Quaternion::getIdentity());

  odom_as_cartesian.mult(cartesian_world_trans_inverse_, pose);
  odom_as_cartesian.setRotation(tf2::Quaternion::getIdentity());

  // Now convert the data back to lat/long and place into the message
  navsat_conversions::UTMtoLL(
    odom_as_cartesian.getOrigin().getY(),
    odom_as_cartesian.getOrigin().getX(),
    utm_zone_,
    latitude,
    longitude);
  altitude = odom_as_cartesian.getOrigin().getZ();
}

void NavSatTransform::getRobotOriginCartesianPose(
  const tf2::Transform & gps_cartesian_pose, tf2::Transform & robot_cartesian_pose,
  const rclcpp::Time & transform_time)
{
  robot_cartesian_pose.setIdentity();

  // Get linear offset from origin for the GPS
  tf2::Transform offset;
  bool can_transform = ros_filter_utilities::lookupTransformSafe(
    tf_buffer_.get(), base_link_frame_id_, gps_frame_id_, transform_time,
    transform_timeout_, offset);

  if (can_transform) {
    // Get the orientation we'll use for our UTM->world transform
    tf2::Quaternion cartesian_orientation = transform_orientation_;
    tf2::Matrix3x3 mat(cartesian_orientation);

    // Add the offsets
    double roll;
    double pitch;
    double yaw;
    mat.getRPY(roll, pitch, yaw);
    yaw += (magnetic_declination_ + yaw_offset_ + utm_meridian_convergence_);
    cartesian_orientation.setRPY(roll, pitch, yaw);

    // Rotate the GPS linear offset by the orientation
    // Zero out the orientation, because the GPS orientation is meaningless, and
    // if it's non-zero, it will make the the computation of robot_cartesian_pose
    // erroneous.
    offset.setOrigin(tf2::quatRotate(cartesian_orientation, offset.getOrigin()));
    offset.setRotation(tf2::Quaternion::getIdentity());

    // Update the initial pose
    robot_cartesian_pose = offset.inverse() * gps_cartesian_pose;
  } else {
    if (gps_frame_id_ != "") {
      RCLCPP_ERROR(
        this->get_logger(),
        "Unable to obtain %s -> %s transform. "
        "Will assume navsat device is mounted at robots origin",
        base_link_frame_id_.c_str(), gps_frame_id_.c_str());
    }

    robot_cartesian_pose = gps_cartesian_pose;
  }
}

void NavSatTransform::getRobotOriginWorldPose(
  const tf2::Transform & gps_odom_pose, tf2::Transform & robot_odom_pose,
  const rclcpp::Time & transform_time)
{
  robot_odom_pose.setIdentity();

  // Remove the offset from base_link
  tf2::Transform gps_offset_rotated;
  bool can_transform = ros_filter_utilities::lookupTransformSafe(
    tf_buffer_.get(), base_link_frame_id_, gps_frame_id_, transform_time,
    transform_timeout_, gps_offset_rotated);

  if (can_transform) {
    tf2::Transform robot_orientation;
    can_transform = ros_filter_utilities::lookupTransformSafe(
      tf_buffer_.get(), world_frame_id_, base_link_frame_id_, transform_time,
      transform_timeout_, robot_orientation);

    if (can_transform) {
      // Zero out rotation because we don't care about the orientation of the
      // GPS receiver relative to base_link
      gps_offset_rotated.setOrigin(
        tf2::quatRotate(
          robot_orientation.getRotation(), gps_offset_rotated.getOrigin()));
      gps_offset_rotated.setRotation(tf2::Quaternion::getIdentity());
      robot_odom_pose = gps_offset_rotated.inverse() * gps_odom_pose;
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Could not obtain %s -> %s transform. "
        "Will not remove offset of navsat device from robot's origin",
        world_frame_id_.c_str(), base_link_frame_id_.c_str());
    }
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "Could not obtain %s -> %s transform. "
      "Will not remove offset of navsat device from robot's origin.",
      base_link_frame_id_.c_str(), gps_frame_id_.c_str());
  }
}

void NavSatTransform::gpsFixCallback(
  const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  gps_frame_id_ = msg->header.frame_id;

  if (gps_frame_id_.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "NavSatFix message has empty frame_id. "
      "Will assume navsat device is mounted at robot's origin");
  }

  // Make sure the GPS data is usable
  bool good_gps =
    (msg->status.status != sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX &&
    !std::isnan(msg->altitude) && !std::isnan(msg->latitude) &&
    !std::isnan(msg->longitude));

  if (good_gps) {
    // If we haven't computed the transform yet, then
    // store this message as the initial GPS data to use
    if (!transform_good_ && !use_manual_datum_) {
      setTransformGps(msg);
    }

    double cartesian_x = 0;
    double cartesian_y = 0;
    std::string cartesian_zone_tmp;
    navsat_conversions::LLtoUTM(
      msg->latitude,
      msg->longitude,
      cartesian_y,
      cartesian_x,
      cartesian_zone_tmp);
    latest_cartesian_pose_.setOrigin(tf2::Vector3(cartesian_x, cartesian_y, msg->altitude));
    latest_cartesian_covariance_.setZero();

    // Copy the measurement's covariance matrix so that we can rotate it later
    for (size_t i = 0; i < POSITION_SIZE; i++) {
      for (size_t j = 0; j < POSITION_SIZE; j++) {
        latest_cartesian_covariance_(i, j) =
          msg->position_covariance[POSITION_SIZE * i + j];
      }
    }

    gps_update_time_ = msg->header.stamp;
    gps_updated_ = true;
  }
}

void NavSatTransform::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // We need the baseLinkFrameId_ from the odometry message, so
  // we need to wait until we receive it.
  if (has_transform_odom_) {
    /* This method only gets called if we don't yet have the
     * IMU data (the subscriber gets shut down once we compute
     * the transform), so we can assumed that every IMU message
     * that comes here is meant to be used for that purpose. */
    tf2::fromMsg(msg->orientation, transform_orientation_);

    // Correct for the IMU's orientation w.r.t. base_link
    tf2::Transform target_frame_trans;
    bool can_transform = ros_filter_utilities::lookupTransformSafe(
      tf_buffer_.get(), base_link_frame_id_, msg->header.frame_id,
      msg->header.stamp, transform_timeout_, target_frame_trans);

    if (can_transform) {
      double roll_offset = 0;
      double pitch_offset = 0;
      double yaw_offset = 0;
      double roll = 0;
      double pitch = 0;
      double yaw = 0;
      ros_filter_utilities::quatToRPY(
        target_frame_trans.getRotation(),
        roll_offset, pitch_offset, yaw_offset);
      ros_filter_utilities::quatToRPY(transform_orientation_, roll, pitch, yaw);

      // Apply the offset (making sure to bound them), and throw them in a
      // vector
      tf2::Vector3 rpy_angles(
        filter_utilities::clampRotation(roll - roll_offset),
        filter_utilities::clampRotation(pitch - pitch_offset),
        filter_utilities::clampRotation(yaw - yaw_offset));

      // Now we need to rotate the roll and pitch by the yaw offset value.
      // Imagine a case where an IMU is mounted facing sideways. In that case
      // pitch for the IMU's world frame is roll for the robot.
      tf2::Matrix3x3 mat;
      mat.setRPY(0.0, 0.0, yaw_offset);
      rpy_angles = mat * rpy_angles;
      transform_orientation_.setRPY(
        rpy_angles.getX(), rpy_angles.getY(),
        rpy_angles.getZ());

      has_transform_imu_ = true;
    }
  }
}

void NavSatTransform::odomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  world_frame_id_ = msg->header.frame_id;
  base_link_frame_id_ = msg->child_frame_id;

  if (!transform_good_ && !use_manual_datum_) {
    setTransformOdometry(msg);
  }

  tf2::fromMsg(msg->pose.pose, latest_world_pose_);
  latest_odom_covariance_.setZero();
  for (size_t row = 0; row < POSE_SIZE; ++row) {
    for (size_t col = 0; col < POSE_SIZE; ++col) {
      latest_odom_covariance_(row, col) =
        msg->pose.covariance[row * POSE_SIZE + col];
    }
  }

  odom_update_time_ = msg->header.stamp;
  odom_updated_ = true;
}

bool NavSatTransform::prepareFilteredGps(
  sensor_msgs::msg::NavSatFix * filtered_gps)
{
  bool new_data = false;

  if (transform_good_ && odom_updated_) {
    mapToLL(
      latest_world_pose_.getOrigin(), filtered_gps->latitude,
      filtered_gps->longitude, filtered_gps->altitude);

    // Rotate the covariance as well
    tf2::Matrix3x3 rot(cartesian_world_trans_inverse_.getRotation());
    Eigen::MatrixXd rot_6d(POSE_SIZE, POSE_SIZE);
    rot_6d.setIdentity();

    for (size_t rInd = 0; rInd < POSITION_SIZE; ++rInd) {
      rot_6d(rInd, 0) = rot.getRow(rInd).getX();
      rot_6d(rInd, 1) = rot.getRow(rInd).getY();
      rot_6d(rInd, 2) = rot.getRow(rInd).getZ();
      rot_6d(rInd + POSITION_SIZE, 3) = rot.getRow(rInd).getX();
      rot_6d(rInd + POSITION_SIZE, 4) = rot.getRow(rInd).getY();
      rot_6d(rInd + POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
    }

    // Rotate the covariance
    latest_odom_covariance_ =
      rot_6d * latest_odom_covariance_.eval() * rot_6d.transpose();

    // Copy the measurement's covariance matrix back
    for (size_t i = 0; i < POSITION_SIZE; i++) {
      for (size_t j = 0; j < POSITION_SIZE; j++) {
        filtered_gps->position_covariance[POSITION_SIZE * i + j] =
          latest_odom_covariance_(i, j);
      }
    }

    filtered_gps->position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
    filtered_gps->status.status =
      sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
    filtered_gps->header.frame_id = base_link_frame_id_;
    filtered_gps->header.stamp = odom_update_time_;

    // Mark this GPS as used
    odom_updated_ = false;
    new_data = true;
  }

  return new_data;
}

bool NavSatTransform::prepareGpsOdometry(nav_msgs::msg::Odometry * gps_odom)
{
  bool new_data = false;

  if (transform_good_ && gps_updated_ && odom_updated_) {
    *gps_odom = cartesianToMap(latest_cartesian_pose_);

    tf2::Transform transformed_cartesian_gps;
    tf2::fromMsg(gps_odom->pose.pose, transformed_cartesian_gps);

    // Want the pose of the vehicle origin, not the GPS
    tf2::Transform transformed_cartesian_robot;
    rclcpp::Time time(static_cast<double>(gps_odom->header.stamp.sec) +
      static_cast<double>(gps_odom->header.stamp.nanosec) /
      1000000000.0);
    getRobotOriginWorldPose(transformed_cartesian_gps, transformed_cartesian_robot, time);

    // Rotate the covariance as well
    tf2::Matrix3x3 rot(cartesian_world_transform_.getRotation());
    Eigen::MatrixXd rot_6d(POSE_SIZE, POSE_SIZE);
    rot_6d.setIdentity();

    for (size_t rInd = 0; rInd < POSITION_SIZE; ++rInd) {
      rot_6d(rInd, 0) = rot.getRow(rInd).getX();
      rot_6d(rInd, 1) = rot.getRow(rInd).getY();
      rot_6d(rInd, 2) = rot.getRow(rInd).getZ();
      rot_6d(rInd + POSITION_SIZE, 3) = rot.getRow(rInd).getX();
      rot_6d(rInd + POSITION_SIZE, 4) = rot.getRow(rInd).getY();
      rot_6d(rInd + POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
    }

    // Rotate the covariance
    latest_cartesian_covariance_ =
      rot_6d * latest_cartesian_covariance_.eval() * rot_6d.transpose();

    // Now fill out the message. Set the orientation to the identity.
    tf2::toMsg(transformed_cartesian_robot, gps_odom->pose.pose);
    gps_odom->pose.pose.position.z =
      (zero_altitude_ ? 0.0 : gps_odom->pose.pose.position.z);

    // Copy the measurement's covariance matrix so that we can rotate it later
    for (size_t i = 0; i < POSE_SIZE; i++) {
      for (size_t j = 0; j < POSE_SIZE; j++) {
        gps_odom->pose.covariance[POSE_SIZE * i + j] =
          latest_cartesian_covariance_(i, j);
      }
    }

    // Mark this GPS as used
    gps_updated_ = false;
    new_data = true;
  }

  return new_data;
}

void NavSatTransform::setTransformGps(
  const sensor_msgs::msg::NavSatFix::SharedPtr & msg)
{
  double cartesian_x {};
  double cartesian_y {};
  double cartesian_z {};
  if (use_local_cartesian_) {
    const double hae_altitude {};
    gps_local_cartesian_.Reset(msg->latitude, msg->longitude, hae_altitude);
    gps_local_cartesian_.Forward(
      msg->latitude,
      msg->longitude,
      msg->altitude,
      cartesian_x,
      cartesian_y,
      cartesian_z);

    // UTM meridian convergence is not meaningful when using local cartesian, so set it to 0.0
    utm_meridian_convergence_ = 0.0;
  } else {
    navsat_conversions::LLtoUTM(
      msg->latitude,
      msg->longitude,
      cartesian_y,
      cartesian_x,
      utm_zone_,
      utm_meridian_convergence_);
    utm_meridian_convergence_ *= navsat_conversions::RADIANS_PER_DEGREE;
  }

  RCLCPP_INFO(
    this->get_logger(), "Datum (latitude, longitude, altitude) is (%0.2f, %0.2f, %0.2f)",
    msg->latitude, msg->longitude, msg->altitude);
  RCLCPP_INFO(
    this->get_logger(), "Datum %s coordinate is (%s, %0.2f, %0.2f)",
    ((use_local_cartesian_) ? "Local Cartesian" : "UTM"), utm_zone_.c_str(), cartesian_x,
    cartesian_y);

  transform_cartesian_pose_.setOrigin(tf2::Vector3(cartesian_x, cartesian_y, msg->altitude));
  transform_cartesian_pose_.setRotation(tf2::Quaternion::getIdentity());
  has_transform_gps_ = true;
}

void NavSatTransform::setTransformOdometry(
  const nav_msgs::msg::Odometry::SharedPtr & msg)
{
  tf2::fromMsg(msg->pose.pose, transform_world_pose_);
  has_transform_odom_ = true;

  // TODO(anyone) add back in Eloquent
  // ROS_INFO_STREAM_ONCE("Initial odometry pose is " << transform_world_pose_);

  // Users can optionally use the (potentially fused) heading from
  // the odometry source, which may have multiple fused sources of
  // heading data, and so would act as a better heading for the
  // UTM->world_frame transform.
  if (!transform_good_ && use_odometry_yaw_ && !use_manual_datum_) {
    sensor_msgs::msg::Imu imu;
    imu.orientation = msg->pose.pose.orientation;
    imu.header.frame_id = msg->child_frame_id;
    imu.header.stamp = msg->header.stamp;
    sensor_msgs::msg::Imu::SharedPtr imuPtr =
      std::make_shared<sensor_msgs::msg::Imu>(imu);
    imuCallback(imuPtr);
  }
}

}  // namespace robot_localization
