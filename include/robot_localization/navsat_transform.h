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

#ifndef ROBOT_LOCALIZATION_NAVSAT_TRANSFORM_H
#define ROBOT_LOCALIZATION_NAVSAT_TRANSFORM_H

#include <robot_localization/SetDatum.h>
#include <robot_localization/ToLL.h>
#include <robot_localization/FromLL.h>
#include <robot_localization/SetUTMZone.h>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <string>

namespace RobotLocalization
{

class NavSatTransform
{
  public:
    //! @brief Constructor
    //!
    NavSatTransform(ros::NodeHandle nh, ros::NodeHandle nh_priv);

    //! @brief Destructor
    //!
    ~NavSatTransform();

  private:
    //! @brief callback function which is called for periodic updates
    //!
    void periodicUpdate(const ros::TimerEvent& event);

    //! @brief Computes the transform from the UTM frame to the odom frame
    //!
    void computeTransform();

    //! @brief Callback for the datum service
    //!
    bool datumCallback(robot_localization::SetDatum::Request& request, robot_localization::SetDatum::Response&);

    //! @brief Callback for the to Lat Long service
    //!
    bool toLLCallback(robot_localization::ToLL::Request& request, robot_localization::ToLL::Response& response);

    //! @brief Callback for the from Lat Long service
    //!
    bool fromLLCallback(robot_localization::FromLL::Request& request, robot_localization::FromLL::Response& response);

    //! @brief Callback for the UTM zone service
    //!
    bool setUTMZoneCallback(robot_localization::SetUTMZone::Request& request,
                            robot_localization::SetUTMZone::Response& response);

    //! @brief Given the pose of the navsat sensor in the cartesian frame, removes the offset from the vehicle's
    //! centroid and returns the cartesian-frame pose of said centroid.
    //!
    void getRobotOriginCartesianPose(const tf2::Transform &gps_cartesian_pose,
                                     tf2::Transform &robot_cartesian_pose,
                                     const ros::Time &transform_time);

    //! @brief Given the pose of the navsat sensor in the world frame, removes the offset from the vehicle's centroid
    //! and returns the world-frame pose of said centroid.
    //!
    void getRobotOriginWorldPose(const tf2::Transform &gps_odom_pose,
                                 tf2::Transform &robot_odom_pose,
                                 const ros::Time &transform_time);

    //! @brief Callback for the GPS fix data
    //! @param[in] msg The NavSatFix message to process
    //!
    void gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg);

    //! @brief Callback for the IMU data
    //! @param[in] msg The IMU message to process
    //!
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);

    //! @brief Callback for the odom data
    //! @param[in] msg The odometry message to process
    //!
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

    //! @brief Converts the odometry data back to GPS and broadcasts it
    //! @param[out] filtered_gps The NavSatFix message to prepare
    //!
    bool prepareFilteredGps(sensor_msgs::NavSatFix &filtered_gps);

    //! @brief Prepares the GPS odometry message before sending
    //! @param[out] gps_odom The odometry message to prepare
    //!
    bool prepareGpsOdometry(nav_msgs::Odometry &gps_odom);

    //! @brief Used for setting the GPS data that will be used to compute the transform
    //! @param[in] msg The NavSatFix message to use in the transform
    //!
    void setTransformGps(const sensor_msgs::NavSatFixConstPtr& msg);

    //! @brief Used for setting the odometry data that will be used to compute the transform
    //! @param[in] msg The odometry message to use in the transform
    //!
    void setTransformOdometry(const nav_msgs::OdometryConstPtr& msg);

    //! @brief Transforms the passed in pose from utm to map frame
    //! @param[in] cartesian_pose the pose in cartesian frame to use to transform
    //!
    nav_msgs::Odometry cartesianToMap(const tf2::Transform& cartesian_pose) const;

    //! @brief Transforms the passed in point from map frame to lat/long
    //! @param[in] point the point in map frame to use to transform
    //!
    void mapToLL(const tf2::Vector3& point, double& latitude, double& longitude, double& altitude) const;

    //! @brief Whether or not we broadcast the cartesian transform
    //!
    bool broadcast_cartesian_transform_;

    //! @brief Whether to broadcast the cartesian transform as parent frame, default as child
    //!
    bool broadcast_cartesian_transform_as_parent_frame_;

    //! @brief Whether or not we have new GPS data
    //!
    //! We only want to compute and broadcast our transformed GPS data if it's new. This variable keeps track of that.
    //!
    bool gps_updated_;

    //! @brief Whether or not the GPS fix is usable
    //!
    bool has_transform_gps_;

    //! @brief Signifies that we have received a usable IMU message
    //!
    bool has_transform_imu_;

    //! @brief Signifies that we have received a usable odometry message
    //!
    bool has_transform_odom_;

    //! @brief Whether or not we have new odometry data
    //!
    //! If we're creating filtered GPS messages, then we only want to broadcast them when new odometry data arrives.
    //!
    bool odom_updated_;

    //! @brief Whether or not we publish filtered GPS messages
    //!
    bool publish_gps_;

    //! @brief Whether or not we've computed a good heading
    //!
    bool transform_good_;

    //! @brief Whether we get our datum from the first GPS message or from the set_datum
    //! service/parameter
    //!
    bool use_manual_datum_;

    //! @brief Whether we get the transform's yaw from the odometry or IMU source
    //!
    bool use_odometry_yaw_;

    //! @brief Whether we use a Local Cartesian (tangent plane ENU) or the UTM coordinates as our cartesian
    //!
    bool use_local_cartesian_;

    //! @brief Local Cartesian projection around gps origin
    //!
    GeographicLib::LocalCartesian gps_local_cartesian_;

    //! @brief Whether or not to report 0 altitude
    //!
    //! If this parameter is true, we always report 0 for the altitude of the converted GPS odometry message.
    //!
    bool zero_altitude_;

    //! @brief Parameter that specifies the magnetic declination for the robot's environment.
    //!
    double magnetic_declination_;

    //! @brief UTM's meridian convergence
    //!
    //! Angle between projected meridian (True North) and UTM's grid Y-axis.
    //! For UTM projection (Ellipsoidal Transverse Mercator) it is zero on the equator and non-zero everywhere else.
    //! It increases as the poles are approached or as we're getting farther from central meridian.
    //!
    double utm_meridian_convergence_;

    //! @brief IMU's yaw offset
    //!
    //! Your IMU should read 0 when facing *magnetic* north. If it doesn't, this (parameterized) value gives the offset
    //! (NOTE: if you have a magenetic declination, use the parameter setting for that).
    //!
    double yaw_offset_;

    //! @brief Frame ID of the robot's body frame
    //!
    //! This is needed for obtaining transforms from the robot's body frame to the frames of sensors (IMU and GPS)
    //!
    std::string base_link_frame_id_;

    //! @brief The frame_id of the GPS message (specifies mounting location)
    //!
    std::string gps_frame_id_;

    //! @brief the UTM zone (zero means UPS)
    //!
    int utm_zone_;

    //! @brief hemisphere (true means north, false means south)
    //!
    bool northp_;

    //! @brief Frame ID of the GPS odometry output
    //!
    //! This will just match whatever your odometry message has
    //!
    std::string world_frame_id_;

    //! @brief Covariance for most recent odometry data
    //!
    Eigen::MatrixXd latest_odom_covariance_;

    //! @brief Covariance for most recent GPS/UTM/LocalCartesian data
    //!
    Eigen::MatrixXd latest_cartesian_covariance_;

    //! @brief Timestamp of the latest good GPS message
    //!
    //! We assign this value to the timestamp of the odometry message that we output
    //!
    ros::Time gps_update_time_;

    //! @brief Timestamp of the latest good odometry message
    //!
    //! We assign this value to the timestamp of the odometry message that we output
    //!
    ros::Time odom_update_time_;

    //! @brief Parameter that specifies the how long we wait for a transform to become available.
    //!
    ros::Duration transform_timeout_;

    //! @brief timer calling periodicUpdate
    //!
    ros::Timer periodicUpdateTimer_;

    //! @brief Latest IMU orientation
    //!
    tf2::Quaternion transform_orientation_;

    //! @brief Latest GPS data, stored as Cartesian coords
    //!
    tf2::Transform latest_cartesian_pose_;

    //! @brief Latest odometry pose data
    //!
    tf2::Transform latest_world_pose_;

    //! @brief Holds the cartesian (UTM or local ENU) pose that is used to compute the transform
    //!
    tf2::Transform transform_cartesian_pose_;

    //! @brief Latest IMU orientation
    //!
    tf2::Transform transform_world_pose_;

    //! @brief Holds the Cartesian->odom transform
    //!
    tf2::Transform cartesian_world_transform_;

    //! @brief Holds the odom->UTM transform for filtered GPS broadcast
    //!
    tf2::Transform cartesian_world_trans_inverse_;

    //! @brief Publiser for filtered gps data
    //!
    ros::Publisher filtered_gps_pub_;

    //! @brief GPS subscriber
    //!
    ros::Subscriber gps_sub_;

    //! @brief Subscribes to imu topic
    //!
    ros::Subscriber imu_sub_;

    //! @brief Odometry subscriber
    //!
    ros::Subscriber odom_sub_;

    //! @brief Publisher for gps data
    //!
    ros::Publisher gps_odom_pub_;

    //! @brief Service for set datum
    //!
    ros::ServiceServer datum_srv_;

    //! @brief Service for to Lat Long
    //!
    ros::ServiceServer to_ll_srv_;

    //! @brief Service for from Lat Long
    //!
    ros::ServiceServer from_ll_srv_;

    //! @brief Service for set UTM zone
    //!
    ros::ServiceServer set_utm_zone_srv_;

    //! @brief Transform buffer for managing coordinate transforms
    //!
    tf2_ros::Buffer tf_buffer_;

    //! @brief Transform listener for receiving transforms
    //!
    tf2_ros::TransformListener tf_listener_;

    //! @brief Used for publishing the static world_frame->cartesian transform
    //!
    tf2_ros::StaticTransformBroadcaster cartesian_broadcaster_;
};

}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_NAVSAT_TRANSFORM_H
