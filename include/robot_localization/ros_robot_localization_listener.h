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

#ifndef ROBOT_LOCALIZATION_ROS_LISTENER_H
#define ROBOT_LOCALIZATION_ROS_LISTENER_H

#include "robot_localization/robot_localization_estimator.h"

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>

namespace RobotLocalization
{

class RosRobotLocalizationListener
{
  public:
    //! @brief Constructor
    //!
    //! The RosFilter constructor makes sure that anyone using
    //! this template is doing so with the correct object type
    //!
    explicit RosRobotLocalizationListener();

    //! @brief Destructor
    //!
    //! Clears out the message filters and topic subscribers.
    //!
    ~RosRobotLocalizationListener();

  private:
    //! @brief Robot Localization Estimator
    //!
    //! The core state estimator that facilitates inter- and
    //! extrapolation between buffered states.
    RobotLocalizationEstimator estimator_;

    //! @brief ROS Nodehandle
    //!
    //! A handle to the ROS node
    ros::NodeHandle nh_;

    //! @brief ROS Nodehandle
    //!
    //! A handle to the ROS node
    ros::NodeHandle nh_p_;

    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<geometry_msgs::AccelWithCovarianceStamped> accel_sub_;
    message_filters::TimeSynchronizer<nav_msgs::Odometry, geometry_msgs::AccelWithCovarianceStamped> sync_;

    void odomAndAccelCallback(const nav_msgs::Odometry& odom, const geometry_msgs::AccelWithCovarianceStamped& accel);

    void getState(const double time, Eigen::VectorXd& state, Eigen::MatrixXd& covariance);

    void getState(const ros::Time& ros_time, Eigen::VectorXd& state, Eigen::MatrixXd& covariance);
};

}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_ROS_FILTER_H
