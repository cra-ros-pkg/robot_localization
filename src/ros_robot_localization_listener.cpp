/*
 * Copyright (c) 2016, TNO IVS Helmond.
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

#include <exception>
#include <functional>
#include <string>
#include <vector>

#include <rclcpp/qos.hpp>

#include "robot_localization/ros_robot_localization_listener.hpp"
#include "robot_localization/ros_filter_utilities.hpp"

#include <Eigen/Dense>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace robot_localization
{
FilterType filterTypeFromString(const std::string& filter_type_str)
{
  if ( filter_type_str == "ekf" )
  {
    return FilterTypes::EKF;
  }
  else if ( filter_type_str == "ukf" )
  {
    return FilterTypes::UKF;
  }
  else
  {
    return FilterTypes::NotDefined;
  }
}

RosRobotLocalizationListener::RosRobotLocalizationListener(
  rclcpp::Node::SharedPtr node, const std::string& ns):
  qos1_(1),
  qos10_(10),
  odom_sub_(node, "~/" + ns + "/odom", qos1_.get_rmw_qos_profile()),
  accel_sub_(node, "~/" + ns + "/acceleration", qos1_.get_rmw_qos_profile()),
  sync_(odom_sub_, accel_sub_, 10u),
  node_clock_(node->get_node_clock_interface()->get_clock()),
  node_logger_(node->get_node_logging_interface()),
  base_frame_id_(""),
  tf_buffer_(node_clock_),
  tf_listener_(tf_buffer_)
{
  int buffer_size = node->declare_parameter<int>("~/buffer_size", 10);

  std::string filter_type_str = node->declare_parameter<std::string>(
    "~/filter_type", std::string("ekf"));
  FilterType filter_type = filterTypeFromString(filter_type_str);
  if ( filter_type == FilterTypes::NotDefined )
  {
    RCLCPP_ERROR(node_logger_->get_logger(),
      "RosRobotLocalizationListener: Parameter filter_type invalid");
    return;
  }

  // Load up the process noise covariance (from the launch file/parameter server)
  // todo: this code is copied from ros_filter. In a refactor, this could be moved to a function in ros_filter_utilities
  Eigen::MatrixXd process_noise_covariance(STATE_SIZE, STATE_SIZE);
  process_noise_covariance.setZero();
  std::vector<double> process_noise_covar_config;

  // Get the process noise from the parameter in the namespace of the filter node we're listening to.
  std::string process_noise_param_namespace = odom_sub_.getTopic().substr(0, odom_sub_.getTopic().find_last_of('/'));
  std::string process_noise_param = process_noise_param_namespace + "/process_noise_covariance";

  if (!node->has_parameter(process_noise_param))
  {
    RCLCPP_ERROR(node_logger_->get_logger(),
      "Process noise covariance not found in the robot localization listener config (namespace %s)!",
      process_noise_param_namespace.c_str());
  }
  else
  {
    try
    {
      node->get_parameter<std::vector<double>>(process_noise_param, process_noise_covar_config);
      if (process_noise_covar_config.size() != STATE_SIZE * STATE_SIZE)
      {
        RCLCPP_ERROR(node_logger_->get_logger(),
          "ERROR unexpected process noise covariance matrix size (%d)",
          process_noise_covar_config.size());
      }

      int mat_size = process_noise_covariance.rows();

      for (int i = 0; i < mat_size; i++)
      {
        for (int j = 0; j < mat_size; j++)
        {
          // These matrices can cause problems if all the types
          // aren't specified with decimal points. Handle that
          // using string streams.
          std::ostringstream ostr;
          ostr << process_noise_covar_config[mat_size * i + j];
          std::istringstream istr(ostr.str());
          istr >> process_noise_covariance(i, j);
        }
      }

      std::stringstream pnc_ss;
      pnc_ss << process_noise_covariance;
      RCLCPP_DEBUG(node_logger_->get_logger(),
        "Process noise covariance is:\n%s\n", pnc_ss.str().c_str());
    }
    catch (std::exception& e)
    {
      RCLCPP_ERROR(node_logger_->get_logger(),
        "ERROR reading robot localization listener config: %s"
        " for process_noise_covariance", e.what());
    }
  }

  std::vector<double> filter_args = node->declare_parameter<
    std::vector<double>>("~/filter_args", std::vector<double>());

  estimator_ = new RobotLocalizationEstimator(buffer_size, filter_type, process_noise_covariance, filter_args);

  sync_.registerCallback(std::bind(
    &RosRobotLocalizationListener::odomAndAccelCallback,
    this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(node_logger_->get_logger(),
    "Ros Robot Localization Listener: Listening to topics %s and %s",
    odom_sub_.getTopic().c_str(), accel_sub_.getTopic().c_str());
}

RosRobotLocalizationListener::~RosRobotLocalizationListener()
{
  delete estimator_;
}

void RosRobotLocalizationListener::odomAndAccelCallback(
  const std::shared_ptr<nav_msgs::msg::Odometry const>& odom,
  const std::shared_ptr<geometry_msgs::msg::AccelWithCovarianceStamped const>& accel)
{
  // Instantiate a state that can be added to the robot localization estimator
  EstimatorState state;
  // Set its time stamp and the state received from the messages
  state.time_stamp = odom->header.stamp.sec;

  // Get the base frame id from the odom message
  if ( base_frame_id_.empty() )
    base_frame_id_ = odom->child_frame_id;

  // Pose: Position
  state.state(StateMemberX) = odom->pose.pose.position.x;
  state.state(StateMemberY) = odom->pose.pose.position.y;
  state.state(StateMemberZ) = odom->pose.pose.position.z;

  // Pose: Orientation
  tf2::Quaternion orientation_quat;
  tf2::fromMsg(odom->pose.pose.orientation, orientation_quat);
  double roll, pitch, yaw;
  ros_filter_utilities::quatToRPY(orientation_quat, roll, pitch, yaw);

  state.state(StateMemberRoll) = roll;
  state.state(StateMemberPitch) = pitch;
  state.state(StateMemberYaw) = yaw;

  // Pose: Covariance
  for ( unsigned int i = 0; i < POSE_SIZE; i++ )
  {
    for ( unsigned int j = 0; j < POSE_SIZE; j++ )
    {
      state.covariance(POSITION_OFFSET + i, POSITION_OFFSET + j) = odom->pose.covariance[i*POSE_SIZE + j];
    }
  }

  // Velocity: Linear
  state.state(StateMemberVx) = odom->twist.twist.linear.x;
  state.state(StateMemberVy) = odom->twist.twist.linear.y;
  state.state(StateMemberVz) = odom->twist.twist.linear.z;

  // Velocity: Angular
  state.state(StateMemberVroll) = odom->twist.twist.angular.x;
  state.state(StateMemberVpitch) = odom->twist.twist.angular.y;
  state.state(StateMemberVyaw) = odom->twist.twist.angular.z;

  // Velocity: Covariance
  for ( unsigned int i = 0; i < TWIST_SIZE; i++ )
  {
    for ( unsigned int j = 0; j < TWIST_SIZE; j++ )
    {
      state.covariance(POSITION_V_OFFSET + i, POSITION_V_OFFSET + j) = odom->twist.covariance[i*TWIST_SIZE + j];
    }
  }

  // Acceleration: Linear
  state.state(StateMemberAx) = accel->accel.accel.linear.x;
  state.state(StateMemberAy) = accel->accel.accel.linear.y;
  state.state(StateMemberAz) = accel->accel.accel.linear.z;

  // Acceleration: Angular is not available in state

  // Acceleration: Covariance
  for ( unsigned int i = 0; i < ACCELERATION_SIZE; i++ )
  {
    for ( unsigned int j = 0; j < ACCELERATION_SIZE; j++ )
    {
      state.covariance(POSITION_A_OFFSET + i, POSITION_A_OFFSET + j) = accel->accel.covariance[i*TWIST_SIZE + j];
    }
  }

  // Add the state to the buffer, so that we can later interpolate between this and earlier states
  estimator_->setState(state);

  return;
}

bool RosRobotLocalizationListener::getState(const double time,
                                            const std::string& frame_id,
                                            Eigen::VectorXd& state,
                                            Eigen::MatrixXd& covariance)
{
  EstimatorState estimator_state;
  state.resize(STATE_SIZE);
  state.setZero();
  covariance.resize(STATE_SIZE, STATE_SIZE);
  covariance.setZero();

  if ( base_frame_id_.empty() )
  {
    if ( estimator_->getSize() == 0 )
    {
      RCLCPP_WARN(node_logger_->get_logger(),
        "Ros Robot Localization Listener: The base frame id is not set. No odom/accel messages have come in.");
    }
    else
    {
      RCLCPP_ERROR(node_logger_->get_logger(),
        "Ros Robot Localization Listener: The base frame id is not set. Is the child_frame_id in the odom "
        "messages set?");
    }

    return false;
  }

  if ( estimator_->getState(time, estimator_state) == EstimatorResults::ExtrapolationIntoPast )
  {
    RCLCPP_WARN(node_logger_->get_logger(),
      "Ros Robot Localization Listener: A state is requested at a time stamp older than the oldest in the "
      "estimator buffer. The result is an extrapolation into the past. Maybe you should increase the buffer "
      "size?");
  }

  if ( frame_id == base_frame_id_ )
  {
    // If the state of the base frame is requested, we can simply return the state we got from the state estimator
    state = estimator_state.state;
    covariance = estimator_state.covariance;
    return true;
  }

  // - - - - - - - - - - - - - - - - - -
  // Calculate the state of the requested frame from the state of the base frame.
  // - - - - - - - - - - - - - - - - - -

  // First get the transform from base to target
  geometry_msgs::msg::TransformStamped base_to_target_transform;
  try
  {
    base_to_target_transform = tf_buffer_.lookupTransform(base_frame_id_,
                                                          frame_id,
                                                          tf2::TimePoint(std::chrono::nanoseconds(static_cast<int>(time * 1000000000))),
                                                          tf2::durationFromSec(0.1)); // TODO: magic number
  }
  catch ( tf2::LookupException e )
  {
    RCLCPP_WARN(node_logger_->get_logger(),
      "Ros Robot Localization Listener: Could not look up transform: %s", e.what());
    return false;
  }

  // And convert it to an eigen Affine transformation
  Eigen::Affine3d target_pose_base = tf2::transformToEigen(base_to_target_transform);

  // Then convert the base pose to an Eigen Affine transformation
  Eigen::Vector3d base_position(estimator_state.state(StateMemberX),
                                estimator_state.state(StateMemberY),
                                estimator_state.state(StateMemberZ));

  Eigen::AngleAxisd roll_angle(estimator_state.state(StateMemberRoll), Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(estimator_state.state(StateMemberPitch), Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(estimator_state.state(StateMemberYaw), Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond base_orientation(yaw_angle * pitch_angle * roll_angle);

  Eigen::Affine3d base_pose(Eigen::Translation3d(base_position) * base_orientation);

  // Now we can calculate the transform from odom to the requested frame (target)...
  Eigen::Affine3d target_pose_odom = base_pose * target_pose_base;

  // ... and put it in the output state
  state(StateMemberX) = target_pose_odom.translation().x();
  state(StateMemberY) = target_pose_odom.translation().y();
  state(StateMemberZ) = target_pose_odom.translation().z();

  Eigen::Vector3d ypr = target_pose_odom.rotation().eulerAngles(2, 1, 0);

  state(StateMemberRoll)  = ypr[2];
  state(StateMemberPitch) = ypr[1];
  state(StateMemberYaw)   = ypr[0];

  // Now let's calculate the twist of the target frame
  // First get the base's twist
  Twist base_velocity;
  Twist target_velocity_base;
  base_velocity.linear = Eigen::Vector3d(estimator_state.state(StateMemberVx),
                                         estimator_state.state(StateMemberVy),
                                         estimator_state.state(StateMemberVz));
  base_velocity.angular = Eigen::Vector3d(estimator_state.state(StateMemberVroll),
                                          estimator_state.state(StateMemberVpitch),
                                          estimator_state.state(StateMemberVyaw));

  // Then calculate the target frame's twist as a result of the base's twist.
  /*
   * We first calculate the coordinates of the velocity vectors (linear and angular) in the base frame. We have to keep
   * in mind that a rotation of the base frame, together with the translational offset of the target frame from the base
   * frame, induces a translational velocity of the target frame.
   */
  target_velocity_base.linear = base_velocity.linear + base_velocity.angular.cross(target_pose_base.translation());
  target_velocity_base.angular = base_velocity.angular;

  // Now we can transform that to the target frame
  Twist target_velocity;
  target_velocity.linear = target_pose_base.rotation().transpose() * target_velocity_base.linear;
  target_velocity.angular = target_pose_base.rotation().transpose() * target_velocity_base.angular;

  state(StateMemberVx) = target_velocity.linear(0);
  state(StateMemberVy) = target_velocity.linear(1);
  state(StateMemberVz) = target_velocity.linear(2);

  state(StateMemberVroll) = target_velocity.angular(0);
  state(StateMemberVpitch) = target_velocity.angular(1);
  state(StateMemberVyaw) = target_velocity.angular(2);

  // TODO: transform covariance
  covariance = estimator_state.covariance;

  return true;
}

bool RosRobotLocalizationListener::getState(const rclcpp::Time& rclcpp_time, const std::string& frame_id,
                                            Eigen::VectorXd& state, Eigen::MatrixXd& covariance)
{
  double time;
  if (rclcpp_time.nanoseconds() == 0)
  {
    RCLCPP_INFO(node_logger_->get_logger(),
      "Ros Robot Localization Listener: State requested at time = zero, returning state at current time");
    time = node_clock_->now().nanoseconds() / 1000000000;
  }
  else
  {
    time = rclcpp_time.nanoseconds() / 1000000000;
  }

  return getState(time, frame_id, state, covariance);
}
}  // namespace RobotLocalization

