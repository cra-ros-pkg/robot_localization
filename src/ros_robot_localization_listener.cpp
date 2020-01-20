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

#include "robot_localization/ros_robot_localization_listener.h"
#include "robot_localization/ros_filter_utilities.h"

#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>
#include <eigen_conversions/eigen_msg.h>

#include <XmlRpcException.h>

namespace RobotLocalization
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

RosRobotLocalizationListener::RosRobotLocalizationListener():
  nh_p_("robot_localization"),
  odom_sub_(nh_, "odometry/filtered", 1),
  accel_sub_(nh_, "accel/filtered", 1),
  sync_(odom_sub_, accel_sub_, 10),
  base_frame_id_(""),
  world_frame_id_(""),
  tf_listener_(tf_buffer_)
{
  int buffer_size;
  nh_p_.param("buffer_size", buffer_size, 10);

  std::string param_ns;
  nh_p_.param("parameter_namespace", param_ns, nh_p_.getNamespace());

  ros::NodeHandle nh_param(param_ns);

  std::string filter_type_str;
  nh_param.param("filter_type", filter_type_str, std::string("ekf"));
  FilterType filter_type = filterTypeFromString(filter_type_str);
  if ( filter_type == FilterTypes::NotDefined )
  {
    ROS_ERROR("RosRobotLocalizationListener: Parameter filter_type invalid");
    return;
  }

  // Load up the process noise covariance (from the launch file/parameter server)
  // TODO(reinzor): this code is copied from ros_filter. In a refactor, this could be moved to a function in
  // ros_filter_utilities
  Eigen::MatrixXd process_noise_covariance(STATE_SIZE, STATE_SIZE);
  process_noise_covariance.setZero();
  XmlRpc::XmlRpcValue process_noise_covar_config;

  if (!nh_param.hasParam("process_noise_covariance"))
  {
    ROS_FATAL_STREAM("Process noise covariance not found in the robot localization listener config (namespace " <<
                     nh_param.getNamespace() << ")! Use the ~parameter_namespace to specify the parameter namespace.");
  }
  else
  {
    try
    {
      nh_param.getParam("process_noise_covariance", process_noise_covar_config);

      ROS_ASSERT(process_noise_covar_config.getType() == XmlRpc::XmlRpcValue::TypeArray);

      int mat_size = process_noise_covariance.rows();

      for (int i = 0; i < mat_size; i++)
      {
        for (int j = 0; j < mat_size; j++)
        {
          try
          {
            // These matrices can cause problems if all the types
            // aren't specified with decimal points. Handle that
            // using string streams.
            std::ostringstream ostr;
            process_noise_covar_config[mat_size * i + j].write(ostr);
            std::istringstream istr(ostr.str());
            istr >> process_noise_covariance(i, j);
          }
          catch(XmlRpc::XmlRpcException &e)
          {
            throw e;
          }
          catch(...)
          {
            throw;
          }
        }
      }

      ROS_DEBUG_STREAM("Process noise covariance is:\n" << process_noise_covariance << "\n");
    }
    catch (XmlRpc::XmlRpcException &e)
    {
      ROS_ERROR_STREAM("ERROR reading robot localization listener config: " <<
                       e.getMessage() <<
                       " for process_noise_covariance (type: " <<
                       process_noise_covar_config.getType() << ")");
    }
  }

  std::vector<double> filter_args;
  nh_param.param("filter_args", filter_args, std::vector<double>());

  estimator_ = new RobotLocalizationEstimator(buffer_size, filter_type, process_noise_covariance, filter_args);

  sync_.registerCallback(&RosRobotLocalizationListener::odomAndAccelCallback, this);

  ROS_INFO_STREAM("Ros Robot Localization Listener: Listening to topics " <<
                  odom_sub_.getTopic() << " and " << accel_sub_.getTopic());

  // Wait until the base and world frames are set by the incoming messages
  while (ros::ok() && base_frame_id_.empty())
  {
    ros::spinOnce();
    ROS_INFO_STREAM_THROTTLE(1.0, "Ros Robot Localization Listener: Waiting for incoming messages on topics " <<
                             odom_sub_.getTopic() << " and " << accel_sub_.getTopic());
    ros::Duration(0.1).sleep();
  }
}

RosRobotLocalizationListener::~RosRobotLocalizationListener()
{
  delete estimator_;
}

void RosRobotLocalizationListener::odomAndAccelCallback(const nav_msgs::Odometry& odom,
                                                        const geometry_msgs::AccelWithCovarianceStamped& accel)
{
  // Instantiate a state that can be added to the robot localization estimator
  EstimatorState state;

  // Set its time stamp and the state received from the messages
  state.time_stamp = odom.header.stamp.toSec();

  // Get the base frame id from the odom message
  if ( base_frame_id_.empty() )
    base_frame_id_ = odom.child_frame_id;

  // Get the world frame id from the odom message
  if ( world_frame_id_.empty() )
    world_frame_id_ = odom.header.frame_id;

  // Pose: Position
  state.state(StateMemberX) = odom.pose.pose.position.x;
  state.state(StateMemberY) = odom.pose.pose.position.y;
  state.state(StateMemberZ) = odom.pose.pose.position.z;

  // Pose: Orientation
  tf2::Quaternion orientation_quat;
  tf2::fromMsg(odom.pose.pose.orientation, orientation_quat);
  double roll, pitch, yaw;
  RosFilterUtilities::quatToRPY(orientation_quat, roll, pitch, yaw);

  state.state(StateMemberRoll) = roll;
  state.state(StateMemberPitch) = pitch;
  state.state(StateMemberYaw) = yaw;

  // Pose: Covariance
  for ( unsigned int i = 0; i < POSE_SIZE; i++ )
  {
    for ( unsigned int j = 0; j < POSE_SIZE; j++ )
    {
      state.covariance(POSITION_OFFSET + i, POSITION_OFFSET + j) = odom.pose.covariance[i*POSE_SIZE + j];
    }
  }

  // Velocity: Linear
  state.state(StateMemberVx) = odom.twist.twist.linear.x;
  state.state(StateMemberVy) = odom.twist.twist.linear.y;
  state.state(StateMemberVz) = odom.twist.twist.linear.z;

  // Velocity: Angular
  state.state(StateMemberVroll) = odom.twist.twist.angular.x;
  state.state(StateMemberVpitch) = odom.twist.twist.angular.y;
  state.state(StateMemberVyaw) = odom.twist.twist.angular.z;

  // Velocity: Covariance
  for ( unsigned int i = 0; i < TWIST_SIZE; i++ )
  {
    for ( unsigned int j = 0; j < TWIST_SIZE; j++ )
    {
      state.covariance(POSITION_V_OFFSET + i, POSITION_V_OFFSET + j) = odom.twist.covariance[i*TWIST_SIZE + j];
    }
  }

  // Acceleration: Linear
  state.state(StateMemberAx) = accel.accel.accel.linear.x;
  state.state(StateMemberAy) = accel.accel.accel.linear.y;
  state.state(StateMemberAz) = accel.accel.accel.linear.z;

  // Acceleration: Angular is not available in state

  // Acceleration: Covariance
  for ( unsigned int i = 0; i < ACCELERATION_SIZE; i++ )
  {
    for ( unsigned int j = 0; j < ACCELERATION_SIZE; j++ )
    {
      state.covariance(POSITION_A_OFFSET + i, POSITION_A_OFFSET + j) = accel.accel.covariance[i*TWIST_SIZE + j];
    }
  }

  // Add the state to the buffer, so that we can later interpolate between this and earlier states
  estimator_->setState(state);

  return;
}

bool findAncestorRecursiveYAML(YAML::Node& tree, const std::string& source_frame, const std::string& target_frame)
{
  if ( source_frame == target_frame )
  {
    return true;
  }

  std::string parent_frame = tree[source_frame]["parent"].Scalar();

  if ( parent_frame.empty() )
  {
    return false;
  }

  return findAncestorRecursiveYAML(tree, parent_frame, target_frame);
}

// Cache, assumption that the tree parent child order does not change over time
static std::map<std::string, std::vector<std::string> > ancestor_map;
static std::map<std::string, std::vector<std::string> > descendant_map;
bool findAncestor(const tf2_ros::Buffer& buffer, const std::string& source_frame, const std::string& target_frame)
{
  // Check cache
  const std::vector<std::string>& ancestors = ancestor_map[source_frame];
  if (std::find(ancestors.begin(), ancestors.end(), target_frame) != ancestors.end())
  {
    return true;
  }
  const std::vector<std::string>& descendants = descendant_map[source_frame];
  if (std::find(descendants.begin(), descendants.end(), target_frame) != descendants.end())
  {
    return false;
  }

  std::stringstream frames_stream(buffer.allFramesAsYAML());
  YAML::Node frames_yaml = YAML::Load(frames_stream);

  bool target_frame_is_ancestor = findAncestorRecursiveYAML(frames_yaml, source_frame, target_frame);
  bool target_frame_is_descendant = findAncestorRecursiveYAML(frames_yaml, target_frame, source_frame);

  // Caching
  if (target_frame_is_ancestor)
  {
    ancestor_map[source_frame].push_back(target_frame);
  }
  if (target_frame_is_descendant)
  {
    descendant_map[source_frame].push_back(target_frame);
  }

  return target_frame_is_ancestor;
}


bool RosRobotLocalizationListener::getState(const double time,
                                            const std::string& frame_id,
                                            Eigen::VectorXd& state,
                                            Eigen::MatrixXd& covariance,
                                            std::string world_frame_id) const
{
  EstimatorState estimator_state;
  state.resize(STATE_SIZE);
  state.setZero();
  covariance.resize(STATE_SIZE, STATE_SIZE);
  covariance.setZero();

  if ( base_frame_id_.empty() || world_frame_id_.empty()  )
  {
    if ( estimator_->getSize() == 0 )
    {
      ROS_WARN("Ros Robot Localization Listener: The base or world frame id is not set. "
               "No odom/accel messages have come in.");
    }
    else
    {
      ROS_ERROR("Ros Robot Localization Listener: The base or world frame id is not set. "
                "Are the child_frame_id and the header.frame_id in the odom messages set?");
    }

    return false;
  }

  if ( estimator_->getState(time, estimator_state) == EstimatorResults::ExtrapolationIntoPast )
  {
    ROS_WARN("Ros Robot Localization Listener: A state is requested at a time stamp older than the oldest in the "
             "estimator buffer. The result is an extrapolation into the past. Maybe you should increase the buffer "
             "size?");
  }

  // If no world_frame_id is specified, we will default to the world frame_id of the received odometry message
  if (world_frame_id.empty())
  {
    world_frame_id = world_frame_id_;
  }

  if ( frame_id == base_frame_id_ && world_frame_id == world_frame_id_ )
  {
    // If the state of the base frame is requested and the world frame equals the world frame of the robot_localization
    // estimator, we can simply return the state we got from the state estimator
    state = estimator_state.state;
    covariance = estimator_state.covariance;
    return true;
  }

  // - - - - - - - - - - - - - - - - - -
  // Get the transformation between the requested world frame and the world_frame of the estimator
  // - - - - - - - - - - - - - - - - - -
  Eigen::Affine3d world_pose_requested_frame;

  // If the requested frame is the same as the tracker, set to identity
  if (world_frame_id == world_frame_id_)
  {
    world_pose_requested_frame.setIdentity();
  }
  else
  {
    geometry_msgs::TransformStamped world_requested_to_world_transform;
    try
    {
      // TODO(reinzor): magic number
      world_requested_to_world_transform = tf_buffer_.lookupTransform(world_frame_id,
                                                                      world_frame_id_,
                                                                      ros::Time(time),
                                                                      ros::Duration(0.1));

      if ( findAncestor(tf_buffer_, world_frame_id, base_frame_id_) )
      {
        ROS_ERROR_STREAM("You are trying to get the state with respect to world frame " << world_frame_id <<
                         ", but this frame is a child of robot base frame " << base_frame_id_ <<
                         ", so this doesn't make sense.");
        return false;
      }
    }
    catch ( const tf2::TransformException &e )
    {
      ROS_WARN_STREAM("Ros Robot Localization Listener: Could not look up transform: " << e.what());
      return false;
    }

    // Convert to pose
    tf::transformMsgToEigen(world_requested_to_world_transform.transform, world_pose_requested_frame);
  }

  // - - - - - - - - - - - - - - - - - -
  // Calculate the state of the requested frame from the state of the base frame.
  // - - - - - - - - - - - - - - - - - -

  // First get the transform from base to target
  geometry_msgs::TransformStamped base_to_target_transform;
  try
  {
    base_to_target_transform = tf_buffer_.lookupTransform(base_frame_id_,
                                                          frame_id,
                                                          ros::Time(time),
                                                          ros::Duration(0.1));  // TODO(reinzor): magic number

    // Check that frame_id is a child of the base frame. If it is not, it does not make sense to request its state.
    // Do this after tf lookup, so we know that there is a connection.
    if ( !findAncestor(tf_buffer_, frame_id, base_frame_id_) )
    {
      ROS_ERROR_STREAM("You are trying to get the state of " << frame_id << ", but this frame is not a child of the "
                                                                            "base frame: " << base_frame_id_ << ".");
      return false;
    }
  }
  catch ( const tf2::TransformException &e )
  {
    ROS_WARN_STREAM("Ros Robot Localization Listener: Could not look up transform: " << e.what());
    return false;
  }

  // And convert it to an eigen Affine transformation
  Eigen::Affine3d target_pose_base;
  tf::transformMsgToEigen(base_to_target_transform.transform, target_pose_base);

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
  Eigen::Affine3d target_pose_odom = world_pose_requested_frame * base_pose * target_pose_base;

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

  // Rotate the covariance as well
  Eigen::MatrixXd rot_6d(POSE_SIZE, POSE_SIZE);
  rot_6d.setIdentity();

  rot_6d.block<POSITION_SIZE, POSITION_SIZE>(POSITION_OFFSET, POSITION_OFFSET) = target_pose_base.rotation();
  rot_6d.block<ORIENTATION_SIZE, ORIENTATION_SIZE>(ORIENTATION_OFFSET, ORIENTATION_OFFSET) =
    target_pose_base.rotation();

  // Rotate the covariance
  covariance.block<POSE_SIZE, POSE_SIZE>(POSITION_OFFSET, POSITION_OFFSET) =
      rot_6d * estimator_state.covariance.block<POSE_SIZE, POSE_SIZE>(POSITION_OFFSET, POSITION_OFFSET) *
      rot_6d.transpose();

  return true;
}

bool RosRobotLocalizationListener::getState(const ros::Time& ros_time, const std::string& frame_id,
                                            Eigen::VectorXd& state, Eigen::MatrixXd& covariance,
                                            const std::string& world_frame_id) const
{
  double time;
  if ( ros_time.isZero() )
  {
    ROS_INFO("Ros Robot Localization Listener: State requested at time = zero, returning state at current time");
    time = ros::Time::now().toSec();
  }
  else
  {
    time = ros_time.toSec();
  }

  return getState(time, frame_id, state, covariance, world_frame_id);
}

const std::string& RosRobotLocalizationListener::getBaseFrameId() const
{
  return base_frame_id_;
}

const std::string& RosRobotLocalizationListener::getWorldFrameId() const
{
  return world_frame_id_;
}

}  // namespace RobotLocalization

