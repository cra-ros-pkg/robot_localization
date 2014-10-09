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

#ifndef RobotLocalization_RosFilter_h
#define RobotLocalization_RosFilter_h

#include <robot_localization/filter_common.h>
#include <robot_localization/filter_base.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <XmlRpcException.h>

#include <Eigen/Dense>

#include <fstream>

// Some typedefs for message filter shared pointers
typedef boost::shared_ptr< message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> > poseMFSubPtr;
typedef boost::shared_ptr< message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped> > twistMFSubPtr;
typedef boost::shared_ptr< tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped> > poseMFPtr;
typedef boost::shared_ptr< tf::MessageFilter<geometry_msgs::TwistWithCovarianceStamped> > twistMFPtr;
typedef boost::shared_ptr< tf::MessageFilter<sensor_msgs::Imu> > imuMFPtr;

// Handy methods for debug output
std::ostream& operator<<(std::ostream& os, const tf::Vector3 &vec)
{
  os << "(" << std::setprecision(20) << vec.getX() << " " << vec.getY() << " " << vec.getZ() << ")\n";

  return os;
}

std::ostream& operator<<(std::ostream& os, const tf::Transform &trans)
{
  double roll, pitch, yaw;
  tf::Matrix3x3 orTmp(trans.getRotation());
  orTmp.getRPY(roll, pitch, yaw);

  os << "Origin: " << trans.getOrigin() <<
        "Rotation (RPY): (" << std::setprecision(20) << roll << ", " << pitch << ", " << yaw << ")\n";

  return os;
}

std::ostream& operator<<(std::ostream& os, const tf::Quaternion &quat)
{
  double roll, pitch, yaw;
  tf::Matrix3x3 orTmp(quat);
  orTmp.getRPY(roll, pitch, yaw);

  os << "(" << std::setprecision(20) << roll << ", " << pitch << ", " << yaw << ")\n";

  return os;
}

namespace RobotLocalization
{
  template<class Filter> class RosFilter
  {
    public:

      //! @brief Constructor.
      //!
      //! The RosFilter constructor makes sure that anyone using
      //! this template is doing so with the correct object type
      //!
      RosFilter() :
          nhLocal_("~")
      {
        // Ensure that anyone who uses this template uses the right
        // kind of template parameter type
        (void) static_cast<FilterBase*>((Filter*) 0);
      }

      ~RosFilter()
      {
        poseMessageFilters_.clear();
        twistMessageFilters_.clear();
        accelerationMessageFilters_.clear();
        poseTopicSubs_.clear();
        twistTopicSubs_.clear();
        imuTopicSubs_.clear();
        odomTopicSubs_.clear();
      }

      //! @brief Retrieves the EKF's output for broadcasting
      //! @param[out] message - The standard ROS odometry message to be filled
      //! @return true if the filter is initialized, false otherwise
      //!
      bool getFilteredOdometryMessage(nav_msgs::Odometry &message)
      {
        // If the filter has received a measurement at some point...
        if (filter_.getInitializedStatus())
        {
          // Grab our current state and covariance estimates
          const Eigen::VectorXd state = filter_.getState();
          const Eigen::MatrixXd estimateErrorCovariance = filter_.getEstimateErrorCovariance();

          // Convert from roll, pitch, and yaw back to quaternion for
          // orientation values
          tf::Quaternion quat;
          quat.setRPY(state(StateMemberRoll), state(StateMemberPitch), state(StateMemberYaw));

          // Fill out the message
          message.pose.pose.position.x = state(StateMemberX);
          message.pose.pose.position.y = state(StateMemberY);
          message.pose.pose.position.z = state(StateMemberZ);
          message.pose.pose.orientation.x = quat.x();
          message.pose.pose.orientation.y = quat.y();
          message.pose.pose.orientation.z = quat.z();
          message.pose.pose.orientation.w = quat.w();
          message.twist.twist.linear.x = state(StateMemberVx);
          message.twist.twist.linear.y = state(StateMemberVy);
          message.twist.twist.linear.z = state(StateMemberVz);
          message.twist.twist.angular.x = state(StateMemberVroll);
          message.twist.twist.angular.y = state(StateMemberVpitch);
          message.twist.twist.angular.z = state(StateMemberVyaw);

          // Our covariance matrix layout doesn't quite match
          for (size_t i = 0; i < POSE_SIZE; i++)
          {
            for (size_t j = 0; j < POSE_SIZE; j++)
            {
              message.pose.covariance[POSE_SIZE * i + j] = estimateErrorCovariance(i, j);
            }
          }

          // POSE_SIZE and TWIST_SIZE are currently the same size, but we can spare a few
          // cycles to be meticulous and not index a twist covariance array on the size of
          // a pose covariance array
          for (size_t i = 0; i < TWIST_SIZE; i++)
          {
            for (size_t j = 0; j < TWIST_SIZE; j++)
            {
              message.twist.covariance[TWIST_SIZE * i + j] = estimateErrorCovariance(i + POSITION_V_OFFSET, j + POSITION_V_OFFSET);
            }
          }

          message.header.stamp = ros::Time::now();
          message.header.frame_id = worldFrameId_;
          message.child_frame_id = baseLinkFrameId_;
        }

        return filter_.getInitializedStatus();
      }

      //! @brief Loads all parameters from file
      //!
      void loadParams()
      {
        // Grab the debug param. If true, the node will produce a LOT of output.
        bool debug;
        nhLocal_.param("debug", debug, false);

        nhLocal_.param("remove_acceleration_normal_force", removeAccNormalForce_, true);

        if (debug)
        {
          std::string debugOutFile;
          nhLocal_.param("debug_out_file", debugOutFile, std::string("robot_localization_debug.txt"));
          debugStream_.open(debugOutFile.c_str());

          // Make sure we succeeeded
          if(debugStream_.is_open())
          {
            filter_.setDebug(debug, &debugStream_);
          }
          else
          {
            ROS_WARN_STREAM("RosFilter::loadParams() - unable to create debug output file " << debugOutFile);
          }
        }

        // Try to resolve the tf_prefix
        nh_.param("/tf_prefix", tfPrefix_, std::string(""));

        if (tfPrefix_.empty())
        {
          nh_.param("tf_prefix", tfPrefix_, std::string(""));
        }

        // These params specify the name of the robot's body frame (typically
        // base_link) and odometry frame (typically odom)
        nhLocal_.param("map_frame", mapFrameId_, std::string("map"));
        nhLocal_.param("odom_frame", odomFrameId_, std::string("odom"));
        nhLocal_.param("base_link_frame", baseLinkFrameId_, std::string("base_link"));

        // These parameters are designed to enforce compliance with REP-105:
        // http://www.ros.org/reps/rep-0105.html
        // When fusing absolute position data from sensors such as GPS, the state
        // estimate can undergo discrete jumps. According to REP-105, we want three
        // coordinate frames: map, odom, and base_link. The map frame can have
        // discontinuities, but is the frame with the most accurate position estimate
        // for the robot and should not suffer from drift. The odom frame drifts over
        // time, but is guaranteed to be continuous and is accurate enough for local
        // planning and navigation. The base_link frame is affixed to the robot. The
        // intention is that some odometry source broadcasts the odom->base_link
        // transform. The localization software should broadcast map->base_link.
        // However, tf does not allow multiple parents for a coordinate frame, so
        // we must *compute* map->base_link, but then use the existing odom->base_link
        // transform to compute *and broadcast* map->odom.
        //
        // robot_localization therefore has two "modes." If your frame_id and
        // child_frame_id parameters match the map_frame and base_link_frame parameters,
        // respectively, then robot_localization will assume someone else is broadcasting
        // odom->base_link, and it will compute map->odom. If your frame_id and
        // child_frame_id parameters match the odom_frame and base_link frame,
        // respectively, then robot_localization will simply broadcast that transform,
        // thereby assuming nothing else is. This allows users to still fuse data
        // without having a map frame.
        //
        // The default is the latter behavior (fusion of odom->base_link)
        nhLocal_.param("world_frame", worldFrameId_, odomFrameId_);

        ROS_FATAL_COND(mapFrameId_ == odomFrameId_ ||
                       odomFrameId_ == baseLinkFrameId_ ||
                       mapFrameId_ == baseLinkFrameId_,
                       "Invalid frame configuration! The values for map_frame, odom_frame, and base_link_frame must be unique");

        // Append tf_prefix if it's specified (@todo: tf2 migration)
        if (!tfPrefix_.empty() && baseLinkFrameId_.at(0) != '/')
        {
          if(mapFrameId_.at(0) != '/')
          {
            mapFrameId_ = "/" + tfPrefix_ + "/" + mapFrameId_;
          }

          if(odomFrameId_.at(0) != '/')
          {
            odomFrameId_ = "/" + tfPrefix_ + "/" + odomFrameId_;
          }

          if(baseLinkFrameId_.at(0) != '/')
          {
            baseLinkFrameId_ = "/" + tfPrefix_ + "/" + baseLinkFrameId_;
          }
        }

        // Update frequency and sensor timeout
        double sensorTimeout;
        nhLocal_.param("frequency", frequency_, 30.0);
        nhLocal_.param("sensor_timeout", sensorTimeout, 1.0 / frequency_);
        filter_.setSensorTimeout(sensorTimeout);

        // Debugging writes to file
        if (filter_.getDebug())
        {
          debugStream_ << "tf_prefix is " << tfPrefix_ << "\n" << "odom_frame is " << odomFrameId_ << "\n" << "base_link_frame is " << baseLinkFrameId_
            << "\n" << "frequency is " << frequency_ << "\n" << "sensor_timeout is " << filter_.getSensorTimeout() << "\n";
        }

        // Create a subscriber for manually setting/resetting pose
        setPoseSub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("set_pose", 1, &RosFilter<Filter>::setPoseCallback, this);

        // Init the last last measurement time so we don't get a huge initial delta
        filter_.setLastMeasurementTime(ros::Time::now().toSec());
        filter_.setLastUpdateTime(ros::Time::now().toSec());

        // Now pull in each topic to which we want to subscribe.
        // Start with odom.
        size_t topicInd = 0;
        bool moreParams = false;
        do
        {
          // Build the string in the form of "odomX", where X is the odom topic number,
          // then check if we have any parameters with that value. Users need to make
          // sure they don't have gaps in their configs (e.g., odom0 and then odom2)
          std::stringstream ss;
          ss << "odom" << topicInd++;
          std::string odomTopicName = ss.str();
          moreParams = nhLocal_.hasParam(odomTopicName);

          if (moreParams)
          {
            // Determine if we want to integrate this sensor differentially
            bool differential;
            nhLocal_.param(odomTopicName + std::string("_differential"), differential, false);

            ////////// HANDLE DEPRECATED DIFFERENTIAL SETTING //////////
            if(!differential)
            {
              XmlRpc::XmlRpcValue diffConfig;

              if (nhLocal_.hasParam(odomTopicName + std::string("_differential")))
              {
                try
                {
                  nhLocal_.getParam(odomTopicName + std::string("_differential"), diffConfig);

                  if(diffConfig.getType() == XmlRpc::XmlRpcValue::TypeArray)
                  {
                    ROS_WARN_STREAM("Per-variable configuration of differential settings is deprecated. " <<
                                    "Please set the value for " << odomTopicName << std::string("_differential") <<
                                    " to \"true\" or \"false.\"");

                    // If any of the variables is true, make the whole sensor differential
                    for (int i = 0; i < diffConfig.size() && !differential; i++)
                    {
                      // The double cast looks strange, but we'll get exceptions if we
                      // remove the cast to bool. vector<bool> is discouraged, so updateVector
                      // uses integers
                      differential = differential || static_cast<int>(static_cast<bool>(diffConfig[i]));
                    }

                    if(differential)
                    {
                      ROS_WARN_STREAM("Setting " << odomTopicName << std::string("_differential") << " to true");
                    }
                  }
                }
                catch(...)
                {

                }
              }
            }
            ////////// END DEPRECATED DIFFERENTIAL SETTING //////////

            // Subscribe using boost::bind, which lets us append arbitrary data,
            // in this case, the topic name (e.g., odom0 or odom1)
            std::string odomTopic;
            nhLocal_.getParam(odomTopicName, odomTopic);

            // Now pull in its boolean update vector configuration. Create separate vectors for pose
            // and twist data, and then zero out the opposite values in each vector (no pose data in
            // the twist update vector and vice-versa).
            std::vector<int> updateVec = loadUpdateConfig(odomTopicName);
            std::vector<int> poseUpdateVec = updateVec;
            std::fill(poseUpdateVec.begin() + POSITION_V_OFFSET, poseUpdateVec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
            std::vector<int> twistUpdateVec = updateVec;
            std::fill(twistUpdateVec.begin() + POSITION_OFFSET, twistUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE, 0);

            // Store the odometry topic subscribers so they dont go out of scope. Also,
            // odometry data has both pose and twist data, each with their own frame_id.
            // The odometry data gets broken up and passed into callbacks for pose and
            // twist data, so we need to create message filters for them, and then
            // manually add the pose and twist messages after we extract them from the
            // odometry message.
            odomTopicSubs_.push_back(
                  nh_.subscribe<nav_msgs::Odometry>(odomTopic, 1,
                                                    boost::bind(&RosFilter<Filter>::odometryCallback, this, _1, odomTopicName, updateVec, differential)));

            poseMFPtr poseFilPtr(new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(tfListener_, worldFrameId_, 1));
            poseFilPtr->registerCallback(boost::bind(&RosFilter<Filter>::poseCallback, this, _1, odomTopicName, worldFrameId_, poseUpdateVec, differential));
            poseMessageFilters_[odomTopicName + "_pose"] = poseFilPtr;

            twistMFPtr twistFilPtr(new tf::MessageFilter<geometry_msgs::TwistWithCovarianceStamped>(tfListener_, baseLinkFrameId_, 1));
            twistFilPtr->registerCallback(boost::bind(&RosFilter<Filter>::twistCallback, this, _1, odomTopicName, baseLinkFrameId_, twistUpdateVec));
            twistMessageFilters_[odomTopicName + "_twist"] = twistFilPtr;

            if(filter_.getDebug())
            {
              debugStream_ << "Subscribed to " << odomTopic << "\n";
            }
          }
        } while (moreParams);

        // Repeat for pose
        topicInd = 0;
        moreParams = false;
        do
        {
          std::stringstream ss;
          ss << "pose" << topicInd++;
          std::string poseTopicName = ss.str();
          moreParams = nhLocal_.hasParam(poseTopicName);

          if (moreParams)
          {
            bool differential;
            nhLocal_.param(poseTopicName + std::string("_differential"), differential, false);

            ////////// HANDLE DEPRECATED DIFFERENTIAL SETTING //////////
            if(!differential)
            {
              XmlRpc::XmlRpcValue diffConfig;

              if (nhLocal_.hasParam(poseTopicName + std::string("_differential")))
              {
                try
                {
                  nhLocal_.getParam(poseTopicName + std::string("_differential"), diffConfig);

                  if(diffConfig.getType() == XmlRpc::XmlRpcValue::TypeArray)
                  {
                    ROS_WARN_STREAM("Per-variable configuration of differential settings is deprecated. " <<
                                    "Please set the value for " << poseTopicName << std::string("_differential") <<
                                    " to \"true\" or \"false.\"");

                    // If any of the variables is true, make the whole sensor differential
                    for (int i = 0; i < diffConfig.size() && !differential; i++)
                    {
                      // The double cast looks strange, but we'll get exceptions if we
                      // remove the cast to bool. vector<bool> is discouraged, so updateVector
                      // uses integers
                      differential = differential || static_cast<int>(static_cast<bool>(diffConfig[i]));
                    }

                    if(differential)
                    {
                      ROS_WARN_STREAM("Setting " << poseTopicName << std::string("_differential") << " to true");
                    }
                  }
                }
                catch(...)
                {

                }
              }
            }
            ////////// END DEPRECATED DIFFERENTIAL SETTING //////////

            std::string poseTopic;
            nhLocal_.getParam(poseTopicName, poseTopic);

            // Pull in the sensor's config, zero out values that are invalid for the pose type
            std::vector<int> updateVec = loadUpdateConfig(poseTopicName);
            std::vector<int> poseUpdateVec = updateVec;
            std::fill(poseUpdateVec.begin() + POSITION_V_OFFSET, poseUpdateVec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
            std::fill(poseUpdateVec.begin() + POSITION_A_OFFSET, poseUpdateVec.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE, 0);

            // Create and store message filter subscriber objects and message filters
            poseMFSubPtr subPtr(new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>());
            subPtr->subscribe(nh_, poseTopic, 1);
            poseMFPtr filPtr(new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*subPtr, tfListener_, worldFrameId_, 1));
            filPtr->registerCallback(boost::bind(&RosFilter<Filter>::poseCallback, this, _1, poseTopicName, worldFrameId_, poseUpdateVec, differential));
            poseTopicSubs_.push_back(subPtr);
            poseMessageFilters_[poseTopicName] = filPtr;

            if (filter_.getDebug())
            {
              debugStream_ << "Subscribed to " << poseTopic << "\n";
            }
          }
        } while (moreParams);

        // Repeat for twist
        topicInd = 0;
        moreParams = false;
        do
        {
          std::stringstream ss;
          ss << "twist" << topicInd++;
          std::string twistTopicName = ss.str();
          moreParams = nhLocal_.hasParam(twistTopicName);

          if (moreParams)
          {
            std::string twistTopic;
            nhLocal_.getParam(twistTopicName, twistTopic);

            // Pull in the sensor's config, zero out values that are invalid for the twist type
            std::vector<int> updateVec = loadUpdateConfig(twistTopicName);
            std::vector<int> twistUpdateVec = updateVec;
            std::fill(twistUpdateVec.begin() + POSITION_OFFSET, twistUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE, 0);

            // Create and store subscriptions and message filters
            twistMFSubPtr subPtr(new message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped>());
            subPtr->subscribe(nh_, twistTopic, 1);
            twistMFPtr filPtr(new tf::MessageFilter<geometry_msgs::TwistWithCovarianceStamped>(*subPtr, tfListener_, baseLinkFrameId_, 1));
            filPtr->registerCallback(boost::bind(&RosFilter<Filter>::twistCallback, this, _1, twistTopicName, baseLinkFrameId_, twistUpdateVec));
            twistTopicSubs_.push_back(subPtr);
            twistMessageFilters_[twistTopicName] = filPtr;

            if (filter_.getDebug())
            {
              debugStream_ << "Subscribed to " << twistTopic << "\n";
            }
          }
        } while (moreParams);

        // Repeat for IMU
        topicInd = 0;
        moreParams = false;
        do
        {
          std::stringstream ss;
          ss << "imu" << topicInd++;
          std::string imuTopicName = ss.str();
          moreParams = nhLocal_.hasParam(imuTopicName);

          if (moreParams)
          {
            bool differential;
            nhLocal_.param(imuTopicName + std::string("_differential"), differential, false);

            ////////// HANDLE DEPRECATED DIFFERENTIAL SETTING //////////
            if(!differential)
            {
              XmlRpc::XmlRpcValue diffConfig;

              if (nhLocal_.hasParam(imuTopicName + std::string("_differential")))
              {
                try
                {
                  nhLocal_.getParam(imuTopicName + std::string("_differential"), diffConfig);

                  if(diffConfig.getType() == XmlRpc::XmlRpcValue::TypeArray)
                  {
                    ROS_WARN_STREAM("Per-variable configuration of differential settings is deprecated. " <<
                                    "Please set the value for " << imuTopicName << std::string("_differential") <<
                                    " to \"true\" or \"false.\"");

                    // If any of the variables is true, make the whole sensor differential
                    for (int i = 0; i < diffConfig.size() && !differential; i++)
                    {
                      // The double cast looks strange, but we'll get exceptions if we
                      // remove the cast to bool. vector<bool> is discouraged, so updateVector
                      // uses integers
                      differential = differential || static_cast<int>(static_cast<bool>(diffConfig[i]));
                    }

                    if(differential)
                    {
                      ROS_WARN_STREAM("Setting " << imuTopicName << std::string("_differential") << " to true");
                    }
                  }
                }
                catch(...)
                {

                }
              }
            }
            ////////// END DEPRECATED DIFFERENTIAL SETTING //////////

            std::string imuTopic;
            nhLocal_.getParam(imuTopicName, imuTopic);

            // Now pull in its boolean update vector configuration and differential
            // update configuration (as this contains pose information)
            std::vector<int> updateVec = loadUpdateConfig(imuTopicName);

            std::vector<int> poseUpdateVec = updateVec;
            std::fill(poseUpdateVec.begin() + POSITION_V_OFFSET, poseUpdateVec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
            std::fill(poseUpdateVec.begin() + POSITION_A_OFFSET, poseUpdateVec.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE, 0);

            std::vector<int> twistUpdateVec = updateVec;
            std::fill(twistUpdateVec.begin() + POSITION_OFFSET, twistUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE, 0);
            std::fill(twistUpdateVec.begin() + POSITION_A_OFFSET, twistUpdateVec.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE, 0);

            std::vector<int> accelUpdateVec = updateVec;
            std::fill(accelUpdateVec.begin() + POSITION_OFFSET, accelUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE, 0);
            std::fill(accelUpdateVec.begin() + POSITION_V_OFFSET, accelUpdateVec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);

            // Create and store subscriptions and message filters as with odometry data
            imuTopicSubs_.push_back(
                  nh_.subscribe<sensor_msgs::Imu>(imuTopic, 1,
                                                  boost::bind(&RosFilter<Filter>::imuCallback, this, _1, imuTopicName, updateVec, differential)));

            // @todo: There's a lot of ambiguity with IMU frames. Should allow a parameter that specifies a target IMU frame.
            poseMFPtr poseFilPtr(new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(tfListener_, baseLinkFrameId_, 1));
            poseFilPtr->registerCallback(boost::bind(&RosFilter<Filter>::poseCallback, this, _1, imuTopicName, baseLinkFrameId_, poseUpdateVec, differential));
            poseMessageFilters_[imuTopicName + "_pose"] = poseFilPtr;

            twistMFPtr twistFilPtr(new tf::MessageFilter<geometry_msgs::TwistWithCovarianceStamped>(tfListener_, baseLinkFrameId_, 1));
            twistFilPtr->registerCallback(boost::bind(&RosFilter<Filter>::twistCallback, this, _1, imuTopicName, baseLinkFrameId_, twistUpdateVec));
            twistMessageFilters_[imuTopicName + "_twist"] = twistFilPtr;


            imuMFPtr accelFilPtr(new tf::MessageFilter<sensor_msgs::Imu>(tfListener_, baseLinkFrameId_, 1));
            accelFilPtr->registerCallback(boost::bind(&RosFilter<Filter>::accelerationCallback, this, _1, imuTopicName, baseLinkFrameId_, accelUpdateVec));
            accelerationMessageFilters_[imuTopicName + "_acceleration"] = accelFilPtr;

            if (filter_.getDebug())
            {
              debugStream_ << "Subscribed to " << imuTopic << "\n";
            }
          }
        } while (moreParams);

        // Load up the process noise covariance (from the launch file/parameter server)
        Eigen::MatrixXd processNoiseCovariance(STATE_SIZE, STATE_SIZE);
        processNoiseCovariance.setZero();
        XmlRpc::XmlRpcValue processNoiseCovarConfig;

        if (nhLocal_.hasParam("process_noise_covariance"))
        {
          try
          {
            nhLocal_.getParam("process_noise_covariance", processNoiseCovarConfig);

            ROS_ASSERT(processNoiseCovarConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

            int matSize = processNoiseCovariance.rows();

            ////////////////////////////// HANDLE DEPRECATED PROCESS NOISE COVARIANCE DIMENSIONS //////////////////////////////
            if (processNoiseCovarConfig.size() != matSize * matSize)
            {
              if(::fabs(::sqrt(processNoiseCovarConfig.size()) - 12) < 0.1)
              {
                ROS_WARN_STREAM("Process_noise_covariance matrix should have " << matSize * matSize << " values.");
                matSize = 12;
              }
              else
              {
                ROS_FATAL_STREAM("Process_noise_covariance matrix must have " << matSize * matSize << " values.");
              }
            }
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            for (int i = 0; i < matSize; i++)
            {
              for (int j = 0; j < matSize; j++)
              {
                processNoiseCovariance(i, j) = processNoiseCovarConfig[matSize * i + j];
              }
            }

            if (filter_.getDebug())
            {
              debugStream_ << "Process noise covariance is:\n" << processNoiseCovariance;
            }
          }
          catch (XmlRpc::XmlRpcException &e)
          {
            ROS_ERROR_STREAM(
              "ERROR reading sensor config: " << e.getMessage() << " for process_noise_covariance (type: " << processNoiseCovarConfig.getType() << ")");
          }

          filter_.setProcessNoiseCovariance(processNoiseCovariance);
        }
      }

      //! @brief Callback method for receiving all IMU messages
      //! @param[in] msg - The ROS IMU message to take in.
      //! @param[in] topicName - The name of the IMU data topic (we support many)
      //! @param[in] updateVector - Specifies which variables we want to update from this measurement
      //! @param[in] differential - Whether we integrate the pose portions of this message differentially
      //!
      //! This method really just separates out the absolute orientation and velocity data into two new
      //! messages and sends them to their respective pose and twist callbacks.
      //!
      void imuCallback(const sensor_msgs::Imu::ConstPtr &msg,
                       const std::string &topicName,
                       const std::vector<int> &updateVector,
                       const bool differential)
      {
        if (filter_.getDebug())
        {
          debugStream_ << "------ RosFilter::imuCallback (" << topicName << ") ------\n" <<
                          "IMU message:\n" << *msg;
        }

        // As with the odometry message, we can separate out the pose- and twist-related variables
        // in the IMU message and pass them to the pose and twist callbacks (filters)

        // Extract the pose (orientation) data, pass it to its filter
        geometry_msgs::PoseWithCovarianceStamped *posPtr = new geometry_msgs::PoseWithCovarianceStamped();
        posPtr->header = msg->header;
        posPtr->pose.pose.orientation = msg->orientation;

        // Copy the covariance for roll, pitch, and yaw
        for (size_t i = 0; i < ORIENTATION_SIZE; i++)
        {
          for (size_t j = 0; j < ORIENTATION_SIZE; j++)
          {
            posPtr->pose.covariance[POSE_SIZE * (i + ORIENTATION_SIZE) + (j + ORIENTATION_SIZE)] = msg->orientation_covariance[ORIENTATION_SIZE * i + j];
          }
        }

        geometry_msgs::PoseWithCovarianceStampedConstPtr pptr(posPtr);
        std::string imuPoseTopicName = topicName + "_pose";
        poseMessageFilters_[imuPoseTopicName]->add(pptr);

        // Repeat for velocity
        geometry_msgs::TwistWithCovarianceStamped *twistPtr = new geometry_msgs::TwistWithCovarianceStamped();
        twistPtr->header = msg->header;
        twistPtr->twist.twist.angular = msg->angular_velocity;

        // Copy the covariance
        for (size_t i = 0; i < ORIENTATION_SIZE; i++)
        {
          for (size_t j = 0; j < ORIENTATION_SIZE; j++)
          {
            twistPtr->twist.covariance[TWIST_SIZE * (i + ORIENTATION_SIZE) + (j + ORIENTATION_SIZE)] = msg->angular_velocity_covariance[ORIENTATION_SIZE * i + j];
          }
        }

        geometry_msgs::TwistWithCovarianceStampedConstPtr tptr(twistPtr);
        std::string imuTwistTopicName = topicName + "_twist";
        twistMessageFilters_[imuTwistTopicName]->add(tptr);

        // We still need to handle the acceleration data, but we don't
        // actually have a good container message for it, so just pass
        // the IMU message on through a message filter.
        std::string imuAccelTopicName = topicName + "_acceleration";
        accelerationMessageFilters_[imuAccelTopicName]->add(msg);

        if (filter_.getDebug())
        {
          debugStream_ << "\n----- /RosFilter::imuCallback (" << topicName << ") ------\n";
        }
      }

      //! @brief Callback method for receiving all odometry messages
      //! @param[in] msg - The ROS odometry message to take in.
      //! @param[in] topicName - The name of the odometry topic (we support many)
      //! @param[in] updateVector - Specifies which variables we want to update from this measurement
      //! @param[in] differential - Whether we integrate the pose portions of this message differentially
      //!
      //! This method really just separates out the pose and twist data into two new messages, and passes
      //! them to their respective callbacks
      //!
      void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg,
                            const std::string &topicName,
                            const std::vector<int> &updateVector,
                            const bool differential)
      {
        if (filter_.getDebug())
        {
          debugStream_ << "------ RosFilter::odometryCallback (" << topicName << ") ------\n" <<
                          "Odometry message:\n" << *msg;
        }

        // Grab the pose portion of the message and pass it to the poseCallback
        geometry_msgs::PoseWithCovarianceStamped *posPtr = new geometry_msgs::PoseWithCovarianceStamped();
        posPtr->header = msg->header;
        posPtr->pose = msg->pose; // Entire pose object, also copies covariance

        geometry_msgs::PoseWithCovarianceStampedConstPtr pptr(posPtr);
        std::string odomPoseTopicName = topicName + "_pose";
        poseMessageFilters_[odomPoseTopicName]->add(pptr);

        // Grab the twist portion of the message and pass it to the twistCallback
        geometry_msgs::TwistWithCovarianceStamped *twistPtr = new geometry_msgs::TwistWithCovarianceStamped();
        twistPtr->header = msg->header;
        twistPtr->header.frame_id = msg->child_frame_id;
        twistPtr->twist = msg->twist; // Entire twist object, also copies covariance

        geometry_msgs::TwistWithCovarianceStampedConstPtr tptr(twistPtr);
        std::string odomTwistTopicName = topicName + "_twist";
        twistMessageFilters_[odomTwistTopicName]->add(tptr);

        if (filter_.getDebug())
        {
          debugStream_ << "\n----- /RosFilter::odometryCallback (" << topicName << ") ------\n";
        }
      }

      //! @brief Callback method for receiving all pose messages
      //! @param[in] msg - The ROS stamped pose with covariance message to take in
      //! @param[in] topicName - The name of the pose topic (we support many)
      //! @param[in] targetFrame - The tf frame name into which we will transform this measurement
      //! @param[in] updateVector - Specifies which variables we want to update from this measurement
      //! @param[in] differential - Whether we integrate the pose portions of this message differentially
      //!
      void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                        const std::string &topicName,
                        const std::string &targetFrame,
                        const std::vector<int> &updateVector,
                        const bool differential)
      {
        if (filter_.getDebug())
        {
          debugStream_ << "------ RosFilter::poseCallback (" << topicName << ") ------\n" <<
                          "Pose message:\n" << *msg;
        }

        // Put the initial value in the lastMessagTimes_ for this variable if it's empty
        if (lastMessageTimes_.count(topicName) == 0)
        {
          lastMessageTimes_.insert(std::pair<std::string, ros::Time>(topicName, msg->header.stamp));
        }

        // Make sure this message is newer than the last one
        if (msg->header.stamp >= lastMessageTimes_[topicName])
        {
          if (filter_.getDebug())
          {
            debugStream_ << "Update vector for " << topicName << " is:\n";
            debugStream_ << updateVector;
          }

          Eigen::VectorXd measurement(STATE_SIZE);
          Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);

          measurement.setZero();
          measurementCovariance.setZero();

          // Make sure we're actually updating at least one of these variables
          if (updateVector[StateMemberX] || updateVector[StateMemberY] || updateVector[StateMemberZ] ||
              updateVector[StateMemberRoll] || updateVector[StateMemberPitch] || updateVector[StateMemberYaw])
          {
            std::vector<int> updateVectorCorrected = updateVector;

            // Prepare the pose data for inclusion in the filter
            if (preparePose(msg, topicName + "_pose", targetFrame, differential, updateVectorCorrected, measurement, measurementCovariance))
            {
              // Store the measurement
              filter_.enqueueMeasurement(topicName + "_pose", measurement, measurementCovariance, updateVectorCorrected, msg->header.stamp.toSec());

              if (filter_.getDebug())
              {
                debugStream_ << "Enqueued new measurement for " << topicName + "_pose\n";
              }
            }
            else if(filter_.getDebug())
            {
              debugStream_ << "Did *not* enqueue measurement for " << topicName + "_pose\n";
            }
          }
          else
          {
            if (filter_.getDebug())
            {
              debugStream_ << "Update vector for " << topicName << " is such that none of its state variables will be updated\n";
            }
          }

          lastMessageTimes_[topicName] = msg->header.stamp;

          if (filter_.getDebug())
          {
            debugStream_ << "Last message time for " << topicName << " is now " << lastMessageTimes_[topicName] << "\n";
          }
        }
        else
        {
          if (filter_.getDebug())
          {
            debugStream_ << "Message is too old. Last message time for " << topicName << " is " << lastMessageTimes_[topicName] << ", current message time is "
              << msg->header.stamp << ".\n";
          }
        }

        if (filter_.getDebug())
        {
          debugStream_ << "\n----- /RosFilter::poseCallback (" << topicName << ") ------\n";
        }
      }

      //! @brief Callback method for manually setting/resetting the internal pose estimate
      //! @param[in] msg - The ROS stamped pose with covariance message to take in
      //!
      void setPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
      {
        if (filter_.getDebug())
        {
          debugStream_ << "------ RosFilter::setPoseCallback ------\n";
          debugStream_ << "Pose message:\n";
          debugStream_ << *msg;
        }

        std::string topicName("setPose");

        // Get rid of any initial poses (pretend we've never had a measurement)
        previousMeasurements_.clear();

        // Get rid of any initial states (pretend we've never had a measurement)
        previousStates_.clear();

        // We want the preparePose method to succeed with its transforms, so
        // we need to act like we've had previous measurements for this sensor.
        tf::Transform empty;
        empty.setIdentity();
        previousMeasurements_.insert(std::pair<std::string, tf::Transform>(topicName, empty));
        previousStates_.insert(std::pair<std::string, tf::Transform>(topicName, empty));

        // Set the state vector to the reported pose
        Eigen::VectorXd measurement(STATE_SIZE);
        Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);
        std::vector<int> updateVector(STATE_SIZE, true);

        // We only measure pose variables, so initialize the vector to 0
        measurement.setZero();

        // Set this to the identity and let the message reset it
        measurementCovariance.setIdentity();
        measurementCovariance *= 1e-6;

        // Prepare the pose data (really just using this to transform it into the target frame)
        preparePose(msg, topicName, odomFrameId_, false, updateVector, measurement, measurementCovariance);

        // Force everything to be reset
        filter_.setState(measurement);
        filter_.setEstimateErrorCovariance(measurementCovariance);

        if (filter_.getDebug())
        {
          debugStream_ << "\n------ /RosFilter::setPoseCallback ------\n";
        }
      }

      //! @brief Callback method for receiving all twist messages
      //! @param[in] msg - The ROS stamped twist with covariance message to take in.
      //! @param[in] topicName - The name of the twist topic (we support many)
      //! @param[in] targetFrame - The tf frame name into which we will transform this measurement
      //! @param[in] updateVector - Specifies which variables we want to update from this measurement
      //!
      void twistCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg,
                         const std::string &topicName,
                         const std::string &targetFrame,
                         const std::vector<int> &updateVector)
      {
        if (filter_.getDebug())
        {
          debugStream_ << "------ RosFilter::twistCallback (" << topicName << ") ------\n";
          debugStream_ << "Twist message:\n";
          debugStream_ << *msg;
        }

        if (lastMessageTimes_.count(topicName) == 0)
        {
          lastMessageTimes_.insert(std::pair<std::string, ros::Time>(topicName, msg->header.stamp));
        }

        // Make sure this message is newer than the last one
        if (msg->header.stamp >= lastMessageTimes_[topicName])
        {
          if (filter_.getDebug())
          {
            debugStream_ << "Update vector for " << topicName << " is:\n";
            debugStream_ << updateVector;
          }

          Eigen::VectorXd measurement(STATE_SIZE);
          Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);

          measurement.setZero();
          measurementCovariance.setZero();

          // Make sure we're actually updating at least one of these variables
          if (updateVector[StateMemberVx] || updateVector[StateMemberVy] || updateVector[StateMemberVz] ||
              updateVector[StateMemberVroll] || updateVector[StateMemberVpitch] || updateVector[StateMemberVyaw])
          {
            std::vector<int> updateVectorCorrected = updateVector;

            // Prepare the twist data for inclusion in the filter
            if (prepareTwist(msg, topicName + "_twist", targetFrame, updateVectorCorrected, measurement, measurementCovariance))
            {
              // Store the measurement
              filter_.enqueueMeasurement(topicName + "_twist", measurement, measurementCovariance, updateVectorCorrected, msg->header.stamp.toSec());

              if (filter_.getDebug())
              {
                debugStream_ << "Enqueued new measurement for " << topicName + "_twist\n";
              }
            }
          }
          else
          {
            if (filter_.getDebug())
            {
              debugStream_ << "Update vector for " << topicName << " is such that none of its state variables will be updated\n";
            }
          }

          lastMessageTimes_[topicName] = msg->header.stamp;

          if (filter_.getDebug())
          {
            debugStream_ << "Last message time for " << topicName << " is now " << lastMessageTimes_[topicName] << "\n";
          }
        }
        else
        {
          if (filter_.getDebug())
          {
            debugStream_ << "Message is too old. Last message time for " << topicName <<
                            " is " << lastMessageTimes_[topicName] << ", current message time is " <<
                            msg->header.stamp << ".\n";
          }
        }

        if (filter_.getDebug())
        {
          debugStream_ << "\n----- /RosFilter::twistCallback (" << topicName << ") ------\n";
        }
      }

      //! @brief Callback method for receiving all accelration (twist) messages
      //! @param[in] msg - The ROS stamped twist with covariance message to take in.
      //! @param[in] topicName - The name of the twist topic (we support many)
      //! @param[in] targetFrame - The tf frame name into which we will transform this measurement
      //! @param[in] updateVector - Specifies which variables we want to update from this measurement
      //!
      void accelerationCallback(const sensor_msgs::Imu::ConstPtr &msg,
                                const std::string &topicName,
                                const std::string &targetFrame,
                                const std::vector<int> &updateVector)
      {
        if (filter_.getDebug())
        {
          debugStream_ << "------ RosFilter::acclerationCallback (" << topicName << ") ------\n";
          debugStream_ << "Twist message:\n";
          debugStream_ << *msg;
        }

        if (lastMessageTimes_.count(topicName) == 0)
        {
          lastMessageTimes_.insert(std::pair<std::string, ros::Time>(topicName, msg->header.stamp));
        }

        // Make sure this message is newer than the last one
        if (msg->header.stamp >= lastMessageTimes_[topicName])
        {
          if (filter_.getDebug())
          {
            debugStream_ << "Update vector for " << topicName << " is:\n";
            debugStream_ << updateVector;
          }

          Eigen::VectorXd measurement(STATE_SIZE);
          Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);

          measurement.setZero();
          measurementCovariance.setZero();

          // Make sure we're actually updating at least one of these variables
          if (updateVector[StateMemberAx] || updateVector[StateMemberAy] || updateVector[StateMemberAz])
          {
            std::vector<int> updateVectorCorrected = updateVector;

            // Prepare the twist data for inclusion in the filter
            if (prepareAcceleration(msg, topicName + "_acceleration", targetFrame, updateVectorCorrected, measurement, measurementCovariance))
            {
              // Store the measurement
              filter_.enqueueMeasurement(topicName + "_acceleration", measurement, measurementCovariance, updateVectorCorrected, msg->header.stamp.toSec());

              if (filter_.getDebug())
              {
                debugStream_ << "Enqueued new measurement for " << topicName + "_acceleration\n";
              }
            }
          }
          else
          {
            if (filter_.getDebug())
            {
              debugStream_ << "Update vector for " << topicName << " is such that none of its state variables will be updated\n";
            }
          }

          lastMessageTimes_[topicName] = msg->header.stamp;

          if (filter_.getDebug())
          {
            debugStream_ << "Last message time for " << topicName << " is now " << lastMessageTimes_[topicName] << "\n";
          }
        }
        else
        {
          if (filter_.getDebug())
          {
            debugStream_ << "Message is too old. Last message time for " << topicName <<
                            " is " << lastMessageTimes_[topicName] << ", current message time is " <<
                            msg->header.stamp << ".\n";
          }
        }

        if (filter_.getDebug())
        {
          debugStream_ << "\n----- /RosFilter::accelerationCallback (" << topicName << ") ------\n";
        }
      }

      //! @brief Main run method
      //!
      void run()
      {
        ros::Time::init();

        loadParams();

        // We may need to broadcast a different transform than
        // the one we've already calculated.
        tf::StampedTransform mapOdomTrans;
        tf::StampedTransform odomBaseLinkTrans;
        geometry_msgs::TransformStamped mapOdomTransMsg;

        // Clear out the transforms
        tf::transformTFToMsg(tf::Transform::getIdentity(), worldBaseLinkTransMsg_.transform);
        tf::transformTFToMsg(tf::Transform::getIdentity(), mapOdomTransMsg.transform);

        // Publisher
        ros::Publisher positionPub = nh_.advertise<nav_msgs::Odometry>("odometry/filtered", 20);
        tf::TransformBroadcaster worldTransformBroadcaster;

        ros::Rate loop_rate(frequency_);

        std::map<std::string, Eigen::VectorXd> postUpdateStates;

        while (ros::ok())
        {
          // Get latest state and publish it
          nav_msgs::Odometry filteredPosition;

          if (getFilteredOdometryMessage(filteredPosition))
          {
            worldBaseLinkTransMsg_.header.stamp = filteredPosition.header.stamp;
            worldBaseLinkTransMsg_.header.frame_id = filteredPosition.header.frame_id;
            worldBaseLinkTransMsg_.child_frame_id = filteredPosition.child_frame_id;

            worldBaseLinkTransMsg_.transform.translation.x = filteredPosition.pose.pose.position.x;
            worldBaseLinkTransMsg_.transform.translation.y = filteredPosition.pose.pose.position.y;
            worldBaseLinkTransMsg_.transform.translation.z = filteredPosition.pose.pose.position.z;
            worldBaseLinkTransMsg_.transform.rotation = filteredPosition.pose.pose.orientation;

            if(filteredPosition.header.frame_id == odomFrameId_)
            {
              worldTransformBroadcaster.sendTransform(worldBaseLinkTransMsg_);
            }
            else if(filteredPosition.header.frame_id == mapFrameId_)
            {
              try
              {
                tf::StampedTransform worldBaseLinkTrans;
                tf::transformStampedMsgToTF(worldBaseLinkTransMsg_, worldBaseLinkTrans);

                tfListener_.lookupTransform(baseLinkFrameId_, odomFrameId_, ros::Time(0), odomBaseLinkTrans);

                // We have a transform from mapFrameId_->baseLinkFrameId_, but it would actually
                // transform data from baseLinkFrameId_->mapFrameId_. We then used lookupTransform, 
                // whose first two arguments are target frame and source frame, to get a transform
                // from baseLinkFrameId_->odomFrameId_ (see http://wiki.ros.org/tf/Overview/Using%20Published%20Transforms). 
                // However, this transform would actually transform data from 
                // odomFrameId_->baseLinkFrameId_. Now imagine that we have a position in the 
                // mapFrameId_ frame. First, we multiply it by the inverse of the 
                // mapFrameId_->baseLinkFrameId, which will transform that data from mapFrameId_ to 
                // baseLinkFrameId_. Now we want to go from baseLinkFrameId_->odomFrameId_, but the
                // transform we have takes data from odomFrameId_->baseLinkFrameId_, so we need its
                // inverse as well. We have now transformed our data from mapFrameId_ to odomFrameId_.
                // Long story short: lookupTransform returns the inverse of what you send when you 
                // broadcast transforms, so be careful.
                //
                mapOdomTrans.setData(odomBaseLinkTrans.inverse() * worldBaseLinkTrans.inverse());
                tf::transformStampedTFToMsg(mapOdomTrans, mapOdomTransMsg);
                mapOdomTransMsg.header.stamp = filteredPosition.header.stamp;
                mapOdomTransMsg.header.frame_id = mapFrameId_;
                mapOdomTransMsg.child_frame_id = odomFrameId_;

                worldTransformBroadcaster.sendTransform(mapOdomTransMsg);
              }
              catch(...)
              {
                ROS_ERROR_STREAM("Could not obtain transform from " << odomFrameId_ << "->" << baseLinkFrameId_);
              }
            }
            else
            {
              ROS_ERROR_STREAM("Odometry message frame_id was " << filteredPosition.header.frame_id <<
                               ", expected " << mapFrameId_ << " or " << odomFrameId_);
            }

            // Fire off the position and the transform
            positionPub.publish(filteredPosition);
          }

          // The spin will enqueue all the available callbacks
          ros::spinOnce();

          // Now we'll integrate any measurements we've received
          filter_.integrateMeasurements(ros::Time::now().toSec(),
                                        postUpdateStates);

          // Now copy the post-update states into our local
          // copies
          std::map<std::string, Eigen::VectorXd>::iterator mapIt;

          for(mapIt = postUpdateStates.begin(); mapIt != postUpdateStates.end(); ++mapIt)
          {
            tf::Transform trans;
            trans.setOrigin(tf::Vector3(mapIt->second(StateMemberX),
                                        mapIt->second(StateMemberY),
                                        mapIt->second(StateMemberZ)));
            tf::Quaternion quat;
            quat.setRPY(mapIt->second(StateMemberRoll),
                        mapIt->second(StateMemberPitch),
                        mapIt->second(StateMemberYaw));
            trans.setRotation(quat);
            previousStates_[mapIt->first] = trans;
          }

          loop_rate.sleep();
        }
      }

    protected:

      Filter filter_;

      //! @brief Loads fusion settings from the config file
      //! @param[in] topicName - The name of the topic for which to load settings
      //! @return The boolean vector of update settings for each variable for this topic
      //!
      std::vector<int> loadUpdateConfig(const std::string &topicName)
      {
        XmlRpc::XmlRpcValue topicConfig;
        std::vector<int> updateVector(STATE_SIZE, 0);
        std::string topicConfigName = topicName + "_config";

        try
        {
          nhLocal_.getParam(topicConfigName, topicConfig);

          ROS_ASSERT(topicConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

          if(topicConfig.size() != STATE_SIZE)
          {
            ROS_WARN_STREAM("Configuration vector for " << topicConfigName << " should have 15 entries.");
          }

          for (int i = 0; i < topicConfig.size(); i++)
          {
            // The double cast looks strange, but we'll get exceptions if we
            // remove the cast to bool. vector<bool> is discouraged, so updateVector
            // uses integers
            updateVector[i] = static_cast<int>(static_cast<bool>(topicConfig[i]));
          }
        }
        catch (XmlRpc::XmlRpcException &e)
        {
          ROS_ERROR_STREAM("ERROR reading sensor update config: " << e.getMessage() << " for topic " <<
                           topicName << " (type: " << topicConfig.getType() << ", expected: " << XmlRpc::XmlRpcValue::TypeArray << ")");
        }

        return updateVector;
      }

      //! @brief Method for safely obtaining transforms.
      //! @param[in] targetFrame - The target frame of the desired transform
      //! @param[in] sourceFrame - The source frame of the desired transform
      //! @param[in] time - The time at which we want the transform
      //! @param[out] targetFrameTrans - The resulting stamped transform object
      //! @return Sets the value of @p targetFrameTrans and returns true if sucessful,
      //! false otherwise.
      //!
      //! This method attempts to obtain a transform from the @p sourceFrame to the @p
      //! targetFrame at the specific @p time. If no transform is available at that time,
      //! it attempts to simply obtain the latest transform. If that still fails, then the
      //! method checks to see if the transform is going from a given frame_id to itself.
      //! If any of these checks succeed, the method sets the value of @p targetFrameTrans
      //! and returns true, otherwise it returns false.
      //!
      bool lookupTransformSafe(const std::string &targetFrame, const std::string &sourceFrame,
                               const ros::Time &time, tf::StampedTransform &targetFrameTrans)
      {
        bool retVal = true;

        // First try to transform the data at the requested time
        try
        {
          tfListener_.lookupTransform(targetFrame, sourceFrame, time, targetFrameTrans);
        }
        catch (tf::TransformException &ex)
        {
          if(filter_.getDebug())
          {
            debugStream_ << "WARNING: Could not obtain transform from " << sourceFrame <<
                            " to " << targetFrame << ". Error was " << ex.what();
          }

          // The issue might be that the transforms that are available are not close
          // enough temporally to be used. In that case, just use the latest available
          // transform and warn the user.
          try
          {
            tfListener_.lookupTransform(targetFrame, sourceFrame, ros::Time(0), targetFrameTrans);

            ROS_WARN_STREAM("Transform from " << sourceFrame << " to " << targetFrame <<
                            " was unavailable for the time requested. Using latest instead.\n");
          }
          catch(tf::TransformException &ex)
          {
            ROS_WARN_STREAM("Could not obtain transform from " << sourceFrame <<
                             " to " << targetFrame << ". Error was " << ex.what() << "\n");

            retVal = false;
          }
        }

        // Transforming from a frame id to itself can fail when the tf tree isn't
        // being broadcast (e.g., for some bag files). This is the only failure that
        // would throw an exception, so check for this situation before giving up.
        if(!retVal)
        {
          std::string msgFrame = (tfPrefix_.empty() ? targetFrame : "/" + tfPrefix_ + "/" + targetFrame);

          if(targetFrame == sourceFrame)
          {
            targetFrameTrans.setIdentity();
            retVal = true;
          }
        }

        return retVal;
      }

      //! @brief Prepares a pose message for integration into the filter
      //! @param[in] msg - The pose message to prepare
      //! @param[in] topicName - The name of the topic over which this message was received
      //! @param[in] targetFrame - The target tf frame
      //! @param differential - Whether we're carrying out differential integration
      //! @param[in,out] updateVector - The update vector for the data source
      //! @param[out] measurement - The pose data converted to a measurement
      //! @param[out] measurementCovariance - The covariance of the converted measurement
      //! @return true indicates that the measurement was successfully prepared, false otherwise
      //!
      bool preparePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                       const std::string &topicName,
                       const std::string &targetFrame,
                       const bool differential,
                       std::vector<int> &updateVector,
                       Eigen::VectorXd &measurement,
                       Eigen::MatrixXd &measurementCovariance)
      {
        if (filter_.getDebug())
        {
          debugStream_ << "------ RosFilter::preparePose (" << topicName << ") ------\n";
        }

        // 1. Get the measurement into a tf-friendly transform (pose) object
        tf::Stamped<tf::Pose> poseTmp;

        // We'll need this later for storing this measurement for differential integration
        tf::Transform curMeasurement;

        // Determine if the message had a frame id associated with it. If not, assume the targetFrame.
        poseTmp.frame_id_ = (msg->header.frame_id == "" ? targetFrame : msg->header.frame_id);
        poseTmp.stamp_ = msg->header.stamp;

        // The selective updating of variables can be a bit tricky, especially
        // when we have differential updating to worry about. Rather than go into
        // the details of why here, suffice it to say that users must, if using
        // selective updating, be *very* careful about which variables they exclude
        // from the measurement
        poseTmp.setOrigin(tf::Vector3(msg->pose.pose.position.x,
                                      msg->pose.pose.position.y,
                                      msg->pose.pose.position.z));

        tf::Quaternion orientation;

        // Handle bad (empty) quaternions
        if(msg->pose.pose.orientation.x == 0 && msg->pose.pose.orientation.y == 0 &&
           msg->pose.pose.orientation.z == 0 && msg->pose.pose.orientation.w == 0)
        {
          orientation.setValue(0.0, 0.0, 0.0, 1.0);

          if(updateVector[StateMemberRoll] || updateVector[StateMemberPitch] || updateVector[StateMemberYaw])
          {
            ROS_WARN_STREAM("The " << topicName << " message contains an invalid orientation quaternion, " <<
                            "but its configuration is such that orientation data is being used.");
          }
        }
        else
        {
          tf::quaternionMsgToTF(msg->pose.pose.orientation, orientation);
        }

        poseTmp.setRotation(orientation);

        // 2. robot_localization lets users configure which variables from the sensor should be
        //    fused with the filter. This is specified for whatever frame the input message happens
        //    to be in. However, the data  may go through transforms before being fused with the state
        //    estimate. In that case, we need to know which of the transformed variables came from the
        //    pre-transformed "approved" variables (i.e., the ones that had "true" in their xxx_config
        //    parameter). To do this, we create a pose from the original upate vector, which contains
        //    only zeros and ones. This pose goes through the same transforms as the measurement. The
        //    non-zero values that result will be used to modify the updateVector.
        tf::Pose maskPosePos;
        tf::Pose maskPoseNeg;
        maskPosePos.setOrigin(tf::Vector3(updateVector[StateMemberX],
                                       updateVector[StateMemberY],
                                       updateVector[StateMemberZ]));

        maskPoseNeg.setOrigin(-maskPosePos.getOrigin());

        tf::Quaternion maskOrientation;
        maskOrientation.setRPY(updateVector[StateMemberRoll],
                               updateVector[StateMemberPitch],
                               updateVector[StateMemberYaw]);

        maskPosePos.setRotation(maskOrientation);

        maskOrientation.setRPY(-updateVector[StateMemberRoll],
                               -updateVector[StateMemberPitch],
                               -updateVector[StateMemberYaw]);

        maskPoseNeg.setRotation(maskOrientation);

        // 3. We'll need to rotate the covariance as well. Create a container and
        // copy over the covariance data
        Eigen::MatrixXd covarianceRotated(POSE_SIZE, POSE_SIZE);
        covarianceRotated.setZero();

        for (size_t i = 0; i < POSE_SIZE; i++)
        {
          for (size_t j = 0; j < POSE_SIZE; j++)
          {
            covarianceRotated(i, j) = msg->pose.covariance[POSE_SIZE * i + j];
          }
        }

        if(filter_.getDebug())
        {
          debugStream_ << "Original measurement as tf object:\n" << poseTmp <<
                          "\nOriginal update vector:\n" << updateVector <<
                          "\nOriginal covariance matrix:\n" << covarianceRotated << "\n";
        }

        // 4. We have a series of transforms to carry out:
        //   a. Transform into the target frame
        //   b. Remove the previous measurement's value (only if carrying out differential integration)
        //   c. Apply the current state as a transform to get a measurement that is consistent with the
        //      state (again, only if carrying out differential integration)

        // First, we want to make sure we can carry out all the transforms we need.

        // If this is the first measurement from a sensor, create a value in previousStates_
        // (we only use this in differential integration)
        if(differential && previousStates_.count(topicName) == 0)
        {
          tf::Pose prevPose;
          tf::transformMsgToTF(worldBaseLinkTransMsg_.transform, prevPose);
          previousStates_.insert(std::pair<std::string, tf::Transform>(topicName, prevPose));
        }

        // 4a. Get the target frame transformation
        tf::StampedTransform targetFrameTrans;

        bool canTransform = lookupTransformSafe(targetFrame, poseTmp.frame_id_, poseTmp.stamp_, targetFrameTrans);

        if(canTransform)
        {
          // Apply the target frame transformation to the pose object 
          poseTmp.mult(targetFrameTrans, poseTmp);

          // Now apply it to the masks, positive first
          maskPosePos.setOrigin(targetFrameTrans.getBasis() * maskPosePos.getOrigin());
          maskPoseNeg.setOrigin(targetFrameTrans.getBasis() * maskPoseNeg.getOrigin());

          // Now copy the mask values back into the update vector
          updateVector[StateMemberX] = static_cast<int>(::fabs(maskPosePos.getOrigin().getX()) >= 1e-6 || ::fabs(maskPoseNeg.getOrigin().getX()) >= 1e-6);
          updateVector[StateMemberY] = static_cast<int>(::fabs(maskPosePos.getOrigin().getY()) >= 1e-6 || ::fabs(maskPoseNeg.getOrigin().getY()) >= 1e-6);
          updateVector[StateMemberZ] = static_cast<int>(::fabs(maskPosePos.getOrigin().getZ()) >= 1e-6 || ::fabs(maskPoseNeg.getOrigin().getZ()) >= 1e-6);

          double rollPos, pitchPos, yawPos;
          double rollNeg, pitchNeg, yawNeg;
          quatToRPY(maskPosePos.getRotation(), rollPos, pitchPos, yawPos);
          quatToRPY(maskPoseNeg.getRotation(), rollNeg, pitchNeg, yawNeg);
          updateVector[StateMemberRoll] = static_cast<int>(::fabs(rollPos) >= 1e-6 || ::fabs(rollNeg) >= 1e-6);
          updateVector[StateMemberPitch] = static_cast<int>(::fabs(pitchPos) >= 1e-6 || ::fabs(pitchNeg) >= 1e-6);
          updateVector[StateMemberYaw] = static_cast<int>(::fabs(yawPos) >= 1e-6 || ::fabs(yawNeg) >= 1e-6);

          if(filter_.getDebug())
          {
            debugStream_ << poseTmp.frame_id_ << "->" << targetFrame << " transform:\n" << targetFrameTrans <<
                            "\nAfter applying transform to " << targetFrame << ", update vector is:\n" << updateVector <<
                            "\nAfter applying transform to " << targetFrame << ", measurement is:\n" << poseTmp << "\n";
          }

          poseTmp.frame_id_ = targetFrame;

          // Store the measurement as a transform for the next value (differential integration)
          curMeasurement = poseTmp;

          // If we're in differential mode, we want to make sure
          // we have a previous measurement to work with.
          canTransform = (!differential || previousMeasurements_.count(topicName) > 0);
          if(differential)
          {
            if(canTransform)
            {
              // 4b. If we are carrying out differential integration and
              // we have a previous measurement for this sensor,then we
              // need to apply the inverse of that measurement to this new
              // measurement.

              // Even if we're not using all of the variables from this sensor,
              // we need to use the whole measurement to determine the delta
              // to the new measurement
              tf::Pose prevMeasurement = previousMeasurements_[topicName];

              // Determine the pose delta by removing the previous measurement.
              poseTmp.setData(prevMeasurement.inverseTimes(poseTmp));

              /*
               * TAM: two options here: we can just add the difference between
               * this measurement and the previous one to our current state so
               * as to generate a new measurement, or we can create a velocity
               * and feed it to prepareTwist. Not sure which is better, so sticking
              double dt = msg->header.stamp.toSec() - lastMeasurementTime.toSec();
              double xVel = poseTmp.getOrigin().getX() / dt;
              double yVel = poseTmp.getOrigin().getY() / dt;
              double zVel = poseTmp.getOrigin().getZ() / dt;

              double rollVel = 0;
              double pitchVel = 0;
              double yawVel = 0;

              quatToRPY(poseTmp.getRotation(), rollVel, pitchVel, yawVel);
              rollVel /= dt;
              pitchVel /= dt;
              yawVel /= dt;

              geometry_msgs::TwistWithCovarianceStamped *twistPtr = new geometry_msgs::TwistWithCovarianceStamped();
              twistPtr->header = msg->header;
              twistPtr->header.frame_id = baseLinkFrameId_;
              twistPtr->twist.twist.linear.x = xVel;
              twistPtr->twist.twist.linear.y = yVel;
              twistPtr->twist.twist.linear.z = zVel;
              twistPtr->twist.twist.angular.x = rollVel;
              twistPtr->twist.twist.angular.y = pitchVel;
              twistPtr->twist.twist.angular.z = yawVel;
              std::vector<int> twistUpdateVec(STATE_SIZE, false);
              std::copy(updateVector.begin() + POSITION_OFFSET, updateVector.begin() + POSE_SIZE, twistUpdateVec.begin() + POSITION_V_OFFSET);
              std::copy(twistUpdateVec.begin(), twistUpdateVec.end(), updateVector.begin());
              geometry_msgs::TwistWithCovarianceStampedConstPtr ptr(twistPtr);

              prepareTwist(ptr, topicName + "_twist", twistPtr->header.frame_id, updateVector, measurement, measurementCovariance);
              previousMeasurements_[topicName] = curMeasurement;
              return canTransform;*/

              if (filter_.getDebug())
              {
                debugStream_ << "Previous measurement:\n" << previousMeasurements_[topicName] <<
                                "\nAfter removing previous measurement, measurement is:\n" << poseTmp << "\n";
              }

              // 4c. Now we have the data in the right frame, but we may have more
              // than one data source for absolute pose information in this frame. This
              // is why we allow for differential integration. Take the variables
              // that are integrated differentially, and transform them by essentially
              // adding them to our current state. This produces a new measurement that
              // is consistent with our state, even if the original measurement wasn't.
              tf::Pose prevPose = previousStates_[topicName];

              // "Add" this measurement to the previous pose
              poseTmp.setData(prevPose * poseTmp);

              if (filter_.getDebug())
              {
                debugStream_ << "Transforming to align with state. State is:\n" << prevPose <<
                                "\nMeasurement is now:\n" << poseTmp << "\n";
              }
            }
            else if(filter_.getDebug())
            {
              debugStream_ << topicName << " has no previous measurements and is being "
                              "updated differentially. Could not transform this measurement.\n";
            }
          }

          // Store the measurement so we can remove it
          previousMeasurements_[topicName] = curMeasurement;

          if(canTransform)
          {
            // 5. Now rotate the covariance: create an augmented
            // matrix that contains a 3D rotation matrix in the
            // upper-left and lower-right quadrants, with zeros
            // elsewhere
            tf::Matrix3x3 rot(targetFrameTrans.getRotation());
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
            covarianceRotated = rot6d * covarianceRotated.eval() * rot6d.transpose();

            if (filter_.getDebug())
            {
              debugStream_ << "Transformed covariance is \n" << covarianceRotated << "\n";
            }

            // 6. Finally, copy everything into our measurement and covariance objects
            measurement(StateMemberX) = poseTmp.getOrigin().x();
            measurement(StateMemberY) = poseTmp.getOrigin().y();
            measurement(StateMemberZ) = poseTmp.getOrigin().z();

            // The filter needs roll, pitch, and yaw values instead of quaternions
            double roll, pitch, yaw;
            quatToRPY(poseTmp.getRotation(), roll, pitch, yaw);
            measurement(StateMemberRoll) = roll;
            measurement(StateMemberPitch) = pitch;
            measurement(StateMemberYaw) = yaw;

            measurementCovariance.block(0, 0, POSE_SIZE, POSE_SIZE) = covarianceRotated.block(0, 0, POSE_SIZE, POSE_SIZE);
          }
        }
        else if(filter_.getDebug())
        {
          debugStream_ << "Could not transform measurement into " << targetFrame << ". Ignoring...";
        }

        if(filter_.getDebug())
        {
          debugStream_ << "\n----- /RosFilter::preparePose (" << topicName << ") ------\n";
        }

        return canTransform;
      }

      //! @brief Prepares a twist message for integration into the filter
      //! @param[in] msg - The twist message to prepare
      //! @param[in] topicName - The name of the topic over which this message was received
      //! @param[in] targetFrame - The target tf frame
      //! @param[in,out] updateVector - The update vector for the data source
      //! @param[out] measurement - The twist data converted to a measurement
      //! @param[out] measurementCovariance - The covariance of the converted measurement
      //! @return true indicates that the measurement was successfully prepared, false otherwise
      //!
      bool prepareTwist(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg,
                        const std::string &topicName,
                        const std::string &targetFrame,
                        std::vector<int> &updateVector,
                        Eigen::VectorXd &measurement,
                        Eigen::MatrixXd &measurementCovariance)
      {
        if (filter_.getDebug())
        {
          debugStream_ << "------ RosFilter::prepareTwist (" << topicName << ") ------\n";
        }

        // 1. Get the measurement into two separate vector objects.
        tf::Vector3 twistLin(msg->twist.twist.linear.x,
                             msg->twist.twist.linear.y,
                             msg->twist.twist.linear.z);
        tf::Vector3 twistRot(msg->twist.twist.angular.x,
                             msg->twist.twist.angular.y,
                             msg->twist.twist.angular.z);

        // Set relevant header info
        std::string msgFrame = (msg->header.frame_id == "" ? baseLinkFrameId_ : msg->header.frame_id);

        // 2. robot_localization lets users configure which variables from the sensor should be
        //    fused with the filter. This is specified at the sensor level. However, the data
        //    may go through transforms before being fused with the state estimate. In that case,
        //    we need to know which of the transformed variables came from the pre-transformed
        //    "approved" variables (i.e., the ones that had "true" in their xxx_config parameter).
        //    To do this, we create a pose from the original upate vector, which contains only
        //    zeros and ones. This pose goes through the same transforms as the measurement. The
        //    non-zero values that result will be used to modify the updateVector.
        tf::Vector3 maskTwistLinPos(updateVector[StateMemberVx],
                                    updateVector[StateMemberVy],
                                    updateVector[StateMemberVz]);
        tf::Vector3 maskTwistLinNeg(-updateVector[StateMemberVx],
                                    -updateVector[StateMemberVy],
                                    -updateVector[StateMemberVz]);
        tf::Vector3 maskTwistRotPos(updateVector[StateMemberVroll],
                                    updateVector[StateMemberVpitch],
                                    updateVector[StateMemberVyaw]);
        tf::Vector3 maskTwistRotNeg(-updateVector[StateMemberVroll],
                                    -updateVector[StateMemberVpitch],
                                    -updateVector[StateMemberVyaw]);

        // 3. We'll need to rotate the covariance as well
        Eigen::MatrixXd covarianceRotated(TWIST_SIZE, TWIST_SIZE);
        covarianceRotated.setZero();

        // Copy the measurement's covariance matrix so that we can rotate it later
        for (size_t i = 0; i < TWIST_SIZE; i++)
        {
          for (size_t j = 0; j < TWIST_SIZE; j++)
          {
            covarianceRotated(i, j) = msg->twist.covariance[TWIST_SIZE * i + j];
          }
        }

        if(filter_.getDebug())
        {
          debugStream_ << "Original measurement as tf object:\nLinear: " << twistLin <<
                          "\nRotational: " << twistRot <<
                          "\nOriginal update vector:\n" << updateVector <<
                          "\nOriginal covariance matrix:\n" << covarianceRotated << "\n";
        }

        // 4. We need to transform this into the target frame (probably base_link)
        // It's unlikely that we'll get a velocity measurement in another frame, but
        // we have to handle the situation.
        bool canTransform = true;
        tf::StampedTransform targetFrameTrans;

        lookupTransformSafe(targetFrame, msgFrame, msg->header.stamp, targetFrameTrans);

        if(canTransform)
        {
          // Transform to correct frame. Note that we can get linear velocity
          // as a result of the sensor offset and rotational velocity
          twistRot = targetFrameTrans.getBasis() * twistRot;
          twistLin = targetFrameTrans.getBasis() * twistLin - targetFrameTrans.getOrigin().cross(twistRot);
          maskTwistRotPos = targetFrameTrans.getBasis() * maskTwistRotPos;
          maskTwistRotNeg = targetFrameTrans.getBasis() * maskTwistRotNeg;
          maskTwistLinPos = targetFrameTrans.getBasis() * maskTwistLinPos;
          maskTwistLinNeg = targetFrameTrans.getBasis() * maskTwistLinNeg;

          // Now copy the mask values back into the update vector
          updateVector[StateMemberVx] = static_cast<int>(::fabs(maskTwistLinPos.getX()) >= 1e-6 || ::fabs(maskTwistLinNeg.getX()) >= 1e-6);
          updateVector[StateMemberVy] = static_cast<int>(::fabs(maskTwistLinPos.getY()) >= 1e-6 || ::fabs(maskTwistLinNeg.getY()) >= 1e-6);
          updateVector[StateMemberVz] = static_cast<int>(::fabs(maskTwistLinPos.getZ()) >= 1e-6 || ::fabs(maskTwistLinNeg.getZ()) >= 1e-6);
          updateVector[StateMemberVroll] = static_cast<int>(::fabs(maskTwistRotPos.getX()) >= 1e-6 || ::fabs(maskTwistRotNeg.getX()) >= 1e-6);
          updateVector[StateMemberVpitch] = static_cast<int>(::fabs(maskTwistRotPos.getY()) >= 1e-6 || ::fabs(maskTwistRotNeg.getY()) >= 1e-6);
          updateVector[StateMemberVyaw] = static_cast<int>(::fabs(maskTwistRotPos.getZ()) >= 1e-6 || ::fabs(maskTwistRotNeg.getZ()) >= 1e-6);

          if(filter_.getDebug())
          {
            debugStream_ << msg->header.frame_id << "->" << targetFrame << " transform:\n" << targetFrameTrans <<
                            "\nAfter applying transform to " << targetFrame << ", update vector is:\n" << updateVector <<
                            "\nAfter applying transform to " << targetFrame << ", measurement is:\n" <<
                            "Linear: " << twistLin << "Rotational: " << twistRot << "\n";
          }

          // 5. Now rotate the covariance: create an augmented
          // matrix that contains a 3D rotation matrix in the
          // upper-left and lower-right quadrants, and zeros
          // elsewhere
          tf::Matrix3x3 rot(targetFrameTrans.getRotation());
          Eigen::MatrixXd rot6d(TWIST_SIZE, TWIST_SIZE);
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

          // Carry out the rotation
          covarianceRotated = rot6d * covarianceRotated.eval() * rot6d.transpose();

          if (filter_.getDebug())
          {
            debugStream_ << "Transformed covariance is \n" << covarianceRotated << "\n";
          }

          // 6. Store our corrected measurement and covariance
          measurement(StateMemberVx) = twistLin.getX();
          measurement(StateMemberVy) = twistLin.getY();
          measurement(StateMemberVz) = twistLin.getZ();
          measurement(StateMemberVroll) = twistRot.getX();
          measurement(StateMemberVpitch) = twistRot.getY();
          measurement(StateMemberVyaw) = twistRot.getZ();

          // Copy the covariances
          measurementCovariance.block(POSITION_V_OFFSET, POSITION_V_OFFSET, TWIST_SIZE, TWIST_SIZE) = covarianceRotated.block(0, 0, TWIST_SIZE, TWIST_SIZE);
        }
        else if(filter_.getDebug())
        {
          debugStream_ << "Could not transform measurement into " << targetFrame << ". Ignoring...";
        }

        if (filter_.getDebug())
        {
          debugStream_ << "\n----- /RosFilter::prepareTwist (" << topicName << ") ------\n";
        }

        return canTransform;
      }

      //! @brief Prepares an IMU message's linear acceleration for integration into the filter
      //! @param[in] msg - The IMU message to prepare
      //! @param[in] topicName - The name of the topic over which this message was received
      //! @param[in] targetFrame - The target tf frame
      //! @param[in] updateVector - The update vector for the data source
      //! @param[in] measurement - The twist data converted to a measurement
      //! @param[in] measurementCovariance - The covariance of the converted measurement
      //!
      bool prepareAcceleration(const sensor_msgs::Imu::ConstPtr &msg,
                               const std::string &topicName,
                               const std::string &targetFrame,
                               std::vector<int> &updateVector,
                               Eigen::VectorXd &measurement,
                               Eigen::MatrixXd &measurementCovariance)
      {
        if (filter_.getDebug())
        {
          debugStream_ << "------ RosFilter::prepareAcceleration (" << topicName << ") ------\n";
        }

        // 1. Get the measurement into a vector
        tf::Vector3 accTmp(msg->linear_acceleration.x,
                           msg->linear_acceleration.y,
                           msg->linear_acceleration.z);

        // Set relevant header info
        std::string msgFrame = (msg->header.frame_id == "" ? baseLinkFrameId_ : msg->header.frame_id);

        // 2. robot_localization lets users configure which variables from the sensor should be
        //    fused with the filter. This is specified at the sensor level. However, the data
        //    may go through transforms before being fused with the state estimate. In that case,
        //    we need to know which of the transformed variables came from the pre-transformed
        //    "approved" variables (i.e., the ones that had "true" in their xxx_config parameter).
        //    To do this, we create a pose from the original upate vector, which contains only
        //    zeros and ones. This pose goes through the same transforms as the measurement. The
        //    non-zero values that result will be used to modify the updateVector.
        tf::Vector3 maskAccLinPos(updateVector[StateMemberAx],
                                  updateVector[StateMemberAy],
                                  updateVector[StateMemberAz]);
        tf::Vector3 maskAccLinNeg(-updateVector[StateMemberAx],
                                  -updateVector[StateMemberAy],
                                  -updateVector[StateMemberAz]);

        // 3. We'll need to rotate the covariance as well
        Eigen::MatrixXd covarianceRotated(ACCELERATION_SIZE, ACCELERATION_SIZE);
        covarianceRotated.setZero();

        // Copy the measurement's covariance matrix so that we can rotate it later
        for (size_t i = 0; i < ACCELERATION_SIZE; i++)
        {
          for (size_t j = 0; j < ACCELERATION_SIZE; j++)
          {
            covarianceRotated(i, j) = msg->linear_acceleration_covariance[ACCELERATION_SIZE * i + j];
          }
        }

        if(filter_.getDebug())
        {
          debugStream_ << "Original measurement as tf object: " << accTmp <<
                          "\nOriginal update vector:\n" << updateVector <<
                          "\nOriginal covariance matrix:\n" << covarianceRotated << "\n";
        }

        // 4. We need to transform this into the target frame (probably base_link)
        // It's unlikely that we'll get a velocity measurement in another frame, but
        // we have to handle the situation.
        bool canTransform = true;
        tf::StampedTransform targetFrameTrans;

        lookupTransformSafe(targetFrame, msgFrame, msg->header.stamp, targetFrameTrans);

        if(canTransform)
        {
          // We don't know if the user has already handled the removal
          // of normal forces, so we use a parameter
          if(removeAccNormalForce_)
          {
            tf::Vector3 normAcc(0, 0, 9.80665);
            tf::Quaternion curAttitude;
            tf::Transform trans;
            tf::quaternionMsgToTF(msg->orientation, curAttitude);
            trans.setRotation(curAttitude);
            tf::Vector3 rotNorm = trans.getBasis().inverse() * normAcc;
            accTmp.setX(accTmp.getX() - rotNorm.getX());
            accTmp.setY(accTmp.getY() - rotNorm.getY());
            accTmp.setZ(accTmp.getZ() - rotNorm.getZ());

            if(filter_.getDebug())
            {
              debugStream_ << "Orientation is " << curAttitude;
              debugStream_ << "Acceleration due to gravity is " << rotNorm;
              debugStream_ << "After removing acceleration due to gravity, acceleration is " << accTmp;
            }
          }

          // Transform to correct frame
          accTmp = targetFrameTrans.getBasis() * accTmp;
          maskAccLinPos = targetFrameTrans.getBasis() * maskAccLinPos;
          maskAccLinNeg = targetFrameTrans.getBasis() * maskAccLinNeg;

          // Now copy the mask values back into the update vector
          updateVector[StateMemberAx] = static_cast<int>(::fabs(maskAccLinPos.getX()) >= 1e-6 || ::fabs(maskAccLinNeg.getX()) >= 1e-6);
          updateVector[StateMemberAy] = static_cast<int>(::fabs(maskAccLinPos.getY()) >= 1e-6 || ::fabs(maskAccLinNeg.getY()) >= 1e-6);
          updateVector[StateMemberAz] = static_cast<int>(::fabs(maskAccLinPos.getZ()) >= 1e-6 || ::fabs(maskAccLinNeg.getZ()) >= 1e-6);

          // 5. Now rotate the covariance: create an augmented
          // matrix that contains a 3D rotation matrix in the
          // upper-left and lower-right quadrants, and zeros
          // elsewhere
          tf::Matrix3x3 rot(targetFrameTrans.getRotation());
          Eigen::MatrixXd rot3d(ACCELERATION_SIZE, ACCELERATION_SIZE);
          rot3d.setIdentity();

          for(size_t rInd = 0; rInd < ACCELERATION_SIZE; ++rInd)
          {
            rot3d(rInd, 0) = rot.getRow(rInd).getX();
            rot3d(rInd, 1) = rot.getRow(rInd).getY();
            rot3d(rInd, 2) = rot.getRow(rInd).getZ();
          }

          // Carry out the rotation
          covarianceRotated = rot3d * covarianceRotated.eval() * rot3d.transpose();

          if (filter_.getDebug())
          {
            debugStream_ << "Transformed covariance is \n" << covarianceRotated << "\n";
          }

          // 6. Store our corrected measurement and covariance
          measurement(StateMemberAx) = accTmp.getX();
          measurement(StateMemberAy) = accTmp.getY();
          measurement(StateMemberAz) = accTmp.getZ();

          // Copy the covariances
          measurementCovariance.block(POSITION_A_OFFSET, POSITION_A_OFFSET, ACCELERATION_SIZE, ACCELERATION_SIZE) = covarianceRotated.block(0, 0, ACCELERATION_SIZE, ACCELERATION_SIZE);
        }
        else if(filter_.getDebug())
        {
          debugStream_ << "Could not transform measurement into " << targetFrame << ". Ignoring...";
        }

        if (filter_.getDebug())
        {
          debugStream_ << "\n----- /RosFilter::prepareAcceleration(" << topicName << ") ------\n";
        }

        return canTransform;
      }

      //! @brief Utility method for converting quaternion to RPY
      //! @param[in] quat - The quaternion to convert
      //! @param[out] roll - The converted roll
      //! @param[out] pitch - The converted pitch
      //! @param[out] yaw - The converted yaw
      //!
      inline void quatToRPY(const tf::Quaternion &quat, double &roll, double &pitch, double &yaw)
      {
        tf::Matrix3x3 orTmp(quat);
        orTmp.getRPY(roll, pitch, yaw);
      }

      //! @brief If including acceleration, whether or not we remove
      //! acceleration due to gravity
      //!
      bool removeAccNormalForce_;

      //! @brief The frequency of the run loop
      //!
      double frequency_;

      //! @brief tf prefix
      //!
      std::string tfPrefix_;

      //! @brief tf frame name for the robot's body frame
      //!
      std::string baseLinkFrameId_;

      //! @brief tf frame name for the robot's odometry (world-fixed) frame
      //!
      std::string odomFrameId_;

      //! @brief tf frame name for the robot's map (world-fixed) frame
      //!
      std::string mapFrameId_;

      //! @brief tf frame name that is the parent frame of the transform
      //!        that this node will calculate and broadcast.
      //!
      std::string worldFrameId_;

      //! @brief Store the last time a message from each topic was received
      //!
      //! If we're getting messages rapidly, we may accidentally get
      //! an older message arriving after a newer one. This variable keeps
      //! track of the most recent message time for each subscribed message
      //! topic. We also use it when listening to odometry messages to
      //! determine if we should be using messages from that topic.
      //!
      std::map<std::string, ros::Time> lastMessageTimes_;

      //! @brief Stores the last measurement from a given topic for
      //! differential integration
      //!
      //! To carry out differential integration, we have to (1) transform
      //! that into the target frame (probably the frame specified by
      //! @p odomFrameId_), (2) "subtract"  the previous measurement from
      //! the current measurement, and then (3) transform it again by the previous
      //! state estimate. This holds the measurements used for step (2).
      //!
      std::map<std::string, tf::Transform> previousMeasurements_;

      //! @brief Stores the last state estimate at the time the previous
      //! measurement from this sensor was captured
      //!
      //! To carry out differential integration, we have to (1) transform
      //! that into the target frame (probably the frame specified by
      //! @p odomFrameId_), (2)  "subtract" the previous measurement from
      //! the current measurement, and then (3) transform it again by the previous
      //! state estimate. This holds the measurements used for step (3).
      //!
      std::map<std::string, tf::Transform> previousStates_;

      //! @brief Vector to hold our odometry message filter subscriber objects so they
      //! don't go out of scope.
      //!
      std::vector<ros::Subscriber> odomTopicSubs_;

      //! @brief Vector to hold our pose message filter subscriber objects so they
      //! don't go out of scope.
      //!
      std::vector<poseMFSubPtr> poseTopicSubs_;

      //! @brief Vector to hold our pose message filters so they don't go out of scope.
      //!
      std::map<std::string, poseMFPtr> poseMessageFilters_;

      //! @brief Vector to hold our twist message filter subscriber objects so they
      //! don't go out of scope.
      //!
      std::vector<twistMFSubPtr>  twistTopicSubs_;

      //! @brief Vector to hold our twist message filters so they don't go out of scope.
      //!
      std::map<std::string, twistMFPtr> twistMessageFilters_;

      //! @brief Vector to hold our acceleration (represented as IMU) message filters so they don't go out of scope.
      //!
      std::map<std::string, imuMFPtr> accelerationMessageFilters_;

      //! @brief Vector to hold our IMU message filter subscriber objects so they
      //! don't go out of scope.
      //!
      std::vector<ros::Subscriber> imuTopicSubs_;

      //! @brief Subscribes to the set_pose topic (usually published from rviz) - a geometry_msgs/PoseWithCovarianceStamped
      //!
      ros::Subscriber setPoseSub_;

      //! @brief Node handle
      //!
      ros::NodeHandle nh_;

      //! @brief Local node handle (for params)
      //!
      ros::NodeHandle nhLocal_;

      //! @brief Message that contains our latest transform (i.e., state)
      //!
      //! We use the vehicle's latest state in a number of places, and often
      //! use it as a transform, so this is the most convenient variable to
      //! use as our global "current state" object
      //!
      geometry_msgs::TransformStamped worldBaseLinkTransMsg_;

      //! @brief Transform listener for managing coordinate transforms
      //!
      tf::TransformListener tfListener_;

      //! @brief Used for outputting debug messages
      //!
      std::ofstream debugStream_;

  };
}

#endif
