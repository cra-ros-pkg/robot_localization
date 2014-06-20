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

#include <XmlRpcException.h>

#include <Eigen/Dense>

#include <fstream>

// Handy methods for debug output
std::ostream& operator<<(std::ostream& os, const tf::Transform &trans)
{
  tf::Matrix3x3 orientation(trans.getRotation());

  double roll, pitch, yaw;
  orientation.getRPY(roll, pitch, yaw);

  os << "Origin: (" << std::setprecision(20) << trans.getOrigin().getX() << " " <<
        trans.getOrigin().getY() << " " << trans.getOrigin().getZ() << ")\n" <<
        "Rotation (RPY): (" << roll << ", " << pitch << ", " << yaw << ")\n";

  return os;
}

namespace RobotLocalization
{
  template<class Filter> class RosFilter
  {
    public:

      RosFilter() :
          nhLocal_("~")
      {
        // Ensure that anyone who uses this template uses the right
        // kind of template parameter type
        (void) static_cast<FilterBase*>((Filter*) 0);
      }

      ~RosFilter()
      {

      }

      //! @brief Retrieves the EKF's output for broadcasting
      //! @param[out] message - The standard ROS odometry message to be filled
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
          message.header.frame_id = odomFrameName_;
          message.child_frame_id = baseLinkFrameName_;
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
        nhLocal_.param("odom_frame", odomFrameName_, std::string("odom"));
        nhLocal_.param("base_link_frame", baseLinkFrameName_, std::string("base_link"));

        // Append tf_prefix if it's specified (@todo: tf2 migration)
        if (!tfPrefix_.empty() && baseLinkFrameName_.at(0) != '/')
        {
          if(baseLinkFrameName_.at(0) != '/')
          {
            baseLinkFrameName_ = "/" + tfPrefix_ + "/" + baseLinkFrameName_;
          }

          if(odomFrameName_.at(0) != '/')
          {
            odomFrameName_ = "/" + tfPrefix_ + "/" + odomFrameName_;
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
          debugStream_ << "tf_prefix is " << tfPrefix_ << "\n" << "odom_frame is " << odomFrameName_ << "\n" << "base_link_frame is " << baseLinkFrameName_
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

            // Now pull in its boolean update vector configuration
            std::vector<int> updateVec = loadUpdateConfig(odomTopicName);
            topicSubs_.push_back(
              nh_.subscribe<nav_msgs::Odometry>(odomTopic, 1,
                boost::bind(&RosFilter<Filter>::odometryCallback, this, _1, odomTopicName, updateVec, differential)));

            if (filter_.getDebug())
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

            std::vector<int> updateVec = loadUpdateConfig(poseTopicName);
            topicSubs_.push_back(
              nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(poseTopic, 1,
                boost::bind(&RosFilter<Filter>::poseCallback, this, _1, poseTopicName, odomFrameName_, updateVec, differential)));

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

            // Now pull in its boolean update vector configuration
            std::vector<int> updateVec = loadUpdateConfig(twistTopicName);
            topicSubs_.push_back(
              nh_.subscribe<geometry_msgs::TwistWithCovarianceStamped>(twistTopic, 1,
                boost::bind(&RosFilter<Filter>::twistCallback, this, _1, twistTopicName, baseLinkFrameName_, updateVec)));

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
            /// \brief imuTopic
            ///
            std::string imuTopic;
            nhLocal_.getParam(imuTopicName, imuTopic);

            // Now pull in its boolean update vector configuration and differential
            // update configuration (as this contains pose information)
            std::vector<int> updateVec = loadUpdateConfig(imuTopicName);
            topicSubs_.push_back(
              nh_.subscribe<sensor_msgs::Imu>(imuTopic, 1,
                boost::bind(&RosFilter<Filter>::imuCallback, this, _1, imuTopicName, updateVec, differential)));

            if (filter_.getDebug())
            {
              debugStream_ << "Subscribed to " << imuTopic << "\n";
            }
          }
        } while (moreParams);

        // Load up the process noise covariance (from the launch file/parameter server)
        Eigen::MatrixXd processNoiseCovariance(STATE_SIZE, STATE_SIZE);
        XmlRpc::XmlRpcValue processNoiseCovarConfig;

        if (nhLocal_.hasParam("process_noise_covariance"))
        {
          try
          {
            nhLocal_.getParam("process_noise_covariance", processNoiseCovarConfig);

            ROS_ASSERT(processNoiseCovarConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

            int matSize = processNoiseCovariance.rows();

            if (processNoiseCovarConfig.size() != matSize * matSize)
            {
              ROS_FATAL_STREAM("ERROR: process_noise_covariance matrix must have " << matSize * matSize << " values.");
            }

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
        // in the IMU message and pass them to the pose and twist callbacks

        // Get the update vector for the pose-related variables in the IMU message (attitude)
        std::vector<int> poseUpdateVec(STATE_SIZE, false);
        std::copy(updateVector.begin() + ORIENTATION_OFFSET, updateVector.begin() + ORIENTATION_OFFSET + ORIENTATION_SIZE, poseUpdateVec.begin() + ORIENTATION_SIZE);
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

        // IMU messages will likely be in their own frame, and so they will
        // likely require transforms in poseCallback and twistCallback.
        // The attitude is going to be the same regardless of the robot's position
        // in the world, and so we're safe to just transform it to base_link.
        geometry_msgs::PoseWithCovarianceStampedConstPtr pptr(posPtr);
        poseCallback(pptr, topicName, baseLinkFrameName_, poseUpdateVec, differential);

        std::vector<int> twistUpdateVec(STATE_SIZE, false);
        std::copy(updateVector.begin() + POSITION_V_OFFSET, updateVector.end(), twistUpdateVec.begin() + POSITION_V_OFFSET);
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
        twistCallback(tptr, topicName, baseLinkFrameName_, twistUpdateVec);

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
        std::vector<int> poseUpdateVec(STATE_SIZE, false);
        std::copy(updateVector.begin(), updateVector.begin() + POSE_SIZE, poseUpdateVec.begin());
        geometry_msgs::PoseWithCovarianceStamped *posPtr = new geometry_msgs::PoseWithCovarianceStamped();
        posPtr->header = msg->header;
        posPtr->pose = msg->pose;
        geometry_msgs::PoseWithCovarianceStampedConstPtr pptr(posPtr);
        poseCallback(pptr, topicName, odomFrameName_, poseUpdateVec, differential);

        // Grab the twist portion of the message and pass it to the twistCallback
        std::vector<int> twistUpdateVec(STATE_SIZE, false);
        std::copy(updateVector.begin() + POSITION_V_OFFSET, updateVector.end(), twistUpdateVec.begin() + POSITION_V_OFFSET);
        geometry_msgs::TwistWithCovarianceStamped *twistPtr = new geometry_msgs::TwistWithCovarianceStamped();
        twistPtr->header = msg->header;
        twistPtr->header.frame_id = msg->child_frame_id;
        twistPtr->twist = msg->twist;
        geometry_msgs::TwistWithCovarianceStampedConstPtr tptr(twistPtr);
        twistCallback(tptr, topicName, baseLinkFrameName_, twistUpdateVec);

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

          // First, we'll handle the pose information
          if (updateVector[StateMemberX] || updateVector[StateMemberY] || updateVector[StateMemberZ] ||
              updateVector[StateMemberRoll] || updateVector[StateMemberPitch] || updateVector[StateMemberYaw])
          {
            std::vector<int> updateVectorCorrected = updateVector;

            // Prepare the pose data for inclusion in the filter
            if (preparePose(msg, topicName + "_pose", targetFrame, updateVectorCorrected, differential, measurement, measurementCovariance))
            {
              // Store the measurement
              filter_.enqueueMeasurement(topicName + "_pose", measurement, measurementCovariance, updateVectorCorrected, msg->header.stamp.toSec());
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

        // Prepare the pose data for inclusion in the filter
        preparePose(msg, topicName, odomFrameName_, updateVector, false, measurement, measurementCovariance);
        filter_.setState(measurement);

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

          if (updateVector[StateMemberVx] || updateVector[StateMemberVy] || updateVector[StateMemberVz] ||
              updateVector[StateMemberVroll] || updateVector[StateMemberVpitch] || updateVector[StateMemberVyaw])
          {
            std::vector<int> updateVectorCorrected = updateVector;

            // Prepare the twist data for inclusion in the filter
            if (prepareTwist(msg, topicName + "_twist", targetFrame, updateVectorCorrected, measurement, measurementCovariance))
            {
              // Store the measurement
              filter_.enqueueMeasurement(topicName + "_twist", measurement, measurementCovariance, updateVectorCorrected, msg->header.stamp.toSec());
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

      void run()
      {
        ros::Time::init();

        loadParams();

        // Clear out the transform
        tf::transformTFToMsg(tf::Transform::getIdentity(), odomTrans_.transform);

        // Publisher
        ros::Publisher positionPub = nh_.advertise<nav_msgs::Odometry>("odometry/filtered", 20);
        tf::TransformBroadcaster odomTransformBroadcaster;

        ros::Rate loop_rate(frequency_);

        std::map<std::string, Eigen::VectorXd> postUpdateStates;

        while (ros::ok())
        {
          // Get latest state and publish it
          nav_msgs::Odometry filteredPosition;

          if (getFilteredOdometryMessage(filteredPosition))
          {
            odomTrans_.header.stamp = filteredPosition.header.stamp;
            odomTrans_.header.frame_id = filteredPosition.header.frame_id;
            odomTrans_.child_frame_id = filteredPosition.child_frame_id;

            odomTrans_.transform.translation.x = filteredPosition.pose.pose.position.x;
            odomTrans_.transform.translation.y = filteredPosition.pose.pose.position.y;
            odomTrans_.transform.translation.z = filteredPosition.pose.pose.position.z;
            odomTrans_.transform.rotation = filteredPosition.pose.pose.orientation;

            // Fire off the position and the transform
            positionPub.publish(filteredPosition);
            odomTransformBroadcaster.sendTransform(odomTrans_);
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
        std::vector<int> updateVector(STATE_SIZE, static_cast<int>(false));

        try
        {
          nhLocal_.getParam(topicName + "_config", topicConfig);

          ROS_ASSERT(topicConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

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

      //! @brief Prepares a pose message for integration into the filter
      //! @param[in] msg - The pose message to prepare
      //! @param[in] topicName - The name of the topic over which this message was received
      //! @param[in] targetFrame - The target tf frame
      //! @param[in, out] updateVector - The update vector for the data source
      //! @param[in, out] differential - Whether we're carrying out differential integration
      //! @param[in] measurement - The pose data converted to a measurement
      //! @param[in] measurementCovariance - The covariance of the converted measurement
      //!
      bool preparePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                       const std::string &topicName,
                       const std::string &targetFrame,
                       std::vector<int> &updateVector,
                       const bool differential,
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

        tf::poseMsgToTF(msg->pose.pose, poseTmp);

        // The selective updating of variables can be a bit tricky, especially
        // when we have differential updating to worry about. Rather than go into
        // the details of why here, suffice it to say that users must, if using
        // selective updating, be *very* careful about which variables they exclude
        // from the measurement.
        poseTmp.setOrigin(tf::Vector3(updateVector[StateMemberX] ? msg->pose.pose.position.x : 0.0,
                                      updateVector[StateMemberY] ? msg->pose.pose.position.y : 0.0,
                                      updateVector[StateMemberZ] ? msg->pose.pose.position.z : 0.0));

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, orientation);
        double orRoll, orPitch, orYaw;
        quatToRPY(orientation, orRoll, orPitch, orYaw);
        orientation.setRPY(updateVector[StateMemberRoll] ? orRoll : 0.0,
                           updateVector[StateMemberPitch] ? orPitch : 0.0,
                           updateVector[StateMemberYaw] ? orYaw : 0.0);

        // 2. robot_localization lets users configure which variables from the sensor should be
        //    fused with the filter. This is specified at the sensor level. However, the data
        //    may go through transforms before being fused with the state estimate. In that case,
        //    we need to know which of the transformed variables came from the pre-transformed
        //    "approved" variables (i.e., the ones that had "true" in their xxx_config parameter).
        //    To do this, we create a pose from the original upate vector, which contains only
        //    zeros and ones. This pose goes through the same transforms as the measurement. The
        //    non-zero values that result will be used to modify the updateVector.
        tf::Pose maskPose;
        maskPose.setOrigin(tf::Vector3(updateVector[StateMemberX],
                                       updateVector[StateMemberY],
                                       updateVector[StateMemberZ]));

        tf::Quaternion maskOrientation;
        maskOrientation.setRPY(updateVector[StateMemberRoll],
                               updateVector[StateMemberPitch],
                               updateVector[StateMemberYaw]);

        maskPose.setRotation(maskOrientation);

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
          tf::transformMsgToTF(odomTrans_.transform, prevPose);
          previousStates_.insert(std::pair<std::string, tf::Transform>(topicName, prevPose));
        }

        // 4a. Get the target frame transformation
        bool canTransform = true;
        tf::StampedTransform targetFrameTrans;

        try
        {
          // This throws an exception if it fails
          tfListener_.lookupTransform(targetFrame, poseTmp.frame_id_, poseTmp.stamp_, targetFrameTrans);
        }
        catch (tf::TransformException &ex)
        {
          ROS_WARN_STREAM("Could not obtain transform from " << poseTmp.frame_id_ <<
                           " to " << targetFrame << ". Error was " << ex.what() << "\n");

          if(filter_.getDebug())
          {
            debugStream_ << "WARNING: could not obtain transform from " << poseTmp.frame_id_ <<
                            " to " << targetFrame << ". Error was " << ex.what() << "\n";
          }

          canTransform = false;
        }

        // Transforming from a frame id to itself can fail when the tf tree isn't
        // being broadcast (e.g., for some bag files). This is the only failure that
        // would throw an exception, so check for this situation before giving up.
        if(!canTransform)
        {
          std::string msgFrame = (tfPrefix_.empty() ? poseTmp.frame_id_ : "/" + tfPrefix_ + "/" + poseTmp.frame_id_);

          if(targetFrame == msgFrame)
          {
            targetFrameTrans.setIdentity();
            canTransform = true;
          }
        }

        if(canTransform)
        {
          // Apply the target frame transformation to the
          // pose object and the mask
          poseTmp.mult(targetFrameTrans, poseTmp);
          maskPose.setOrigin(targetFrameTrans.getBasis() * maskPose.getOrigin());

          // Store the measurement as a transform for the next value (differential integration)
          curMeasurement = poseTmp;

          // Now copy the mask values back into the update vector
          updateVector[StateMemberX] = 1 * static_cast<int>(::fabs(maskPose.getOrigin().getX()) >= 1e-6);
          updateVector[StateMemberY] = 1 * static_cast<int>(::fabs(maskPose.getOrigin().getY()) >= 1e-6);
          updateVector[StateMemberZ] = 1 * static_cast<int>(::fabs(maskPose.getOrigin().getZ()) >= 1e-6);

          double roll, pitch, yaw;
          quatToRPY(maskPose.getRotation(), roll, pitch, yaw);
          updateVector[StateMemberRoll] = 1 * (roll >= 1e-6);
          updateVector[StateMemberPitch] = 1 * (pitch >= 1e-6);
          updateVector[StateMemberYaw] = 1 * (yaw >= 1e-6);

          if(filter_.getDebug())
          {
            debugStream_ << poseTmp.frame_id_ << "->" << targetFrame << " transform:\n" << targetFrameTrans <<
                            "\nAfter applying transform to " << targetFrame << ", update vector is:\n" << updateVector <<
                            "\nAfter applying transform to " << targetFrame << ", measurement is:\n" << poseTmp << "\n";
          }

          poseTmp.frame_id_ = targetFrame;

          // If we're in differential mode, we want to make sure
          // we have a previous measurement to work with.
          canTransform = (!differential || previousMeasurements_.count(topicName) > 0);
          if(differential && canTransform)
          {
            // 4b. If we are carrying out differential integration and
            // we have a previous measurement for this sensor,then we
            // need to apply the inverse of that measurement to this new
            // measurement.

            // Even if we're not using all of the variables from this sensor,
            // we need to use the whole measurement to determine the delta
            // to the new measurement
            tf::Pose prevMeasurement = previousMeasurements_[topicName];

            // Determine the position offset by removing the previous measurement.
            // This will also remove part (nearly all) of this measurement's rotation,
            // leaving you with a value that can be added (transformed) onto the current
            // state.
            poseTmp.setOrigin(prevMeasurement.inverse() * poseTmp.getOrigin());
            poseTmp.setRotation(poseTmp.getRotation() * prevMeasurement.getRotation().inverse());

            //poseTmp.setData(prevMeasurement.inverseTimes(poseTmp));

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

            //poseTmp.mult(prevPose, poseTmp);

            tf::Vector3 newPosition = prevPose * poseTmp.getOrigin();
            tf::Quaternion newOrientation = prevPose.getRotation() * poseTmp.getRotation();

            poseTmp.setOrigin(newPosition);
            poseTmp.setRotation(newOrientation);

            if (filter_.getDebug())
            {
              debugStream_ << "Transforming to align with state. State is:\n" << prevPose <<
                              "\nMeasurement is now:\n" << poseTmp << "\n";
            }
          }

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
          quatToRPY(poseTmp.getRotation(), roll, pitch, yaw);
          measurement(StateMemberRoll) = roll;
          measurement(StateMemberPitch) = pitch;
          measurement(StateMemberYaw) = yaw;

          measurementCovariance.block(0, 0, POSE_SIZE, POSE_SIZE) = covarianceRotated.block(0, 0, POSE_SIZE, POSE_SIZE);

          // Store the measurement so we can remove it
          previousMeasurements_[topicName] = curMeasurement;
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
      //! @param[in] updateVector - The update vector for the data source
      //! @param[in] measurement - The twist data converted to a measurement
      //! @param[in] measurementCovariance - The covariance of the converted measurement
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

        // 1. Get the measurement into a tf-friendly transform (twist) object. Zero out
        // values we don't want to use.
        tf::Vector3 twistLin(updateVector[StateMemberVx] ? msg->twist.twist.linear.x : 0.0,
                             updateVector[StateMemberVy] ? msg->twist.twist.linear.y : 0.0,
                             updateVector[StateMemberVz] ? msg->twist.twist.linear.z : 0.0);
        tf::Vector3 twistRot(updateVector[StateMemberVroll] ? msg->twist.twist.angular.x : 0.0,
                             updateVector[StateMemberVpitch] ? msg->twist.twist.angular.y : 0.0,
                             updateVector[StateMemberVyaw] ? msg->twist.twist.angular.z : 0.0);

        // Set relevant header info
        std::string msgFrame = (msg->header.frame_id == "" ? baseLinkFrameName_ : msg->header.frame_id);

        // 2. robot_localization lets users configure which variables from the sensor should be
        //    fused with the filter. This is specified at the sensor level. However, the data
        //    may go through transforms before being fused with the state estimate. In that case,
        //    we need to know which of the transformed variables came from the pre-transformed
        //    "approved" variables (i.e., the ones that had "true" in their xxx_config parameter).
        //    To do this, we create a pose from the original upate vector, which contains only
        //    zeros and ones. This pose goes through the same transforms as the measurement. The
        //    non-zero values that result will be used to modify the updateVector.
        tf::Vector3 maskTwistLin(updateVector[StateMemberVx],
                                 updateVector[StateMemberVy],
                                 updateVector[StateMemberVz]);
        tf::Vector3 maskTwistRot(updateVector[StateMemberVroll],
                                 updateVector[StateMemberVpitch],
                                 updateVector[StateMemberVyaw]);

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

        try
        {
          tfListener_.lookupTransform(targetFrame, msgFrame, msg->header.stamp, targetFrameTrans);
        }
        catch (tf::TransformException &ex)
        {
          ROS_WARN_STREAM("Could not obtain transform from " << msgFrame <<
                           " to " << targetFrame << ". Error was " << ex.what());

          if(filter_.getDebug())
          {
            debugStream_ << "WARNING: Could not obtain transform from " << msgFrame <<
                            " to " << targetFrame << ". Error was " << ex.what();
          }

          canTransform = false;
        }

        if(!canTransform)
        {
          // Transforming from a frame id to itself can fail when the tf tree isn't
          // being broadcast (e.g., for some bag files). Handle this situation.
          std::string msgFrame = (tfPrefix_.empty() ? msg->header.frame_id : "/" + tfPrefix_ + "/" + msgFrame);

          if(targetFrame == msgFrame)
          {
            targetFrameTrans.setIdentity();
            canTransform = true;
          }
        }

        if(canTransform)
        {
          // Transform to correct frame. Note that we can get linear velocity
          // as a result of the sensor offset and rotational velocity
          twistRot = targetFrameTrans.getBasis() * twistRot;
          twistLin = targetFrameTrans.getBasis() * twistLin - targetFrameTrans.getOrigin().cross(twistRot);
          maskTwistRot = targetFrameTrans.getBasis() * maskTwistRot;
          maskTwistLin = targetFrameTrans.getBasis() * maskTwistLin;

          // Now copy the mask values back into the update vector
          updateVector[StateMemberVx] = 1 * static_cast<int>(::fabs(maskTwistLin.getX()) >= 1e-6);
          updateVector[StateMemberVy] = 1 * static_cast<int>(::fabs(maskTwistLin.getY()) >= 1e-6);
          updateVector[StateMemberVz] = 1 * static_cast<int>(::fabs(maskTwistLin.getZ()) >= 1e-6);
          updateVector[StateMemberVroll] = 1 * static_cast<int>(::fabs(maskTwistRot.getX()) >= 1e-6);
          updateVector[StateMemberVpitch] = 1 * static_cast<int>(::fabs(maskTwistRot.getY()) >= 1e-6);
          updateVector[StateMemberVyaw] = 1 * static_cast<int>(::fabs(maskTwistRot.getZ()) >= 1e-6);

          if(filter_.getDebug())
          {
            debugStream_ << msg->header.frame_id << "->" << targetFrame << " transform:\n" << targetFrameTrans <<
                            "\nAfter applying transform to " << targetFrame << ", update vector is:\n" << updateVector <<
                            "\nAfter applying transform to " << targetFrame << ", measurement is:\n" <<
                            "Linear: " << twistRot << "\nRotational: " << twistRot << "\n";
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

      //! @brief Utility method for converting quaternion to RPY
      //! @param[in] quat - The quaternion to convert
      //! @param[in] roll - The converted roll
      //! @param[in] pitch - The converted pitch
      //! @param[in] yaw - The converted yaw
      //!
      inline void quatToRPY(const tf::Quaternion &quat, double &roll, double &pitch, double &yaw)
      {
        tf::Matrix3x3 orTmp(quat);
        orTmp.getRPY(roll, pitch, yaw);
      }

      //! @brief The frequency of the run loop
      //!
      double frequency_;

      //! @brief tf prefix
      //!
      std::string tfPrefix_;

      //! @brief tf frame name for the robot's body frame
      //!
      std::string baseLinkFrameName_;

      //! @brief tf frame name for the robot's odometry (world) frame
      //!
      std::string odomFrameName_;

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
      //! \p odomFrameName_), (2) "subtract"  the previous measurement from
      //! the current measurement, and then (3) transform it again by the previous
      //! state estimate. This holds the measurements used for step (2).
      //!
      std::map<std::string, tf::Transform> previousMeasurements_;

      //! @brief Stores the last state estimate at the time the previous
      //! measurement from this sensor was captured
      //!
      //! To carry out differential integration, we have to (1) transform
      //! that into the target frame (probably the frame specified by
      //! \p odomFrameName_), (2)  "subtract" the previous measurement from
      //! the current measurement, and then (3) transform it again by the previous
      //! state estimate. This holds the measurements used for step (3).
      //!
      std::map<std::string, tf::Transform> previousStates_;

      //! @brief Vector to hold our subscriber objects so they don't go out
      //! of scope.
      //!
      std::vector<ros::Subscriber> topicSubs_;

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
      geometry_msgs::TransformStamped odomTrans_;

      //! @brief Transform listener for managing coordinate transforms
      //!
      tf::TransformListener tfListener_;

      //! @brief Used for outputting debug messages
      //!
      std::ofstream debugStream_;

  };
}

#endif
