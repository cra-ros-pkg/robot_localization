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

// Temporary measure until tf moves to boost::signals2
#define BOOST_SIGNALS_NO_DEPRECATION_WARNING

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

  os << "Origin: (" << trans.getOrigin().getX() << " " << trans.getOrigin().getY() << " " << trans.getOrigin().getZ() << ")\n"
     << "Rotation (RPY): (" << roll << ", " << pitch << ", " << yaw << ")\n";

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
        if (filter.getInitializedStatus())
        {
          const Eigen::VectorXd state = filter.getState();
          const Eigen::MatrixXd estimateErrorCovariance = filter.getEstimateErrorCovariance();

          // Convert from roll, pitch, and yaw back to quaternion for
          // orientation values
          tf::Quaternion quat;
          quat.setRPY(state(StateMemberRoll), state(StateMemberPitch), state(StateMemberYaw));

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

        return filter.getInitializedStatus();
      }

      //! @brief Loads all parameters from file
      //!
      void loadParams()
      {
        // Grab the debug param. This will produce a LOT of output.
        bool debug;
        nhLocal_.param("debug", debug, false);

        if (debug)
        {
          std::string debugOutFile;
          nhLocal_.param("debug_out_file", debugOutFile, std::string("ros_filter.txt"));
          debugStream_.open(debugOutFile.c_str());

          // Make sure we succeeeded
          if(debugStream_.is_open())
          {
            filter.setDebug(debug, &debugStream_);
          }
          else
          {
            ROS_WARN_STREAM("ros_filter::loadParams() - unable to create debug output file " << debugOutFile);
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
        filter.setSensorTimeout(sensorTimeout);

        // Debugging writes to file
        if (filter.getDebug())
        {
          debugStream_ << "tf_prefix is " << tfPrefix_ << "\n" << "odom_frame is " << odomFrameName_ << "\n" << "base_link_frame is " << baseLinkFrameName_
            << "\n" << "frequency is " << frequency_ << "\n" << "sensor_timeout is " << filter.getSensorTimeout() << "\n";
        }

        // Create a subscriber for manually setting/resetting pose
        setPoseSub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("set_pose", 1, &RosFilter<Filter>::setPoseCallback, this);

        // Init the last last measurement time so we don't get a huge initial delta
        filter.setLastMeasurementTime(ros::Time::now().toSec());
        filter.setLastUpdateTime(ros::Time::now().toSec());

        // Now pull in each topic to which we want to subscribe.
        // Start with odom.
        size_t topicInd = 0;
        bool moreParams = false;
        do
        {
          // Build the string in the form of "odomX", where X is the odom topic number
          std::stringstream ss;
          ss << "odom" << topicInd++;
          std::string odomTopicName = ss.str();

          // Check if we have any
          moreParams = nhLocal_.hasParam(odomTopicName);

          if (moreParams)
          {
            // Subscribe using boost::bind, which lets us append arbitrary data,
            // in this case, the topic name (e.g., odom0 or odom1)
            std::string odomTopic;
            nhLocal_.getParam(odomTopicName, odomTopic);

            // Now pull in its boolean update vector configuration and differential
            // update configuration (as this contains pose information)
            std::vector<int> updateVec = loadUpdateConfig(odomTopicName);
            std::vector<int> diffVec = loadDifferentialConfig(odomTopicName);
            topicSubs_.push_back(
              nh_.subscribe<nav_msgs::Odometry>(odomTopic, 1,
                boost::bind(&RosFilter<Filter>::odometryCallback, this, _1, odomTopicName, updateVec, diffVec)));

            if (filter.getDebug())
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
            std::string poseTopic;
            nhLocal_.getParam(poseTopicName, poseTopic);

            // Now pull in its boolean update vector configuration and differential
            // update configuration (as this contains pose information)
            std::vector<int> updateVec = loadUpdateConfig(poseTopicName);
            std::vector<int> diffVec = loadDifferentialConfig(poseTopicName);
            topicSubs_.push_back(
              nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(poseTopic, 1,
                boost::bind(&RosFilter<Filter>::poseCallback, this, _1, poseTopicName, odomFrameName_, updateVec, diffVec)));

            if (filter.getDebug())
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

            if (filter.getDebug())
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
            std::string imuTopic;
            nhLocal_.getParam(imuTopicName, imuTopic);

            // Now pull in its boolean update vector configuration and differential
            // update configuration (as this contains pose information)
            std::vector<int> updateVec = loadUpdateConfig(imuTopicName);
            std::vector<int> diffVec = loadDifferentialConfig(imuTopicName);
            topicSubs_.push_back(
              nh_.subscribe<sensor_msgs::Imu>(imuTopic, 1,
                boost::bind(&RosFilter<Filter>::imuCallback, this, _1, imuTopicName, updateVec, diffVec)));

            if (filter.getDebug())
            {
              debugStream_ << "Subscribed to " << imuTopic << "\n";
            }
          }
        } while (moreParams);

        // Load up the process noise covariance (from the launch file)
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

            if (filter.getDebug())
            {
              debugStream_ << "Process noise covariance is:\n" << processNoiseCovariance;
            }
          }
          catch (XmlRpc::XmlRpcException &e)
          {
            ROS_ERROR_STREAM(
              "ERROR reading sensor config: " << e.getMessage() << " for process_noise_covariance (type: " << processNoiseCovarConfig.getType() << ")");
          }

          filter.setProcessNoiseCovariance(processNoiseCovariance);
        }
      }

      //! @brief Callback method for receiving all IMU messages
      //! @param[in] msg - The ROS IMU message to take in.
      //! @param[in] topicName - The name of the IMU data topic (we support many)
      //! @param[in] updateVector - Specifies which variables we want to update from this measurement
      //!
      void imuCallback(const sensor_msgs::Imu::ConstPtr &msg, const std::string &topicName,
                       const std::vector<int> &updateVector, const std::vector<int> &differentialVector)
      {
        if (filter.getDebug())
        {
          debugStream_ << "------ RosFilter::imuCallback (" << topicName << ") ------\n";
          debugStream_ << "IMU message:\n";
          debugStream_ << *msg;
        }

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

        // IMU messages are in the IMU frame, but once the attitude is converted
        // to base_link, we can work with it directly, even though attitude is technically
        // reported in the odom frame.
        geometry_msgs::PoseWithCovarianceStampedConstPtr pptr(posPtr);
        poseCallback(pptr, topicName, baseLinkFrameName_, poseUpdateVec, differentialVector);

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

        if (filter.getDebug())
        {
          debugStream_ << "\n----- /RosFilter::imuCallback (" << topicName << ") ------\n";
        }
      }

      //! @brief Callback method for receiving all odometry messages
      //! @param[in] msg - The ROS odometry message to take in.
      //! @param[in] topicName - The name of the odometry topic (we support many)
      //! @param[in] updateVector - Specifies which variables we want to update from this measurement
      //!
      void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg, const std::string &topicName,
                            const std::vector<int> &updateVector, const std::vector<int> &differentialVector)
      {
        if (filter.getDebug())
        {
          debugStream_ << "------ RosFilter::odometryCallback (" << topicName << ") ------\n" << "Odometry message:\n" << *msg;
        }

        // Grab the pose portion of the message and pass it to the poseCallback
        std::vector<int> poseUpdateVec(STATE_SIZE, false);
        std::copy(updateVector.begin(), updateVector.begin() + POSE_SIZE, poseUpdateVec.begin());
        geometry_msgs::PoseWithCovarianceStamped *posPtr = new geometry_msgs::PoseWithCovarianceStamped();
        posPtr->header = msg->header;
        posPtr->pose = msg->pose;
        geometry_msgs::PoseWithCovarianceStampedConstPtr pptr(posPtr);
        poseCallback(pptr, topicName, odomFrameName_, poseUpdateVec, differentialVector);

        // Grab the twistr portion of the message and pass it to the twistCallback
        std::vector<int> twistUpdateVec(STATE_SIZE, false);
        std::copy(updateVector.begin() + POSITION_V_OFFSET, updateVector.end(), twistUpdateVec.begin() + POSITION_V_OFFSET);
        geometry_msgs::TwistWithCovarianceStamped *twistPtr = new geometry_msgs::TwistWithCovarianceStamped();
        twistPtr->header = msg->header;
        twistPtr->header.frame_id = msg->child_frame_id;
        twistPtr->twist = msg->twist;
        geometry_msgs::TwistWithCovarianceStampedConstPtr tptr(twistPtr);
        twistCallback(tptr, topicName, baseLinkFrameName_, twistUpdateVec);

        if (filter.getDebug())
        {
          debugStream_ << "\n----- /RosFilter::odometryCallback (" << topicName << ") ------\n";
        }
      }

      //! @brief Callback method for receiving all pose messages
      //! @param[in] msg - The ROS stamped pose with covariance message to take in
      //! @param[in] topicName - The name of the pose topic (we support many)
      //! @param[in] targetFrame - The tf frame name into which we will transform this measurement
      //! @param[in] updateVector - Specifies which variables we want to update from this measurement
      //!
      void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                        const std::string &topicName,
                        const std::string &targetFrame,
                        const std::vector<int> &updateVector,
                        const std::vector<int> &differentialVector)
      {
        if (filter.getDebug())
        {
          debugStream_ << "------ RosFilter::poseCallback (" << topicName << ") ------\n" << "Pose message:\n" << *msg;
        }

        if (lastMessageTimes_.count(topicName) == 0)
        {
          lastMessageTimes_.insert(std::pair<std::string, ros::Time>(topicName, msg->header.stamp));
        }

        // Make sure this message is newer than the last one
        if (msg->header.stamp >= lastMessageTimes_[topicName])
        {
          if (filter.getDebug())
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
            // Prepare the pose data for inclusion in the filter
            if (preparePose(msg, topicName + "_pose", targetFrame, updateVector, differentialVector, measurement, measurementCovariance))
            {
              // Store the measurement
              filter.enqueueMeasurement(measurement, measurementCovariance, updateVector, msg->header.stamp.toSec());
            }
          }
          else
          {
            if (filter.getDebug())
            {
              debugStream_ << "Update vector for " << topicName << " is such that none of its state variables will be updated\n";
            }
          }

          lastMessageTimes_[topicName] = msg->header.stamp;

          if (filter.getDebug())
          {
            debugStream_ << "Last message time for " << topicName << " is now " << lastMessageTimes_[topicName] << "\n";
          }
        }
        else
        {
          if (filter.getDebug())
          {
            debugStream_ << "Message is too old. Last message time for " << topicName << " is " << lastMessageTimes_[topicName] << ", current message time is "
              << msg->header.stamp << ".\n";
          }
        }

        if (filter.getDebug())
        {
          debugStream_ << "\n----- /RosFilter::poseCallback (" << topicName << ") ------\n";
        }
      }

      //! @brief Callback method for manually setting/resetting the internal pose estimate
      //! @param[in] msg - The ROS stamped pose with covariance message to take in
      //!
      void setPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
      {
        if (filter.getDebug())
        {
          debugStream_ << "------ RosFilter::setPoseCallback ------\n";
          debugStream_ << "Pose message:\n";
          debugStream_ << *msg;
        }

        // Get rid of any initial poses (pretend we've never had a measurement)
        initialPoses_.clear();

        // When we pass this pose in as a measurement, it will think it is the first
        // measurement from a made-up sensor, and will therefore just use it as
        // an initial pose from which later poses with that name will be subtracted.
        // We need to give it an initial pose of (0, 0) for this "sensor" so that it
        // subtracts that from the value given, resulting in the original value. This is
        // equivalent to a transform that is the identity.
        tf::Transform empty;
        empty.setIdentity();
        initialPoses_.insert(std::pair<std::string, tf::Transform>("setPose", empty));

        // Set the state vector to the reported pose
        Eigen::VectorXd measurement(STATE_SIZE);
        Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);
        const std::vector<int> updateVector(STATE_SIZE, true);
        const std::vector<int> differentialVector(POSE_SIZE, false);

        // Prepare the pose data for inclusion in the filter
        preparePose(msg, "setPose", odomFrameName_, updateVector, differentialVector, measurement, measurementCovariance);
        filter.setState(measurement);

        if (filter.getDebug())
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
        if (filter.getDebug())
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
          if (filter.getDebug())
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
            // Prepare the twist data for inclusion in the filter
            if (prepareTwist(msg, topicName + "_twist", targetFrame, updateVector, measurement, measurementCovariance))
            {
              // Store the measurement
              filter.enqueueMeasurement(measurement, measurementCovariance, updateVector, msg->header.stamp.toSec());
            }
          }
          else
          {
            if (filter.getDebug())
            {
              debugStream_ << "Update vector for " << topicName << " is such that none of its state variables will be updated\n";
            }
          }

          lastMessageTimes_[topicName] = msg->header.stamp;

          if (filter.getDebug())
          {
            debugStream_ << "Last message time for " << topicName << " is now " << lastMessageTimes_[topicName] << "\n";
          }
        }
        else
        {
          if (filter.getDebug())
          {
            debugStream_ << "Message is too old. Last message time for " << topicName << " is " << lastMessageTimes_[topicName] << ", current message time is "
              << msg->header.stamp << ".\n";
          }
        }

        if (filter.getDebug())
        {
          debugStream_ << "\n----- /RosFilter::twistCallback (" << topicName << ") ------\n";
        }
      }

      void run()
      {
        ros::Time::init();

        loadParams();

        // Publisher
        ros::Publisher positionPub = nh_.advertise<nav_msgs::Odometry>("odometry/filtered", 20);
        tf::TransformBroadcaster odomTransformBroadcaster;
        geometry_msgs::TransformStamped odomTrans;

        ros::Rate loop_rate(frequency_);

        while (ros::ok())
        {
          // Get waypoint from manager, publish it
          nav_msgs::Odometry filteredPosition;

          if (getFilteredOdometryMessage(filteredPosition))
          {
            odomTrans.header.stamp = filteredPosition.header.stamp;
            odomTrans.header.frame_id = filteredPosition.header.frame_id;
            odomTrans.child_frame_id = filteredPosition.child_frame_id;

            odomTrans.transform.translation.x = filteredPosition.pose.pose.position.x;
            odomTrans.transform.translation.y = filteredPosition.pose.pose.position.y;
            odomTrans.transform.translation.z = filteredPosition.pose.pose.position.z;
            odomTrans.transform.rotation = filteredPosition.pose.pose.orientation;

            // Fire off the position and the transform
            positionPub.publish(filteredPosition);
            odomTransformBroadcaster.sendTransform(odomTrans);
          }

          // The spin will enqueue all the available callbacks
          ros::spinOnce();

          // Now we'll integrate any measurements we've received
          filter.integrateMeasurements(ros::Time::now().toSec());

          loop_rate.sleep();
        }
      }

    protected:

      Filter filter;

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

      //! @brief Loads differential settings from the config file
      //! @param[in] topicName - The name of the topic for which to load settings
      //! @return The differential settings (boolean update vector) for the topic in question
      //!
      std::vector<int> loadDifferentialConfig(const std::string &topicName)
      {
        XmlRpc::XmlRpcValue diffConfig;
        std::vector<int> differentialVector(POSE_SIZE, static_cast<int>(false));

        try
        {
          std::string diffTopic = topicName + "_differential";

          if(nhLocal_.hasParam(diffTopic))
          {
            nhLocal_.getParam(diffTopic, diffConfig);

            ROS_ASSERT(diffConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

            for (int i = 0; i < diffConfig.size(); i++)
            {
              // The double cast looks strange, but we'll get exceptions if we
              // remove the cast to bool. vector<bool> is discouraged, so differentialVector
              // uses integers
              differentialVector[i] = static_cast<int>(static_cast<bool>(diffConfig[i]));
            }
          }
          // else, all will be false (which is fine)
        }
        catch (XmlRpc::XmlRpcException &e)
        {
          ROS_ERROR_STREAM("ERROR reading sensor differential config: " << e.getMessage() << " for topic " <<
                           topicName << " (type: " << diffConfig.getType() << ", expected: " << XmlRpc::XmlRpcValue::TypeArray << ")");
        }

        return differentialVector;
      }

      //! @brief Prepares a pose message for integration into the filter
      //! @param[in] msg - The pose message to prepare
      //! @param[in] topicName - The name of the topic over which this message was received
      //! @param[in] targetFrame - The target tf frame
      //! @param[in] updateVector - The update vector for the data source
      //! @param[in] measurement - The pose data converted to a measurement
      //! @param[in] measurementCovariance - The covariance of the converted measurement
      //!
      bool preparePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                       const std::string &topicName,
                       const std::string &targetFrame,
                       const std::vector<int> &updateVector,
                       const std::vector<int> &differentialVector,
                       Eigen::VectorXd &measurement,
                       Eigen::MatrixXd &measurementCovariance)
      {
        if (filter.getDebug())
        {
          debugStream_ << "------ EkfNavigation::preparePose (" << topicName << ") ------\n";
        }

        // Transform this measurement into the base_link frame
        tf::Stamped<tf::Pose> poseTmp;
        tf::Stamped<tf::Pose> poseCorrected;
        tf::Pose poseFinal; // Can't use a stamped pose for multiplication by the initial pose

        // We'll need to rotate the covariance as well
        Eigen::MatrixXd covarianceRotated(POSE_SIZE, POSE_SIZE);
        covarianceRotated.setZero();

        // Determine if the message had a frame id associated with it. If not, assume odom.
        poseTmp.frame_id_ = (msg->header.frame_id == "" ? odomFrameName_ : msg->header.frame_id);
        poseTmp.stamp_ = msg->header.stamp;

        // Convert the position to tf-compatible structures, but only for values we want
        poseTmp.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));

        // Convert the orientation to tf-compatible structures
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, orientation);

        // Handle cases of bad (all-zero) quaternions
        if (orientation.x() == orientation.y() && orientation.y() == orientation.z() && orientation.z() == orientation.w() && orientation.w() == 0)
        {
          ROS_WARN("Bad (all zero) odometry quaternion specified. Correcting...");

          orientation.setRPY(0, 0, 0);
        }

        // Set the rotation of the pose object
        poseTmp.setRotation(orientation);

        // Copy the measurement's covariance matrix so that we can rotate it later
        for (size_t i = 0; i < POSE_SIZE; i++)
        {
          for (size_t j = 0; j < POSE_SIZE; j++)
          {
            covarianceRotated(i, j) = (updateVector[i] && updateVector[j] ? msg->pose.covariance[POSE_SIZE * i + j] : 0.0);
          }
        }

        bool transformSuccessful = true;
        try
        {
          // If there's some lag in the reporting of the transform, we want to ensure we have it
          // before attempting the transform
          transformSuccessful = tfListener_.waitForTransform(targetFrame, poseTmp.frame_id_, poseTmp.stamp_, ros::Duration(0.05));

          if (transformSuccessful)
          {
            tf::StampedTransform trans;
            tfListener_.lookupTransform(targetFrame, poseTmp.frame_id_, poseTmp.stamp_, trans);

            // Transform to correct frame
            poseCorrected.setOrigin(trans * poseTmp.getOrigin());
            poseCorrected.setRotation(trans * poseTmp.getRotation());

            // Now create an augmented matrix that contains
            // a 3D rotation matrix in the upper-left quadrant,
            // an identity matrix in the lower-right quadrant,
            // and zeros elsewhere.
            double roll, pitch, yaw;
            quatToRPY(trans.getRotation(), roll, pitch, yaw);

            double cr = cos(roll);
            double cp = cos(pitch);
            double cy = cos(yaw);
            double sr = sin(roll);
            double sp = sin(pitch);
            double sy = sin(yaw);

            Eigen::MatrixXd rot6d(POSE_SIZE, POSE_SIZE);
            rot6d.setIdentity();
            rot6d(0, 0) = cp * cy;
            rot6d(0, 1) = cy * sr * sp - cr * sy;
            rot6d(0, 2) = cr * cy * sp + sr * sy;
            rot6d(1, 0) = cp * sy;
            rot6d(1, 1) = cr * cy + sr * sp * sy;
            rot6d(1, 2) = -cy * sr + cr * sp * sy;
            rot6d(2, 0) = -sp;
            rot6d(2, 1) = cp * sr;
            rot6d(2, 2) = cr * cp;

            // Rotate the covariance
            covarianceRotated = rot6d * covarianceRotated * rot6d;
          }
        }
        catch (tf::TransformException &ex)
        {
          ROS_ERROR_STREAM("Odometry pose transform error: " << ex.what());
          transformSuccessful = false;
        }

        // Transforming from a frame id to itself can fail when the tf tree isn't
        // being broadcast (e.g., for some bag files). Handle this situation.
        std::string msgFrame = (tfPrefix_.empty() ? poseTmp.frame_id_ : "/" + tfPrefix_ + "/" + poseTmp.frame_id_);

        if(!transformSuccessful && targetFrame == msgFrame)
        {
          poseCorrected = poseTmp;

          transformSuccessful = true;
        }

        if (!transformSuccessful)
        {
          ROS_WARN_STREAM("WARNING: Transform " << poseTmp.frame_id_ << "->" << targetFrame << " was unsuccessful for topic " << topicName);

          if (filter.getDebug())
          {
            debugStream_ << "Transform from " << poseTmp.frame_id_ << "->" << targetFrame << " failed for topic " << topicName
              << ". Ignoring pose measurement.\n";
          }

          return false;
        }
        else
        {
          // Now that we've got a corrected pose, zero out the values we don't want
          tf::Vector3 finalPosition = poseCorrected.getOrigin();
          finalPosition.setX(updateVector[StateMemberX] ? finalPosition.getX() : 0.0);
          finalPosition.setY(updateVector[StateMemberY] ? finalPosition.getY() : 0.0);
          finalPosition.setZ(updateVector[StateMemberZ] ? finalPosition.getZ() : 0.0);
          poseCorrected.setOrigin(finalPosition);

          tf::Quaternion finalOrientation = poseCorrected.getRotation();
          double roll, pitch, yaw;
          quatToRPY(finalOrientation, roll, pitch, yaw);
          roll = (updateVector[StateMemberRoll] ? roll : 0.0);
          pitch = (updateVector[StateMemberPitch] ? pitch : 0.0);
          yaw = (updateVector[StateMemberYaw] ? yaw : 0.0);
          finalOrientation.setRPY(roll, pitch, yaw);
          poseCorrected.setRotation(finalOrientation);

          // If this is our first reading from a given source, then
          // we want to capture its value as a transform for all future
          // measurements from that sensor...
          if (initialPoses_.count(topicName) == 0)
          {
            // ...however, this is only true for variables that we
            // specified as being differentially updated.
            tf::Transform trans = poseCorrected;

            if (filter.getDebug())
            {
              debugStream_ << "Initial measurement for " << topicName << " (before differential update config applied) is :\n";
              debugStream_ << trans;
            }

            trans.getOrigin().setX((static_cast<bool>(differentialVector[StateMemberX]) ? trans.getOrigin().getX() : 0));
            trans.getOrigin().setY((static_cast<bool>(differentialVector[StateMemberY]) ? trans.getOrigin().getY() : 0));
            trans.getOrigin().setZ((static_cast<bool>(differentialVector[StateMemberZ]) ? trans.getOrigin().getZ() : 0));

            // Now handle rotation
            double roll, pitch, yaw;
            quatToRPY(trans.getRotation(), roll, pitch, yaw);
            roll = (static_cast<bool>(differentialVector[StateMemberRoll]) ? roll : 0);
            pitch = (static_cast<bool>(differentialVector[StateMemberPitch]) ? pitch : 0);
            yaw = (static_cast<bool>(differentialVector[StateMemberYaw]) ? yaw : 0);
            tf::Quaternion yawQuat;
            yawQuat.setRPY(roll, pitch, yaw);
            trans.setRotation(yawQuat);

            trans = trans.inverse();

            if (filter.getDebug())
            {
              debugStream_ << "Corrective transform for " << topicName << " is :\n";
              debugStream_ << trans;
            }

            initialPoses_.insert(std::pair<std::string, tf::Transform>(topicName, trans));
          }

          // Correct the position and orientation
          poseFinal = poseCorrected;
          poseFinal = initialPoses_[topicName] * poseFinal;

          if (filter.getDebug())
          {
            debugStream_ << "Corrected odometry pose is :\n";
            debugStream_ << "(" << poseFinal.getOrigin().x() << ", " << poseFinal.getOrigin().y() << ", " << poseFinal.getOrigin().z() << ")\n";
          }

          // Copy the data into the measurement vector. We probably don't need to zero out
          // the values, as that was done previously, but in case the latest transform
          // introduced non-zero values to any of the linear dimensions, zero them again
          measurement(StateMemberX) = (updateVector[StateMemberX] ? poseFinal.getOrigin().x() : 0.0);
          measurement(StateMemberY) = (updateVector[StateMemberY] ? poseFinal.getOrigin().y() : 0.0);
          measurement(StateMemberZ) = (updateVector[StateMemberZ] ? poseFinal.getOrigin().z() : 0.0);

          // The filter needs roll, pitch, and yaw values instead of quaternions
          quatToRPY(poseFinal.getRotation(), roll, pitch, yaw);

          if (filter.getDebug())
          {
            debugStream_ << "Corrected odometry orientation is :\n";
            debugStream_ << "(" << roll << ", " << pitch << ", " << yaw << ")\n";
          }

          measurement(StateMemberRoll) = (updateVector[StateMemberRoll] ? roll : 0.0);
          measurement(StateMemberPitch) = (updateVector[StateMemberPitch] ? pitch : 0.0);
          measurement(StateMemberYaw) = (updateVector[StateMemberYaw] ? yaw : 0.0);

          // Copy the covariances
          measurementCovariance.block(0, 0, POSE_SIZE, POSE_SIZE) = covarianceRotated.block(0, 0, POSE_SIZE, POSE_SIZE);

        } // end if(!transformSuccessful)

        if (filter.getDebug())
        {
          debugStream_ << "\n----- /EkfNavigation::preparePose (" << topicName << ") ------\n";
        }

        return true;
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
                        const std::vector<int> &updateVector,
                        Eigen::VectorXd &measurement,
                        Eigen::MatrixXd &measurementCovariance)
      {
        if (filter.getDebug())
        {
          debugStream_ << "------ EkfNavigation::prepareTwist (" << topicName << ") ------\n";
        }

        // It's very unlikely that someone will hand us a twist with
        // velocities that are NOT in base_link, but in case they do,
        // we need to handle it
        tf::Stamped<tf::Pose> twistTmp;
        tf::Stamped<tf::Pose> twistCorrected;

        // We'll need to rotate the covariance as well
        Eigen::MatrixXd covarianceRotated(TWIST_SIZE, TWIST_SIZE);
        covarianceRotated.setZero();

        twistTmp.frame_id_ = (msg->header.frame_id == "" ? baseLinkFrameName_ : msg->header.frame_id);
        twistTmp.stamp_ = msg->header.stamp;

        // Don't filter out anything here. We need to transform this,
        // then we can zero out the components we're not measuring
        twistTmp.setOrigin(tf::Vector3(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z));

        // It's a velocity, so it gets reported in RPY, but we are
        // transforming it like it's a position, so we have to
        // convert it to a quaternion
        tf::Quaternion orientation;
        orientation.setRPY(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
        twistTmp.setRotation(orientation);

        // Set relevant header info
        twistTmp.frame_id_ = (msg->header.frame_id == "" ? baseLinkFrameName_ : msg->header.frame_id);
        twistTmp.stamp_ = msg->header.stamp;

        // Copy the measurement's covariance matrix so that we can rotate it later
        for (size_t i = 0; i < TWIST_SIZE; i++)
        {
          for (size_t j = 0; j < TWIST_SIZE; j++)
          {
            covarianceRotated(i, j) = (updateVector[i + POSITION_V_OFFSET] && updateVector[j + POSITION_V_OFFSET] ? msg->twist.covariance[TWIST_SIZE * i + j] : 0.0);
          }
        }

        bool transformSuccessful = true;
        try
        {
          tf::StampedTransform rotationTrans;

          // If there's some lag in the reporting of the transform, we want to ensure we have it
          // before attempting the transform
          transformSuccessful = tfListener_.waitForTransform(targetFrame, twistTmp.frame_id_, twistTmp.stamp_, ros::Duration(0.05));

          if (transformSuccessful)
          {
            tf::StampedTransform trans;
            tfListener_.lookupTransform(targetFrame, twistTmp.frame_id_, twistTmp.stamp_, trans);

            // Transform to correct frame
            twistCorrected.setOrigin(trans * twistTmp.getOrigin());
            twistCorrected.setRotation(trans * twistTmp.getRotation());

            // Now create an augmented matrix that contains
            // a 3D rotation matrix in the upper-left quadrant,
            // an identity matrix in the lower-right quadrant,
            // and zeros elsewhere.
            double roll, pitch, yaw;
            quatToRPY(trans.getRotation(), roll, pitch, yaw);

            double cr = cos(roll);
            double cp = cos(pitch);
            double cy = cos(yaw);
            double sr = sin(roll);
            double sp = sin(pitch);
            double sy = sin(yaw);

            Eigen::MatrixXd rot6d(TWIST_SIZE, TWIST_SIZE);
            rot6d.setIdentity();
            rot6d(0, 0) = cp * cy;
            rot6d(0, 1) = cy * sr * sp - cr * sy;
            rot6d(0, 2) = cr * cy * sp + sr * sy;
            rot6d(1, 0) = cp * sy;
            rot6d(1, 1) = cr * cy + sr * sp * sy;
            rot6d(1, 2) = -cy * sr + cr * sp * sy;
            rot6d(2, 0) = -sp;
            rot6d(2, 1) = cp * sr;
            rot6d(2, 2) = cr * cp;

            // Rotate the covariance
            covarianceRotated = rot6d * covarianceRotated * rot6d;
          }

        }
        catch (tf::TransformException &ex)
        {
          ROS_ERROR_STREAM("Odometry pose transform error: " << ex.what());
          transformSuccessful = false;
        }

        // Transforming from a frame id to itself can fail when the tf tree isn't
        // being broadcast (e.g., for some bag files). Handle this situation.
        std::string msgFrame = (tfPrefix_.empty() ? twistTmp.frame_id_ : "/" + tfPrefix_ + "/" + twistTmp.frame_id_);

        if(!transformSuccessful && targetFrame == msgFrame)
        {
          twistCorrected = twistTmp;

          transformSuccessful = true;
        }

        if (!transformSuccessful)
        {
          ROS_WARN_STREAM("WARNING: Transform " << twistTmp.frame_id_ << "->" << targetFrame << " was unsuccessful.");

          if (filter.getDebug())
          {
            debugStream_ << "Transform from " << twistTmp.frame_id_ << "->" << targetFrame << " failed. Ignoring twist measurement.";
          }

          return false;
        }
        else
        {
          if (filter.getDebug())
          {
            debugStream_ << "Corrected odometry linear velocity is :\n";
            debugStream_ << "(" << twistCorrected.getOrigin().x() << ", " << twistCorrected.getOrigin().y() << ", " << twistCorrected.getOrigin().z() << ")\n";
          }

          // Now that we've got a corrected twist, zero out the values we don't want
          tf::Vector3 finalLinearVelocity = twistCorrected.getOrigin();
          finalLinearVelocity.setX(updateVector[StateMemberVx] ? finalLinearVelocity.getX() : 0.0);
          finalLinearVelocity.setY(updateVector[StateMemberVy] ? finalLinearVelocity.getY() : 0.0);
          finalLinearVelocity.setZ(updateVector[StateMemberVz] ? finalLinearVelocity.getZ() : 0.0);
          twistCorrected.setOrigin(finalLinearVelocity);

          tf::Quaternion finalAngularVelocity = twistCorrected.getRotation();
          double rollVel, pitchVel, yawVel;
          quatToRPY(finalAngularVelocity, rollVel, pitchVel, yawVel);
          rollVel = (updateVector[StateMemberVroll] ? rollVel : 0.0);
          pitchVel = (updateVector[StateMemberVpitch] ? pitchVel : 0.0);
          yawVel = (updateVector[StateMemberVyaw] ? yawVel : 0.0);

          // Copy everything into the measurement vector. Zero out what we don't want.
          measurement(StateMemberVx) = twistCorrected.getOrigin().x();
          measurement(StateMemberVy) = twistCorrected.getOrigin().y();
          measurement(StateMemberVz) = twistCorrected.getOrigin().z();

          // Copy everything into the measurement vector. Zero out what we don't want.
          measurement(StateMemberVroll) = rollVel;
          measurement(StateMemberVpitch) = pitchVel;
          measurement(StateMemberVyaw) = yawVel;

          if (filter.getDebug())
          {
            debugStream_ << "Corrected angular velocity is :\n";
            debugStream_ << "(" << rollVel << ", " << pitchVel << ", " << yawVel << ")\n";
          }

          // Copy the covariances
          measurementCovariance.block(POSITION_V_OFFSET, POSITION_V_OFFSET, TWIST_SIZE, TWIST_SIZE) = covarianceRotated.block(0, 0, TWIST_SIZE, TWIST_SIZE);
        }

        if (filter.getDebug())
        {
          debugStream_ << "\n----- /EkfNavigation::prepareTwist (" << topicName << ") ------\n";
        }

        return true;
      }

      //! @brief Utility method for converting quaternion to RPY
      //! @param[in] quat - The quaternion to convert
      //! @param[in] roll - The converted roll
      //! @param[in] pitch - The converted pitch
      //! @param[in] yaw - The converted yaw
      //!
      static void quatToRPY(const tf::Quaternion &quat, double &roll, double &pitch, double &yaw)
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

      //! @brief tf frame name for the robot's odometry frame
      //!
      std::string odomFrameName_;

      //! @brief If we're getting messages rapidly, we may accidentally get
      //! an older message arriving after a newer one. This variable keeps
      //! track of the most recent message time for each subscribed message
      //! topic. We also use it when listening to odometry messages to
      //! determine if we should be using messages from that topic.
      //!
      std::map<std::string, ros::Time> lastMessageTimes_;

      //! @brief Used for differential updates. When we first start, one can
      //! envision scenarios wherein a robot's raw odometry assumes we
      //! started at an (x, y, z, roll, pitch, yaw) of (0, 0, 0, 0, 0, 0).
      //! However, the initial IMU measurement will almost certainly
      //! disagree. There may also be situations where one odometry message
      //! assumes an initial pose that differs from another. This variable
      //! stores the transforms that will remove the initial measurement's
      //! value from all future measurements, before feeding those measurements
      //! into the filter. Note that this is only used for pose data, and not
      //! velocity.
      //!
      std::map<std::string, tf::Transform> initialPoses_;

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

      //! @brief Transform listener for managing coordinate transforms
      //!
      tf::TransformListener tfListener_;

      //! @brief Used for outputting debug messages
      //!
      std::ofstream debugStream_;

  };
}

#endif
