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

#include "robot_localization/ros_filter.h"
#include "robot_localization/filter_utilities.h"
#include "robot_localization/ekf.h"
#include "robot_localization/ukf.h"

namespace RobotLocalization
{
  template<typename T>
  RosFilter<T>::RosFilter(std::vector<double> args) :
      nhLocal_("~"),
      filter_(args),
      frequency_(30.0),
      printDiagnostics_(false),
      twoDMode_(false)
  {
    stateVariableNames_.push_back("X");
    stateVariableNames_.push_back("Y");
    stateVariableNames_.push_back("Z");
    stateVariableNames_.push_back("ROLL");
    stateVariableNames_.push_back("PITCH");
    stateVariableNames_.push_back("YAW");
    stateVariableNames_.push_back("X_VELOCITY");
    stateVariableNames_.push_back("Y_VELOCITY");
    stateVariableNames_.push_back("Z_VELOCITY");
    stateVariableNames_.push_back("ROLL_VELOCITY");
    stateVariableNames_.push_back("PITCH_VELOCITY");
    stateVariableNames_.push_back("YAW_VELOCITY");
    stateVariableNames_.push_back("X_ACCELERATION");
    stateVariableNames_.push_back("Y_ACCELERATION");
    stateVariableNames_.push_back("Z_ACCELERATION");
  }

  template<typename T>
  RosFilter<T>::~RosFilter()
  {
    poseMessageFilters_.clear();
    twistMessageFilters_.clear();
    accelerationMessageFilters_.clear();
    poseTopicSubs_.clear();
    twistTopicSubs_.clear();
    imuTopicSubs_.clear();
    odomTopicSubs_.clear();
  }

  template<typename T>
  void RosFilter<T>::accelerationCallback(const sensor_msgs::Imu::ConstPtr &msg,
                                       const std::string &topicName,
                                       const std::string &targetFrame,
                                       const std::vector<int> &updateVector,
                                       const double mahalanobisThresh)
  {
    RF_DEBUG("------ RosFilter::accelerationCallback (" << topicName << ") ------\n"
             "Twist message:\n" << *msg);

    if (lastMessageTimes_.count(topicName) == 0)
    {
      lastMessageTimes_.insert(std::pair<std::string, ros::Time>(topicName, msg->header.stamp));
    }

    // Make sure this message is newer than the last one
    if (msg->header.stamp >= lastMessageTimes_[topicName])
    {
      RF_DEBUG("Update vector for " << topicName << " is:\n" << updateVector);

      Eigen::VectorXd measurement(STATE_SIZE);
      Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);

      measurement.setZero();
      measurementCovariance.setZero();

      // Make sure we're actually updating at least one of these variables
      if (updateVector[StateMemberAx] || updateVector[StateMemberAy] || updateVector[StateMemberAz])
      {
        std::vector<int> updateVectorCorrected = updateVector;

        // Prepare the twist data for inclusion in the filter
        if (prepareAcceleration(msg, topicName, targetFrame, updateVectorCorrected, measurement, measurementCovariance))
        {
          // Store the measurement. Add an "acceleration" suffix so we know what kind of measurement
          // we're dealing with when we debug the core filter logic.
          enqueueMeasurement(topicName, measurement, measurementCovariance, updateVectorCorrected, mahalanobisThresh, msg->header.stamp);

          RF_DEBUG("Enqueued new measurement for " << topicName << "_acceleration\n");
        }
        else
        {
          RF_DEBUG("Did *not* enqueue measurement for " << topicName << "_acceleration\n");
        }
      }
      else
      {
        RF_DEBUG("Update vector for " << topicName << " is such that none of its state variables will be updated\n");
      }

      lastMessageTimes_[topicName] = msg->header.stamp;

      RF_DEBUG("Last message time for " << topicName << " is now " << lastMessageTimes_[topicName] << "\n");
    }
    else
    {
      RF_DEBUG("Message is too old. Last message time for " << topicName <<
               " is " << lastMessageTimes_[topicName] << ", current message time is " <<
               msg->header.stamp << ".\n");
    }

    RF_DEBUG("\n----- /RosFilter::accelerationCallback (" << topicName << ") ------\n");
  }

  template<typename T>
  void RosFilter<T>::enqueueMeasurement(const std::string &topicName,
                                     const Eigen::VectorXd &measurement,
                                     const Eigen::MatrixXd &measurementCovariance,
                                     const std::vector<int> &updateVector,
                                     const double mahalanobisThresh,
                                     const ros::Time &time)
  {
    Measurement meas;

    meas.topicName_ = topicName;
    meas.measurement_ = measurement;
    meas.covariance_ = measurementCovariance;
    meas.updateVector_ = updateVector;
    meas.time_ = time.toSec();
    meas.mahalanobisThresh_ = mahalanobisThresh;
    measurementQueue_.push(meas);
  }

  template<typename T>
  void RosFilter<T>::forceTwoD(Eigen::VectorXd &measurement,
                               Eigen::MatrixXd &measurementCovariance,
                               std::vector<int> &updateVector)
  {
    measurement(StateMemberZ) = 0.0;
    measurement(StateMemberRoll) = 0.0;
    measurement(StateMemberPitch) = 0.0;
    measurement(StateMemberVz) = 0.0;
    measurement(StateMemberVroll) = 0.0;
    measurement(StateMemberVpitch) = 0.0;
    measurement(StateMemberAz) = 0.0;

    measurementCovariance(StateMemberZ, StateMemberZ) = 1e-6;
    measurementCovariance(StateMemberRoll, StateMemberRoll) = 1e-6;
    measurementCovariance(StateMemberPitch, StateMemberPitch) = 1e-6;
    measurementCovariance(StateMemberVz, StateMemberVz) = 1e-6;
    measurementCovariance(StateMemberVroll, StateMemberVroll) = 1e-6;
    measurementCovariance(StateMemberVpitch, StateMemberVpitch) = 1e-6;
    measurementCovariance(StateMemberAz, StateMemberAz) = 1e-6;

    updateVector[StateMemberZ] = 1;
    updateVector[StateMemberRoll] = 1;
    updateVector[StateMemberPitch] = 1;
    updateVector[StateMemberVz] = 1;
    updateVector[StateMemberVroll] = 1;
    updateVector[StateMemberVpitch] = 1;
    updateVector[StateMemberAz] = 1;
  }

  template<typename T>
  bool RosFilter<T>::getFilteredOdometryMessage(nav_msgs::Odometry &message)
  {
    // If the filter has received a measurement at some point...
    if (filter_.getInitializedStatus())
    {
      // Grab our current state and covariance estimates
      const Eigen::VectorXd &state = filter_.getState();
      const Eigen::MatrixXd &estimateErrorCovariance = filter_.getEstimateErrorCovariance();

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

  template<typename T>
  void RosFilter<T>::imuCallback(const sensor_msgs::Imu::ConstPtr &msg,
                              const std::string &topicName)
  {
    RF_DEBUG("------ RosFilter::imuCallback (" << topicName << ") ------\n" <<
             "IMU message:\n" << *msg);

    // As with the odometry message, we can separate out the pose- and twist-related variables
    // in the IMU message and pass them to the pose and twist callbacks (filters)

    std::string imuPoseTopicName = topicName + "_pose";
    if(poseMessageFilters_.count(imuPoseTopicName) > 0)
    {
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
      poseMessageFilters_[imuPoseTopicName]->add(pptr);
    }

    std::string imuTwistTopicName = topicName + "_twist";
    if(twistMessageFilters_.count(imuTwistTopicName) > 0)
    {
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
      twistMessageFilters_[imuTwistTopicName]->add(tptr);
    }

    std::string imuAccelTopicName = topicName + "_acceleration";
    if(accelerationMessageFilters_.count(imuAccelTopicName) > 0)
    {
      // We still need to handle the acceleration data, but we don't
      // actually have a good container message for it, so just pass
      // the IMU message on through a message filter.
      accelerationMessageFilters_[imuAccelTopicName]->add(msg);
    }

    RF_DEBUG("\n----- /RosFilter::imuCallback (" << topicName << ") ------\n");
  }

  template<typename T>
  void RosFilter<T>::integrateMeasurements(const double currentTime)
  {
    RF_DEBUG("------ RosFilter::integrateMeasurements ------\n\n"
             "Integration time is " << std::setprecision(20) << currentTime << "\n"
             << measurementQueue_.size() << " measurements in queue.\n");

    // If we have any measurements in the queue, process them
    if (!measurementQueue_.empty())
    {
      while (!measurementQueue_.empty())
      {
        Measurement measurement = measurementQueue_.top();
        measurementQueue_.pop();

        // This will call predict and, if necessary, correct
        filter_.processMeasurement(measurement);
      }

      filter_.setLastUpdateTime(currentTime);
    }
    else if (filter_.getInitializedStatus())
    {
      // In the event that we don't get any measurements for a long time,
      // we still need to continue to estimate our state. Therefore, we
      // should project the state forward here.
      double lastUpdateDelta = currentTime - filter_.getLastUpdateTime();

      // If we get a large delta, then continuously predict until
      if(lastUpdateDelta >= filter_.getSensorTimeout())
      {
        RF_DEBUG("Sensor timeout! Last update time was " << filter_.getLastUpdateTime() <<
                 ", current time is " << currentTime <<
                 ", delta is " << lastUpdateDelta << "\n");

        filter_.validateDelta(lastUpdateDelta);
        filter_.predict(lastUpdateDelta);

        // Update the last measurement time and last update time
        filter_.setLastMeasurementTime(filter_.getLastMeasurementTime() + lastUpdateDelta);
        filter_.setLastUpdateTime(filter_.getLastUpdateTime() + lastUpdateDelta);
      }
    }
    else
    {
      RF_DEBUG("Filter not yet initialized.\n");
    }

    RF_DEBUG("\n----- /RosFilter::integrateMeasurements ------\n");
  }

  template<typename T>
  void RosFilter<T>::loadParams()
  {
    // For diagnostic purposes, collect information about how many different
    // sources are measuring each absolute pose variable and do not have
    // differential integration enabled.
    std::map<StateMembers, int> absPoseVarCounts;
    absPoseVarCounts[StateMemberX] = 0;
    absPoseVarCounts[StateMemberY] = 0;
    absPoseVarCounts[StateMemberZ] = 0;
    absPoseVarCounts[StateMemberRoll] = 0;
    absPoseVarCounts[StateMemberPitch] = 0;
    absPoseVarCounts[StateMemberYaw] = 0;

    // Determine if we'll be printing diagnostic information
    nhLocal_.param("print_diagnostics", printDiagnostics_, false);

    // Grab the debug param. If true, the node will produce a LOT of output.
    bool debug;
    nhLocal_.param("debug", debug, false);

    if (debug)
    {
      std::string debugOutFile;

      try
      {
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
      catch(const std::exception &e)
      {
        ROS_WARN_STREAM("RosFilter::loadParams() - unable to create debug output file" << debugOutFile
                        << ". Error was " << e.what() << "\n");
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

    /*
     * These parameters are designed to enforce compliance with REP-105:
     * http://www.ros.org/reps/rep-0105.html
     * When fusing absolute position data from sensors such as GPS, the state
     * estimate can undergo discrete jumps. According to REP-105, we want three
     * coordinate frames: map, odom, and base_link. The map frame can have
     * discontinuities, but is the frame with the most accurate position estimate
     * for the robot and should not suffer from drift. The odom frame drifts over
     * time, but is guaranteed to be continuous and is accurate enough for local
     * planning and navigation. The base_link frame is affixed to the robot. The
     * intention is that some odometry source broadcasts the odom->base_link
     * transform. The localization software should broadcast map->base_link.
     * However, tf does not allow multiple parents for a coordinate frame, so
     * we must *compute* map->base_link, but then use the existing odom->base_link
     * transform to compute *and broadcast* map->odom.
     *
     * The state estimation nodes in robot_localization therefore have two "modes."
     * If your world_frame parameter value matches the odom_frame parameter value,
     * then robot_localization will assume someone else is broadcasting a transform
     * from odom_frame->base_link_frame, and it will compute the
     * map_frame->odom_frame transform. Otherwise, it will simply compute the
     * odom_frame->base_link_frame transform.
     *
     * The default is the latter behavior (broadcast of odom->base_link).
     * */
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

    // Transform future dating
    double offsetTmp;
    nhLocal_.param("transform_time_offset", offsetTmp, 0.0);
    tfTimeOffset_.fromSec(offsetTmp);

    // Update frequency and sensor timeout
    double sensorTimeout;
    nhLocal_.param("frequency", frequency_, 30.0);
    nhLocal_.param("sensor_timeout", sensorTimeout, 1.0 / frequency_);
    filter_.setSensorTimeout(sensorTimeout);

    // Determine if we're in 2D mode
    nhLocal_.param("two_d_mode", twoDMode_, false);

    // Debugging writes to file
    RF_DEBUG("tf_prefix is " << tfPrefix_ <<
             "\nmap_frame is " << mapFrameId_ <<
             "\nodom_frame is " << odomFrameId_ <<
             "\nbase_link_frame is " << baseLinkFrameId_ <<
             "\nworld_frame is " << worldFrameId_ <<
             "\nfrequency is " << frequency_ <<
             "\nsensor_timeout is " << filter_.getSensorTimeout() <<
             "\ntwo_d_mode is " << (twoDMode_ ? "true" : "false") <<
             "\nprint_diagnostics is " << (printDiagnostics_ ? "true" : "false") << "\n");

    // Create a subscriber for manually setting/resetting pose
    setPoseSub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("set_pose", 1, &RosFilter<T>::setPoseCallback, this);

    // Create a service for manually setting/resetting pose
    setPoseSrv_ = nh_.advertiseService("set_pose", &RosFilter<T>::setPoseSrvCallback, this);

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

        // Determine if we want to integrate this sensor relatively
        bool relative;
        nhLocal_.param(odomTopicName + std::string("_relative"), relative, false);

        std::string odomTopic;
        nhLocal_.getParam(odomTopicName, odomTopic);

        // Check for pose rejection threshold
        double poseMahalanobisThresh;
        nhLocal_.param(odomTopicName + std::string("_pose_rejection_threshold"), poseMahalanobisThresh, std::numeric_limits<double>::max());

        // Check for twist rejection threshold
        double twistMahalanobisThresh;
        nhLocal_.param(odomTopicName + std::string("_twist_rejection_threshold"), twistMahalanobisThresh, std::numeric_limits<double>::max());

        // Now pull in its boolean update vector configuration. Create separate vectors for pose
        // and twist data, and then zero out the opposite values in each vector (no pose data in
        // the twist update vector and vice-versa).
        std::vector<int> updateVec = loadUpdateConfig(odomTopicName);
        std::vector<int> poseUpdateVec = updateVec;
        std::fill(poseUpdateVec.begin() + POSITION_V_OFFSET, poseUpdateVec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
        std::vector<int> twistUpdateVec = updateVec;
        std::fill(twistUpdateVec.begin() + POSITION_OFFSET, twistUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE, 0);

        int poseUpdateSum = std::accumulate(poseUpdateVec.begin(), poseUpdateVec.end(), 0);
        int twistUpdateSum = std::accumulate(twistUpdateVec.begin(), twistUpdateVec.end(), 0);

        // Store the odometry topic subscribers so they dont go out of scope. Also,
        // odometry data has both pose and twist data, each with their own frame_id.
        // The odometry data gets broken up and passed into callbacks for pose and
        // twist data, so we need to create message filters for them, and then
        // manually add the pose and twist messages after we extract them from the
        // odometry message.

        int odomQueueSize = 1;
        nhLocal_.param(odomTopicName + "_queue_size", odomQueueSize, 1);

        if(poseUpdateSum + twistUpdateSum > 0)
        {
          odomTopicSubs_.push_back(
                nh_.subscribe<nav_msgs::Odometry>(odomTopic, odomQueueSize,
                                                  boost::bind(&RosFilter<T>::odometryCallback, this, _1, odomTopicName)));
        }
        else
        {
          RL_DIAGNOSTIC(odomTopic << " is listed as an input topic, but all update variables are false");
        }

        if(poseUpdateSum > 0)
        {
          poseMFPtr poseFilPtr(new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(tfListener_, worldFrameId_, odomQueueSize));
          std::string odomPoseTopicName = odomTopicName + "_pose";
          poseFilPtr->registerCallback(boost::bind(&RosFilter<T>::poseCallback, this, _1, odomPoseTopicName, worldFrameId_, poseUpdateVec, differential, relative, false, poseMahalanobisThresh));
          poseFilPtr->registerFailureCallback(boost::bind(&RosFilter<T>::transformPoseFailureCallback, this, _1, _2, odomTopicName, worldFrameId_));
          poseMessageFilters_[odomPoseTopicName] = poseFilPtr;

          if(!differential)
          {
            absPoseVarCounts[StateMemberX] += poseUpdateVec[StateMemberX];
            absPoseVarCounts[StateMemberY] += poseUpdateVec[StateMemberY];
            absPoseVarCounts[StateMemberZ] += poseUpdateVec[StateMemberZ];
            absPoseVarCounts[StateMemberRoll] += poseUpdateVec[StateMemberRoll];
            absPoseVarCounts[StateMemberPitch] += poseUpdateVec[StateMemberPitch];
            absPoseVarCounts[StateMemberYaw] += poseUpdateVec[StateMemberYaw];
          }
        }

        if(twistUpdateSum > 0)
        {
          twistMFPtr twistFilPtr(new tf::MessageFilter<geometry_msgs::TwistWithCovarianceStamped>(tfListener_, baseLinkFrameId_, odomQueueSize));
          std::string odomTwistTopicName = odomTopicName + "_twist";
          twistFilPtr->registerCallback(boost::bind(&RosFilter<T>::twistCallback, this, _1, odomTwistTopicName, baseLinkFrameId_, twistUpdateVec, twistMahalanobisThresh));
          twistFilPtr->registerFailureCallback(boost::bind(&RosFilter<T>::transformTwistFailureCallback, this, _1, _2, odomTopicName, baseLinkFrameId_));
          twistMessageFilters_[odomTwistTopicName] = twistFilPtr;
        }

        RF_DEBUG("Subscribed to " << odomTopic << " (" << odomTopicName << ")\n\t" <<
                 odomTopicName << "_differential is " << (differential ? "true" : "false") << "\n\t" <<
                 odomTopicName << "_pose_rejection_threshold is " << poseMahalanobisThresh << "\n\t" <<
                 odomTopicName << "_twist_rejection_threshold is " << twistMahalanobisThresh << "\n\t" <<
                 odomTopicName << "_queue_size is " << odomQueueSize << "\n\t" <<
                 odomTopicName << " pose update vector is " << poseUpdateVec << "\t"<<
                 odomTopicName << " twist update vector is " << twistUpdateVec);
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

        // Determine if we want to integrate this sensor relatively
        bool relative;
        nhLocal_.param(poseTopicName + std::string("_relative"), relative, false);

        std::string poseTopic;
        nhLocal_.getParam(poseTopicName, poseTopic);

        // Check for pose rejection threshold
        double poseMahalanobisThresh;
        nhLocal_.param(poseTopicName + std::string("_rejection_threshold"), poseMahalanobisThresh, std::numeric_limits<double>::max());

        int poseQueueSize = 1;
        nhLocal_.param(poseTopicName + "_queue_size", poseQueueSize, 1);

        // Pull in the sensor's config, zero out values that are invalid for the pose type
        std::vector<int> poseUpdateVec = loadUpdateConfig(poseTopicName);
        std::fill(poseUpdateVec.begin() + POSITION_V_OFFSET, poseUpdateVec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
        std::fill(poseUpdateVec.begin() + POSITION_A_OFFSET, poseUpdateVec.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE, 0);

        int poseUpdateSum = std::accumulate(poseUpdateVec.begin(), poseUpdateVec.end(), 0);

        if(poseUpdateSum > 0)
        {
          // Create and store message filter subscriber objects and message filters
          poseMFSubPtr subPtr(new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>());
          subPtr->subscribe(nh_, poseTopic, poseQueueSize);
          poseMFPtr filPtr(new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*subPtr, tfListener_, worldFrameId_, poseQueueSize));
          filPtr->registerCallback(boost::bind(&RosFilter<T>::poseCallback, this, _1, poseTopicName, worldFrameId_, poseUpdateVec, differential, relative, false, poseMahalanobisThresh));
          filPtr->registerFailureCallback(boost::bind(&RosFilter<T>::transformPoseFailureCallback, this, _1, _2, poseTopicName, worldFrameId_));
          poseTopicSubs_.push_back(subPtr);
          poseMessageFilters_[poseTopicName] = filPtr;

          if(!differential)
          {
            absPoseVarCounts[StateMemberX] += poseUpdateVec[StateMemberX];
            absPoseVarCounts[StateMemberY] += poseUpdateVec[StateMemberY];
            absPoseVarCounts[StateMemberZ] += poseUpdateVec[StateMemberZ];
            absPoseVarCounts[StateMemberRoll] += poseUpdateVec[StateMemberRoll];
            absPoseVarCounts[StateMemberPitch] += poseUpdateVec[StateMemberPitch];
            absPoseVarCounts[StateMemberYaw] += poseUpdateVec[StateMemberYaw];
          }
        }
        else
        {
          ROS_WARN_STREAM("Warning: " << poseTopic << " is listed as an input topic, but all pose update variables are false");
        }

        RF_DEBUG("Subscribed to " << poseTopic << " (" << poseTopicName << ")\n\t" <<
                 poseTopicName << "_differential is " << (differential ? "true" : "false") << "\n\t" <<
                 poseTopicName << "_rejection_threshold is " << poseMahalanobisThresh << "\n\t" <<
                 poseTopicName << "_queue_size is " << poseQueueSize << "\n\t" <<
                 poseTopicName << " update vector is " << poseUpdateVec);
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

        // Check for twist rejection threshold
        double twistMahalanobisThresh;
        nhLocal_.param(twistTopicName + std::string("_rejection_threshold"), twistMahalanobisThresh, std::numeric_limits<double>::max());

        int twistQueueSize = 1;
        nhLocal_.param(twistTopicName + "_queue_size", twistQueueSize, 1);

        // Pull in the sensor's config, zero out values that are invalid for the twist type
        std::vector<int> twistUpdateVec = loadUpdateConfig(twistTopicName);
        std::fill(twistUpdateVec.begin() + POSITION_OFFSET, twistUpdateVec.begin() + POSITION_OFFSET + POSE_SIZE, 0);

        int twistUpdateSum = std::accumulate(twistUpdateVec.begin(), twistUpdateVec.end(), 0);

        if(twistUpdateSum > 0)
        {
          // Create and store subscriptions and message filters
          twistMFSubPtr subPtr(new message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped>());
          subPtr->subscribe(nh_, twistTopic, twistQueueSize);
          twistMFPtr filPtr(new tf::MessageFilter<geometry_msgs::TwistWithCovarianceStamped>(*subPtr, tfListener_, baseLinkFrameId_, twistQueueSize));
          filPtr->registerCallback(boost::bind(&RosFilter<T>::twistCallback, this, _1, twistTopicName, baseLinkFrameId_, twistUpdateVec, twistMahalanobisThresh));
          filPtr->registerFailureCallback(boost::bind(&RosFilter<T>::transformTwistFailureCallback, this, _1, _2, twistTopicName, baseLinkFrameId_));
          twistTopicSubs_.push_back(subPtr);
          twistMessageFilters_[twistTopicName] = filPtr;
        }
        else
        {
          ROS_WARN_STREAM("Warning: " << twistTopic << " is listed as an input topic, but all twist update variables are false");
        }

        RF_DEBUG("Subscribed to " << twistTopic << " (" << twistTopicName << ")\n\t" <<
                 twistTopicName << "_rejection_threshold is " << twistMahalanobisThresh << "\n\t" <<
                 twistTopicName << "_queue_size is " << twistQueueSize << "\n\t" <<
                 twistTopicName << " update vector is " << twistUpdateVec);
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

        // Determine if we want to integrate this sensor relatively
        bool relative;
        nhLocal_.param(imuTopicName + std::string("_relative"), relative, false);

        std::string imuTopic;
        nhLocal_.getParam(imuTopicName, imuTopic);

        // Check for pose rejection threshold
        double poseMahalanobisThresh;
        nhLocal_.param(imuTopicName + std::string("_pose_rejection_threshold"), poseMahalanobisThresh, std::numeric_limits<double>::max());

        // Check for angular velocity rejection threshold. Handle deprecated name.
        double twistMahalanobisThresh;
        std::string correctImuTwistRejectionName = imuTopicName + std::string("_twist_rejection_threshold");
        std::string deprecatedImuTwistRejectionName = imuTopicName + std::string("_angular_velocity_rejection_threshold");
        if(nhLocal_.hasParam(correctImuTwistRejectionName))
        {
          nhLocal_.param(correctImuTwistRejectionName, twistMahalanobisThresh, std::numeric_limits<double>::max());
        }
        else if(nhLocal_.hasParam(deprecatedImuTwistRejectionName))
        {
          nhLocal_.param(deprecatedImuTwistRejectionName, twistMahalanobisThresh, std::numeric_limits<double>::max());

          ROS_WARN_STREAM("Detected deprecated parameter " << deprecatedImuTwistRejectionName << ". Please use " <<
                          correctImuTwistRejectionName << " intstead.");
        }
        else
        {
          twistMahalanobisThresh = std::numeric_limits<double>::max();
        }

        // Check for acceleration rejection threshold
        double accelMahalanobisThresh;
        nhLocal_.param(imuTopicName + std::string("_linear_acceleration_rejection_threshold"), accelMahalanobisThresh, std::numeric_limits<double>::max());

        bool removeGravAcc = false;
        if(!nhLocal_.getParam(imuTopicName + "_remove_gravitational_acceleration", removeGravAcc))
        {
          // Handle deprecated method
          nhLocal_.param("remove_gravitational_acceleration", removeGravAcc, false);

          ROS_WARN_STREAM("Detected deprecated parameter remove_gravitational_acceleration. Please specify this " <<
                          "parameter for each IMU, e.g., " << imuTopicName + "_remove_gravitational_acceleration");
        }
        removeGravitationalAcc_[imuTopicName + "_acceleration"] = removeGravAcc;

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

        int poseUpdateSum = std::accumulate(poseUpdateVec.begin(), poseUpdateVec.end(), 0);
        int twistUpdateSum = std::accumulate(twistUpdateVec.begin(), twistUpdateVec.end(), 0);
        int accelUpdateSum = std::accumulate(accelUpdateVec.begin(), accelUpdateVec.end(), 0);

        int imuQueueSize = 1;
        nhLocal_.param(imuTopicName + "_queue_size", imuQueueSize, 1);

        if(poseUpdateSum + twistUpdateSum + accelUpdateSum > 0)
        {
          // Create and store subscriptions and message filters as with odometry data
          imuTopicSubs_.push_back(
                nh_.subscribe<sensor_msgs::Imu>(imuTopic, imuQueueSize,
                                                boost::bind(&RosFilter<T>::imuCallback, this, _1, imuTopicName)));
        }
        else
        {
          ROS_WARN_STREAM("Warning: " << imuTopic << " is listed as an input topic, but all its update variables are false");
        }

        if(poseUpdateSum > 0)
        {
          // @todo: There's a lot of ambiguity with IMU frames. Should allow a parameter that specifies a target IMU frame.
          poseMFPtr poseFilPtr(new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(tfListener_, baseLinkFrameId_, imuQueueSize));
          std::string imuPoseTopicName = imuTopicName + "_pose";
          poseFilPtr->registerCallback(boost::bind(&RosFilter<T>::poseCallback, this, _1, imuPoseTopicName, baseLinkFrameId_, poseUpdateVec, differential, relative, true, poseMahalanobisThresh));
          poseFilPtr->registerFailureCallback(boost::bind(&RosFilter<T>::transformPoseFailureCallback, this, _1, _2, imuTopicName, baseLinkFrameId_));
          poseMessageFilters_[imuPoseTopicName] = poseFilPtr;

          if(!differential)
          {
            absPoseVarCounts[StateMemberRoll] += poseUpdateVec[StateMemberRoll];
            absPoseVarCounts[StateMemberPitch] += poseUpdateVec[StateMemberPitch];
            absPoseVarCounts[StateMemberYaw] += poseUpdateVec[StateMemberYaw];
          }
        }

        if(twistUpdateSum > 0)
        {
          twistMFPtr twistFilPtr(new tf::MessageFilter<geometry_msgs::TwistWithCovarianceStamped>(tfListener_, baseLinkFrameId_, imuQueueSize));
          std::string imuTwistTopicName = imuTopicName + "_twist";
          twistFilPtr->registerCallback(boost::bind(&RosFilter<T>::twistCallback, this, _1, imuTwistTopicName, baseLinkFrameId_, twistUpdateVec, twistMahalanobisThresh));
          twistFilPtr->registerFailureCallback(boost::bind(&RosFilter<T>::transformTwistFailureCallback, this, _1, _2, imuTopicName, baseLinkFrameId_));
          twistMessageFilters_[imuTwistTopicName] = twistFilPtr;
        }

        if(accelUpdateSum > 0)
        {
          imuMFPtr accelFilPtr(new tf::MessageFilter<sensor_msgs::Imu>(tfListener_, baseLinkFrameId_, imuQueueSize));
          std::string imuAccelTopicName = imuTopicName + "_acceleration";
          accelFilPtr->registerCallback(boost::bind(&RosFilter<T>::accelerationCallback, this, _1, imuAccelTopicName, baseLinkFrameId_, accelUpdateVec, accelMahalanobisThresh));
          accelFilPtr->registerFailureCallback(boost::bind(&RosFilter<T>::transformImuFailureCallback, this, _1, _2, imuTopicName, baseLinkFrameId_));
          accelerationMessageFilters_[imuAccelTopicName] = accelFilPtr;
        }

        RF_DEBUG("Subscribed to " << imuTopic << " (" << imuTopicName << ")\n\t" <<
                 imuTopicName << "_differential is " << (differential ? "true" : "false") << "\n\t" <<
                 imuTopicName << "_pose_rejection_threshold is " << poseMahalanobisThresh << "\n\t" <<
                 imuTopicName << "_twist_rejection_threshold is " << twistMahalanobisThresh << "\n\t" <<
                 imuTopicName << "_linear_acceleration_rejection_threshold is " << accelMahalanobisThresh << "\n\t" <<
                 imuTopicName << "_remove_gravitational_acceleration is " << (removeGravAcc ? "true" : "false") << "\n\t" <<
                 imuTopicName << "_queue_size is " << imuQueueSize << "\n\t" <<
                 imuTopicName << " pose update vector is " << poseUpdateVec << "\t"<<
                 imuTopicName << " twist update vector is " << twistUpdateVec << "\t" <<
                 imuTopicName << " acceleration update vector is " << accelUpdateVec);
      }
    } while (moreParams);

    // Warn users about multiple non-differential input sources
    if(printDiagnostics_)
    {
      for(int stateVar = StateMemberX; stateVar <= StateMemberYaw; ++stateVar)
      {
        if(absPoseVarCounts[static_cast<StateMembers>(stateVar)] > 1)
        {
          RL_DIAGNOSTIC(absPoseVarCounts[static_cast<StateMembers>(stateVar - POSITION_OFFSET)] <<
                        " absolute pose inputs detected for " <<
                        stateVariableNames_[stateVar] << ". This may result in oscillations.");
        }
      }
    }

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
            try
            {
              // These matrices can cause problems if all the types
              // aren't specified with decimal points. Handle that
              // using string streams.
              std::ostringstream ostr;
              ostr << processNoiseCovarConfig[matSize * i + j];
              std::istringstream istr(ostr.str());
              istr >> processNoiseCovariance(i, j);
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

        RF_DEBUG("Process noise covariance is:\n" << processNoiseCovariance << "\n");
      }
      catch (XmlRpc::XmlRpcException &e)
      {
        ROS_ERROR_STREAM(
          "ERROR reading sensor config: " << e.getMessage() << " for process_noise_covariance (type: " << processNoiseCovarConfig.getType() << ")");
      }

      filter_.setProcessNoiseCovariance(processNoiseCovariance);
    }

    // Load up the process noise covariance (from the launch file/parameter server)
    Eigen::MatrixXd initialEstimateErrorCovariance(STATE_SIZE, STATE_SIZE);
    initialEstimateErrorCovariance.setZero();
    XmlRpc::XmlRpcValue estimateErrorCovarConfig;

    if (nhLocal_.hasParam("initial_estimate_covariance"))
    {
      try
      {
        nhLocal_.getParam("initial_estimate_covariance", estimateErrorCovarConfig);

        ROS_ASSERT(estimateErrorCovarConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);

        int matSize = initialEstimateErrorCovariance.rows();

        for (int i = 0; i < matSize; i++)
        {
          for (int j = 0; j < matSize; j++)
          {
            try
            {
              // These matrices can cause problems if all the types
              // aren't specified with decimal points. Handle that
              // using string streams.
              std::ostringstream ostr;
              ostr << estimateErrorCovarConfig[matSize * i + j];
              std::istringstream istr(ostr.str());
              istr >> initialEstimateErrorCovariance(i, j);
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

        RF_DEBUG("Initial estimate error covariance is:\n" << initialEstimateErrorCovariance << "\n");
      }
      catch (XmlRpc::XmlRpcException &e)
      {
        ROS_ERROR_STREAM(
          "ERROR reading initial_estimate_covariance (type: " << estimateErrorCovarConfig.getType() << "): " << e.getMessage());
      }
      catch(...)
      {
        ROS_ERROR_STREAM(
          "ERROR reading initial_estimate_covariance (type: " << estimateErrorCovarConfig.getType() << ")");
      }

      filter_.setEstimateErrorCovariance(initialEstimateErrorCovariance);
    }
  }

  template<typename T>
  void RosFilter<T>::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg,
                                   const std::string &topicName)
  {
    RF_DEBUG("------ RosFilter::odometryCallback (" << topicName << ") ------\n" <<
             "Odometry message:\n" << *msg);

    std::string odomPoseTopicName = topicName + "_pose";
    if(poseMessageFilters_.count(odomPoseTopicName) > 0)
    {
      // Grab the pose portion of the message and pass it to the poseCallback
      geometry_msgs::PoseWithCovarianceStamped *posPtr = new geometry_msgs::PoseWithCovarianceStamped();
      posPtr->header = msg->header;
      posPtr->pose = msg->pose; // Entire pose object, also copies covariance

      geometry_msgs::PoseWithCovarianceStampedConstPtr pptr(posPtr);
      poseMessageFilters_[odomPoseTopicName]->add(pptr);
    }

    std::string odomTwistTopicName = topicName + "_twist";
    if(twistMessageFilters_.count(odomTwistTopicName) > 0)
    {
      // Grab the twist portion of the message and pass it to the twistCallback
      geometry_msgs::TwistWithCovarianceStamped *twistPtr = new geometry_msgs::TwistWithCovarianceStamped();
      twistPtr->header = msg->header;
      twistPtr->header.frame_id = msg->child_frame_id;
      twistPtr->twist = msg->twist; // Entire twist object, also copies covariance


      geometry_msgs::TwistWithCovarianceStampedConstPtr tptr(twistPtr);
      twistMessageFilters_[odomTwistTopicName]->add(tptr);
    }

    RF_DEBUG("\n----- /RosFilter::odometryCallback (" << topicName << ") ------\n");
  }

  template<typename T>
  void RosFilter<T>::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                               const std::string &topicName,
                               const std::string &targetFrame,
                               const std::vector<int> &updateVector,
                               const bool differential,
                               const bool relative,
                               const bool imuData,
                               const double mahalanobisThresh)
  {
    RF_DEBUG("------ RosFilter::poseCallback (" << topicName << ") ------\n" <<
             "Pose message:\n" << *msg);

    // Put the initial value in the lastMessagTimes_ for this variable if it's empty
    if (lastMessageTimes_.count(topicName) == 0)
    {
      lastMessageTimes_.insert(std::pair<std::string, ros::Time>(topicName, msg->header.stamp));
    }

    // Make sure this message is newer than the last one
    if (msg->header.stamp >= lastMessageTimes_[topicName])
    {
      RF_DEBUG("Update vector for " << topicName << " is:\n" << updateVector);

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
        if (preparePose(msg, topicName, targetFrame, differential, relative, imuData, updateVectorCorrected, measurement, measurementCovariance))
        {
          // Store the measurement. Add a "pose" suffix so we know what kind of measurement
          // we're dealing with when we debug the core filter logic.
          enqueueMeasurement(topicName, measurement, measurementCovariance, updateVectorCorrected, mahalanobisThresh, msg->header.stamp);

          RF_DEBUG("Enqueued new measurement for " << topicName << "\n");
        }
        else
        {
          RF_DEBUG("Did *not* enqueue measurement for " << topicName << "\n");
        }
      }
      else
      {
        RF_DEBUG("Update vector for " << topicName << " is such that none of its state variables will be updated\n");
      }

      lastMessageTimes_[topicName] = msg->header.stamp;

      RF_DEBUG("Last message time for " << topicName << " is now " << lastMessageTimes_[topicName] << "\n");
    }
    else
    {
      RF_DEBUG("Message is too old. Last message time for " << topicName << " is "
               << lastMessageTimes_[topicName] << ", current message time is "
               << msg->header.stamp << ".\n");
    }

    RF_DEBUG("\n----- /RosFilter::poseCallback (" << topicName << ") ------\n");
  }

  template<typename T>
  void RosFilter<T>::run()
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

    while (ros::ok())
    {
      // Get latest state and publish it
      nav_msgs::Odometry filteredPosition;

      if (getFilteredOdometryMessage(filteredPosition))
      {
        worldBaseLinkTransMsg_.header.stamp = filteredPosition.header.stamp + tfTimeOffset_;
        worldBaseLinkTransMsg_.header.frame_id = filteredPosition.header.frame_id;
        worldBaseLinkTransMsg_.child_frame_id = filteredPosition.child_frame_id;

        worldBaseLinkTransMsg_.transform.translation.x = filteredPosition.pose.pose.position.x;
        worldBaseLinkTransMsg_.transform.translation.y = filteredPosition.pose.pose.position.y;
        worldBaseLinkTransMsg_.transform.translation.z = filteredPosition.pose.pose.position.z;
        worldBaseLinkTransMsg_.transform.rotation = filteredPosition.pose.pose.orientation;

        // If the worldFrameId_ is the odomFrameId_ frame, then we can just send the transform. If the
        // worldFrameId_ is the mapFrameId_ frame, we'll have some work to do.
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

            /*
             * First, see these two references:
             * http://wiki.ros.org/tf/Overview/Using%20Published%20Transforms#lookupTransform
             * http://wiki.ros.org/geometry/CoordinateFrameConventions#Transform_Direction
             * We have a transform from mapFrameId_->baseLinkFrameId_, but it would actually transform
             * a given pose from baseLinkFrameId_->mapFrameId_. We then used lookupTransform, whose
             * first two arguments are target frame and source frame, to get a transform from
             * baseLinkFrameId_->odomFrameId_. However, this transform would actually transform data
             * from odomFrameId_->baseLinkFrameId_. Now imagine that we have a position in the
             * mapFrameId_ frame. First, we multiply it by the inverse of the
             * mapFrameId_->baseLinkFrameId, which will transform that data from mapFrameId_ to
             * baseLinkFrameId_. Now we want to go from baseLinkFrameId_->odomFrameId_, but the
             * transform we have takes data from odomFrameId_->baseLinkFrameId_, so we need its
             * inverse as well. We have now transformed our data from mapFrameId_ to odomFrameId_.
             * However, if we want other users to be able to do the same, we need to broadcast
             * the inverse of that entire transform.
            */

            mapOdomTrans.mult(worldBaseLinkTrans, odomBaseLinkTrans);

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
      integrateMeasurements(ros::Time::now().toSec());

      if(!loop_rate.sleep())
      {
        ROS_WARN_STREAM("Failed to meet update rate! Try decreasing the rate, limiting sensor output frequency, or limiting the number of sensors.");
      }
    }
  }

  template<typename T>
  void RosFilter<T>::setPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
  {
    RF_DEBUG("------ RosFilter::setPoseCallback ------\nPose message:\n" << *msg);

    std::string topicName("setPose");

    // Get rid of any initial poses (pretend we've never had a measurement)
    initialMeasurements_.clear();
    previousMeasurements_.clear();
    previousMeasurementCovariances_.clear();

    // Clear out the measurement queue, so that we don't immediately undo our
    // reset.
    while(!measurementQueue_.empty())
    {
      measurementQueue_.pop();
    }

    // Set the state vector to the reported pose
    Eigen::VectorXd measurement(STATE_SIZE);
    Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);
    std::vector<int> updateVector(STATE_SIZE, true);

    // We only measure pose variables, so initialize the vector to 0
    measurement.setZero();

    // Set this to the identity and let the message reset it
    measurementCovariance.setIdentity();
    measurementCovariance *= 1e-6;

    // Prepare the pose data (really just using this to transform it into the target frame).
    // Twist data is going to get zeroed out.
    preparePose(msg, topicName, odomFrameId_, false, false, false, updateVector, measurement, measurementCovariance);

    // For the state
    filter_.setState(measurement);
    filter_.setEstimateErrorCovariance(measurementCovariance);

    filter_.setLastMeasurementTime(ros::Time::now().toSec());
    filter_.setLastUpdateTime(ros::Time::now().toSec());

    RF_DEBUG("\n------ /RosFilter::setPoseCallback ------\n");
  }

  template<typename T>
  bool RosFilter<T>::setPoseSrvCallback(robot_localization::SetPose::Request& request,
                          robot_localization::SetPose::Response&)
  {
    geometry_msgs::PoseWithCovarianceStamped::Ptr msg;
    msg = boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>(request.pose);
    setPoseCallback(msg);

    return true;
  }

  template<typename T>
  std::string RosFilter<T>::tfFailureReasonString(const tf::FilterFailureReason reason)
  {
    std::string retVal;

    switch(reason)
    {
      case tf::filter_failure_reasons::OutTheBack:
        retVal = std::string("The timestamp on the message is more than the cache length earlier than the newest data in the transform cache");
        break;
      case tf::filter_failure_reasons::EmptyFrameID:
        retVal = std::string("The message frame_id is empty");
        break;
      case tf::filter_failure_reasons::Unknown:
      default:
        retVal = std::string("No transform exists from source to target frame");
        break;
    }

    return retVal;
  }

  template<typename T>
  void RosFilter<T>::transformImuFailureCallback(const sensor_msgs::Imu::ConstPtr &msg,
                                              const tf::FilterFailureReason reason,
                                              const std::string &topicName,
                                              const std::string &targetFrame)
  {
    std::stringstream stream;
    std::string warning;

    stream << "WARNING: failed to transform from " << msg->header.frame_id <<
              "->" << targetFrame << " for " << topicName << " message received at " <<
              msg->header.stamp << ". " << tfFailureReasonString(reason) << ".\n";
    warning = stream.str();

    ROS_WARN_STREAM_THROTTLE(2.0, warning);
    RF_DEBUG(warning);
  }

  template<typename T>
  void RosFilter<T>::transformPoseFailureCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                                               const tf::FilterFailureReason reason,
                                               const std::string &topicName,
                                               const std::string &targetFrame)
  {
    std::stringstream stream;
    std::string warning;

    stream << "WARNING: failed to transform from " << msg->header.frame_id <<
              "->" << targetFrame << " for " << topicName << " message received at " <<
              msg->header.stamp << ". " << tfFailureReasonString(reason) << ".\n";
    warning = stream.str();

    ROS_WARN_STREAM_THROTTLE(2.0, warning);
    RF_DEBUG(warning);
  }

  template<typename T>
  void RosFilter<T>::transformTwistFailureCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg,
                                                const tf::FilterFailureReason reason,
                                                const std::string &topicName,
                                                const std::string &targetFrame)
  {
    std::stringstream stream;
    std::string warning;

    stream << "WARNING: failed to transform from " << msg->header.frame_id <<
              "->" << targetFrame << " for " << topicName << " message received at " <<
              msg->header.stamp << ". " << tfFailureReasonString(reason) << ".\n";
    warning = stream.str();

    ROS_WARN_STREAM_THROTTLE(2.0, warning);
    RF_DEBUG(warning);
  }

  template<typename T>
  void RosFilter<T>::twistCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg,
                                const std::string &topicName,
                                const std::string &targetFrame,
                                const std::vector<int> &updateVector,
                                const double mahalanobisThresh)
  {
    RF_DEBUG("------ RosFilter::twistCallback (" << topicName << ") ------\n"
             "Twist message:\n" << *msg);

    if (lastMessageTimes_.count(topicName) == 0)
    {
      lastMessageTimes_.insert(std::pair<std::string, ros::Time>(topicName, msg->header.stamp));
    }

    // Make sure this message is newer than the last one
    if (msg->header.stamp >= lastMessageTimes_[topicName])
    {
      RF_DEBUG("Update vector for " << topicName << " is:\n" << updateVector);

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
        if (prepareTwist(msg, topicName, targetFrame, updateVectorCorrected, measurement, measurementCovariance))
        {
          // Store the measurement. Add a "twist" suffix so we know what kind of measurement
          // we're dealing with when we debug the core filter logic.
          enqueueMeasurement(topicName, measurement, measurementCovariance, updateVectorCorrected, mahalanobisThresh, msg->header.stamp);

          RF_DEBUG("Enqueued new measurement for " << topicName << "_twist\n");
        }
        else
        {
          RF_DEBUG("Did *not* enqueue measurement for " << topicName << "_twist\n");
        }
      }
      else
      {
        RF_DEBUG("Update vector for " << topicName << " is such that none of its state variables will be updated\n");
      }

      lastMessageTimes_[topicName] = msg->header.stamp;

      RF_DEBUG("Last message time for " << topicName << " is now " << lastMessageTimes_[topicName] << "\n");
    }
    else
    {
      RF_DEBUG("Message is too old. Last message time for " << topicName <<
               " is " << lastMessageTimes_[topicName] << ", current message time is " <<
               msg->header.stamp << ".\n");
    }

    RF_DEBUG("\n----- /RosFilter::twistCallback (" << topicName << ") ------\n");
  }

  template<typename T>
  void RosFilter<T>::copyCovariance(const double *arr,
                                    Eigen::MatrixXd &covariance,
                                    const std::string &topicName,
                                    const std::vector<int> &updateVector,
                                    const size_t offset,
                                    const size_t dimension)
  {
    for (size_t i = 0; i < dimension; i++)
    {
      for (size_t j = 0; j < dimension; j++)
      {
        covariance(i, j) = arr[dimension * i + j];

        if(printDiagnostics_)
        {
          std::string iVar = stateVariableNames_[offset + i];

          if(covariance(i, j) > 1e3 && (updateVector[offset  + i] || updateVector[offset  + j]))
          {
            std::string jVar = stateVariableNames_[offset + j];

            RL_DIAGNOSTIC("For topic " << topicName << ", the covariance at position (" << dimension * i + j <<
                          "), which corresponds to " << (i == j ? iVar + " variance" : iVar + " and " + jVar + " covariance") <<
                          ", the value is extremely large (" << covariance(i, j) << "), but the update vector for " <<
                          (i == j ? iVar : iVar + " and/or " + jVar) << " is set to true. This may produce undesirable results.");
          }
          else if(updateVector[i] && i == j && covariance(i, j) == 0)
          {
            RL_DIAGNOSTIC("For topic " << topicName << ", the covariance at position (" << dimension * i + j <<
                          "), which corresponds to " << iVar << " variance, was zero. This will be replaced with "
                          "a small value to maintain filter stability, but should be corrected at the message origin node.");
          }
          else if(updateVector[i] && i == j && covariance(i, j) < 0)
          {
            RL_DIAGNOSTIC("For topic " << topicName << ", the covariance at position (" << dimension * i + j <<
                          "), which corresponds to " << iVar << " variance, was negative. This will be replaced with "
                          "a small positive value to maintain filter stability, but should be corrected at the message origin node.");
          }
        }
      }
    }
  }

  template<typename T>
  void RosFilter<T>::copyCovariance(const Eigen::MatrixXd &covariance,
                                 double *arr,
                                 const size_t dimension)
  {
    for (size_t i = 0; i < dimension; i++)
    {
      for (size_t j = 0; j < dimension; j++)
      {
        arr[dimension * i + j] = covariance(i, j);
      }
    }
  }

  template<typename T>
  std::vector<int> RosFilter<T>::loadUpdateConfig(const std::string &topicName)
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
      ROS_FATAL_STREAM("Could not read sensor update configuration for topic " << topicName <<
                       " (type: " << topicConfig.getType() << ", expected: " << XmlRpc::XmlRpcValue::TypeArray
                       << "). Error was " << e.getMessage() << "\n");
    }

    return updateVector;
  }

  template<typename T>
  bool RosFilter<T>::lookupTransformSafe(const std::string &targetFrame,
                                         const std::string &sourceFrame,
                                         const ros::Time &time,
                                         tf::StampedTransform &targetFrameTrans)
  {
    bool retVal = true;

    // First try to transform the data at the requested time
    try
    {
      tfListener_.lookupTransform(targetFrame, sourceFrame, time, targetFrameTrans);
    }
    catch (tf::TransformException &ex)
    {
      RF_DEBUG("WARNING: Could not obtain transform from " << sourceFrame <<
               " to " << targetFrame << ". Error was " << ex.what());

      // The issue might be that the transforms that are available are not close
      // enough temporally to be used. In that case, just use the latest available
      // transform and warn the user.
      try
      {
        tfListener_.lookupTransform(targetFrame, sourceFrame, ros::Time(0), targetFrameTrans);

        ROS_WARN_STREAM_THROTTLE(2.0, "Transform from " << sourceFrame << " to " << targetFrame <<
                                      " was unavailable for the time requested. Using latest instead.\n");
      }
      catch(tf::TransformException &ex)
      {
        ROS_WARN_STREAM_THROTTLE(2.0, "Could not obtain transform from " << sourceFrame <<
                                      " to " << targetFrame << ". Error was " << ex.what() << "\n");

        retVal = false;
      }
    }

    // Transforming from a frame id to itself can fail when the tf tree isn't
    // being broadcast (e.g., for some bag files). This is the only failure that
    // would throw an exception, so check for this situation before giving up.
    if(!retVal)
    {
      if(targetFrame == sourceFrame)
      {
        targetFrameTrans.setIdentity();
        retVal = true;
      }
    }

    return retVal;
  }

  template<typename T>
  bool RosFilter<T>::prepareAcceleration(const sensor_msgs::Imu::ConstPtr &msg,
                           const std::string &topicName,
                           const std::string &targetFrame,
                           std::vector<int> &updateVector,
                           Eigen::VectorXd &measurement,
                           Eigen::MatrixXd &measurementCovariance)
  {
    RF_DEBUG("------ RosFilter::prepareAcceleration (" << topicName << ") ------\n");

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
    tf::Matrix3x3 maskAcc(updateVector[StateMemberAx], 0, 0,
                          0, updateVector[StateMemberAy], 0,
                          0, 0, updateVector[StateMemberAz]);

    // 3. We'll need to rotate the covariance as well
    Eigen::MatrixXd covarianceRotated(ACCELERATION_SIZE, ACCELERATION_SIZE);
    covarianceRotated.setZero();

    this->copyCovariance(&(msg->linear_acceleration_covariance[0]), covarianceRotated, topicName, updateVector, POSITION_A_OFFSET, ACCELERATION_SIZE);

    RF_DEBUG("Original measurement as tf object: " << accTmp <<
             "\nOriginal update vector:\n" << updateVector <<
             "\nOriginal covariance matrix:\n" << covarianceRotated << "\n");

    // 4. We need to transform this into the target frame (probably base_link)
    // It's unlikely that we'll get a velocity measurement in another frame, but
    // we have to handle the situation.
    tf::StampedTransform targetFrameTrans;
    bool canTransform = lookupTransformSafe(targetFrame, msgFrame, msg->header.stamp, targetFrameTrans);

    if(canTransform)
    {
      // We don't know if the user has already handled the removal
      // of normal forces, so we use a parameter
      if(removeGravitationalAcc_[topicName])
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

        RF_DEBUG("Orientation is " << curAttitude <<
                 "Acceleration due to gravity is " << rotNorm <<
                 "After removing acceleration due to gravity, acceleration is " << accTmp << "\n");
      }

      // Transform to correct frame
      // @todo: This needs to take into account offsets from the origin. Right now,
      // it assumes that if the sensor is placed at some non-zero offset from the
      // vehicle's center, that the vehicle turns with constant velocity. This needs
      // to be something like accTmp = targetFrameTrans.getBasis() * accTmp - targetFrameTrans.getOrigin().cross(rotation_acceleration);
      // We can get rotational acceleration by differentiating the rotational velocity (if it's available)
      accTmp = targetFrameTrans.getBasis() * accTmp;
      maskAcc = targetFrameTrans.getBasis() * maskAcc;

      // Now use the mask values to determinme which update vector values should true
      updateVector[StateMemberAx] = static_cast<int>(maskAcc.getRow(StateMemberAx - POSITION_A_OFFSET).length() >= 1e-6);
      updateVector[StateMemberAy] = static_cast<int>(maskAcc.getRow(StateMemberAy - POSITION_A_OFFSET).length() >= 1e-6);
      updateVector[StateMemberAz] = static_cast<int>(maskAcc.getRow(StateMemberAz - POSITION_A_OFFSET).length() >= 1e-6);

      RF_DEBUG(msg->header.frame_id << "->" << targetFrame << " transform:\n" << targetFrameTrans <<
               "\nAfter applying transform to " << targetFrame << ", update vector is:\n" << updateVector <<
               "\nAfter applying transform to " << targetFrame << ", measurement is:\n" << accTmp << "\n");

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

      RF_DEBUG("Transformed covariance is \n" << covarianceRotated << "\n");

      // 6. Store our corrected measurement and covariance
      measurement(StateMemberAx) = accTmp.getX();
      measurement(StateMemberAy) = accTmp.getY();
      measurement(StateMemberAz) = accTmp.getZ();

      // Copy the covariances
      measurementCovariance.block(POSITION_A_OFFSET, POSITION_A_OFFSET, ACCELERATION_SIZE, ACCELERATION_SIZE) = covarianceRotated.block(0, 0, ACCELERATION_SIZE, ACCELERATION_SIZE);

      // 7. If we're in 2D mode, handle that.
      if(twoDMode_)
      {
        forceTwoD(measurement, measurementCovariance, updateVector);
      }
    }
    else
    {
      RF_DEBUG("Could not transform measurement into " << targetFrame << ". Ignoring...\n");
    }

    RF_DEBUG("\n----- /RosFilter::prepareAcceleration(" << topicName << ") ------\n");

    return canTransform;
  }

  template<typename T>
  bool RosFilter<T>::preparePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                              const std::string &topicName,
                              const std::string &targetFrame,
                              const bool differential,
                              const bool relative,
                              const bool imuData,
                              std::vector<int> &updateVector,
                              Eigen::VectorXd &measurement,
                              Eigen::MatrixXd &measurementCovariance)
  {
    bool retVal = false;

    RF_DEBUG("------ RosFilter::preparePose (" << topicName << ") ------\n");

    // 1. Get the measurement into a tf-friendly transform (pose) object
    tf::Stamped<tf::Pose> poseTmp;

    // This is mostly in support of IMU data: the IMU message has only a single
    // frame_id, but reports its data in two separate frames. We get around this
    // by making the targetFrame of the orientation data blank, which implies
    // that the data is actually specified in our world_frame. This will likely
    // not strictly be true, but as long as the IMU data is reported in a world-
    // fixed ENU frame, then users can just turn on the differential setting.
    std::string finalTargetFrame;
    if(targetFrame == "" && msg->header.frame_id == "")
    {
      // Blank target and message frames mean we can just
      // use our world_frame
      finalTargetFrame = worldFrameId_;
    }
    else if(targetFrame == "")
    {
      // A blank target frame means we shouldn't bother
      // transforming the data
      finalTargetFrame = msg->header.frame_id;
    }
    else
    {
      // Otherwise, we should use our target frame
      finalTargetFrame = targetFrame;
    }

    RF_DEBUG("Final target frame for " << topicName << " is " << finalTargetFrame << "\n");

    poseTmp.frame_id_ = msg->header.frame_id;
    poseTmp.stamp_ = msg->header.stamp;

    // Fill out the position data
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
        RL_DIAGNOSTIC("The " << topicName << " message contains an invalid orientation quaternion, " <<
                      "but its configuration is such that orientation data is being used. Correcting...");
      }
    }
    else
    {
      tf::quaternionMsgToTF(msg->pose.pose.orientation, orientation);
    }

    poseTmp.setRotation(orientation);

    // We'll need this later for storing this measurement for differential integration
    tf::Transform curMeasurement;

    // 2. If this sensor is relative, remove the initial measurement for it
    if(relative)
    {
      if(initialMeasurements_.count(topicName) == 0)
      {
        initialMeasurements_.insert(std::pair<std::string, tf::Transform>(topicName, poseTmp));
      }

      tf::Pose initialMeasurement = initialMeasurements_[topicName];
      poseTmp.setData(initialMeasurement.inverseTimes(poseTmp));
    }

    // 3. Get the target frame transformation
    tf::StampedTransform targetFrameTrans;
    bool canTransform = lookupTransformSafe(finalTargetFrame, poseTmp.frame_id_, poseTmp.stamp_, targetFrameTrans);

    // 4. robot_localization lets users configure which variables from the sensor should be
    //    fused with the filter. This is specified at the sensor level. However, the data
    //    may go through transforms before being fused with the state estimate. In that case,
    //    we need to know which of the transformed variables came from the pre-transformed
    //    "approved" variables (i.e., the ones that had "true" in their xxx_config parameter).
    //    To do this, we construct matrices using the update vector values on the diagonals,
    //    pass this matrix through the rotation, and use the length of each row to determine
    //    the transformed update vector.
    tf::Matrix3x3 maskPosition(updateVector[StateMemberX], 0, 0,
                               0, updateVector[StateMemberY], 0,
                               0, 0, updateVector[StateMemberZ]);

    tf::Matrix3x3 maskOrientation(updateVector[StateMemberRoll], 0, 0,
                                  0, updateVector[StateMemberPitch], 0,
                                  0, 0, updateVector[StateMemberYaw]);

    maskPosition = targetFrameTrans.getBasis() * maskPosition;
    maskOrientation = targetFrameTrans.getBasis() * maskOrientation;

    // Now copy the mask values back into the update vector
    updateVector[StateMemberX] = static_cast<int>(maskPosition.getRow(StateMemberX - POSITION_OFFSET).length() >= 1e-6);
    updateVector[StateMemberY] = static_cast<int>(maskPosition.getRow(StateMemberY - POSITION_OFFSET).length() >= 1e-6);
    updateVector[StateMemberZ] = static_cast<int>(maskPosition.getRow(StateMemberZ - POSITION_OFFSET).length() >= 1e-6);
    updateVector[StateMemberRoll] = static_cast<int>(maskOrientation.getRow(StateMemberRoll - ORIENTATION_OFFSET).length() >= 1e-6);
    updateVector[StateMemberPitch] = static_cast<int>(maskOrientation.getRow(StateMemberPitch - ORIENTATION_OFFSET).length() >= 1e-6);
    updateVector[StateMemberYaw] = static_cast<int>(maskOrientation.getRow(StateMemberYaw - ORIENTATION_OFFSET).length() >= 1e-6);

    // 5. We'll need to rotate the covariance as well. Create a container and
    // copy over the covariance data
    Eigen::MatrixXd covariance(POSE_SIZE, POSE_SIZE);
    covariance.setZero();
    copyCovariance(&(msg->pose.covariance[0]), covariance, topicName, updateVector, POSITION_OFFSET, POSE_SIZE);

    RF_DEBUG(msg->header.frame_id << "->" << finalTargetFrame << " transform:\n" << targetFrameTrans <<
             "\nAfter applying transform to " << finalTargetFrame << ", update vector is:\n" << updateVector <<
             "\nOriginal measurement as tf object:\n" << poseTmp <<
             "\nOriginal covariance matrix:\n" << covariance << "\n");

    // Now rotate the covariance: create an augmented matrix that
    // contains a 3D rotation matrix in the upper-left and lower-right
    // quadrants, with zeros elsewhere.
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

    // Now carry out the rotation
    Eigen::MatrixXd covarianceRotated = rot6d * covariance * rot6d.transpose();

    RF_DEBUG("After rotating into the " << finalTargetFrame << " frame, covariance is \n" << covarianceRotated << "\n");

    // Regardless of whether or not we are using differential mode, we'll need
    // to transform the data into a target frame.
    if(canTransform)
    {
      // Two cases: if we're in differential mode, we need to generate a twist
      // message. Otherwise, we just transform it to the target frame.
      if(differential)
      {
        bool success = false;

        // We're going to be playing with poseTmp, so store it,
        // as we'll need to save its current value for the next
        // measurement.
        curMeasurement = poseTmp;

        // Make sure we have previous measurements to work with
        if(previousMeasurements_.count(topicName) > 0 && previousMeasurementCovariances_.count(topicName) > 0)
        {
          // 6a. If we are carrying out differential integration and
          // we have a previous measurement for this sensor,then we
          // need to apply the inverse of that measurement to this new
          // measurement to produce a "delta" measurement between the two.
          // Even if we're not using all of the variables from this sensor,
          // we need to use the whole measurement to determine the delta
          // to the new measurement
          tf::Pose prevMeasurement = previousMeasurements_[topicName];
          poseTmp.setData(prevMeasurement.inverseTimes(poseTmp));

          RF_DEBUG("Previous measurement:\n" << previousMeasurements_[topicName] <<
                   "\nAfter removing previous measurement, measurement delta is:\n" << poseTmp << "\n");

          // 6b. Now we we have a measurement delta in the frame_id of the
          // message, but we want that delta to be in the target frame, so
          // we need to apply the rotation of the target frame transform.
          targetFrameTrans.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
          poseTmp.mult(targetFrameTrans, poseTmp);

          RF_DEBUG("After rotating to the target frame, measurement delta is:\n" << poseTmp << "\n");

          // 6c. Now use the time difference from the last message to compute
          // translational and rotational velocities
          double dt = msg->header.stamp.toSec() - lastMessageTimes_[topicName].toSec();
          double xVel = poseTmp.getOrigin().getX() / dt;
          double yVel = poseTmp.getOrigin().getY() / dt;
          double zVel = poseTmp.getOrigin().getZ() / dt;

          double rollVel = 0;
          double pitchVel = 0;
          double yawVel = 0;

          RosFilterUtilities::quatToRPY(poseTmp.getRotation(), rollVel, pitchVel, yawVel);
          rollVel /= dt;
          pitchVel /= dt;
          yawVel /= dt;

          // 6d. Fill out the velocity data in the message
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

          // 6e. Now rotate the previous covariance for this measurement to get it
          // into the target frame, and add the current measurement's rotated covariance
          // to the previous measurement's rotated covariance, and multiply by the time delta.
          Eigen::MatrixXd prevCovarRotated = rot6d * previousMeasurementCovariances_[topicName] * rot6d.transpose();
          covarianceRotated = (covarianceRotated.eval() + prevCovarRotated) * dt;
          copyCovariance(covarianceRotated, &(twistPtr->twist.covariance[0]), POSE_SIZE);

          RF_DEBUG("Previous measurement covariance:\n" << previousMeasurementCovariances_[topicName] <<
                   "\nPrevious measurement covariance rotated:\n" << prevCovarRotated <<
                   "\nFinal twist covariance:\n" << covarianceRotated << "\n");

          // Now pass this on to prepareTwist, which will convert it to the required frame
          success = prepareTwist(ptr, topicName + "_twist", twistPtr->header.frame_id, updateVector, measurement, measurementCovariance);
        }

        // 6f. Update the previous measurement and measurement covariance
        previousMeasurements_[topicName] = curMeasurement;
        previousMeasurementCovariances_[topicName] = covariance;

        retVal = success;
      }
      else
      {
        // 6. If we're not handling this differentially, then transform it to the
        // target frame. However, the means of transforming this data varies
        // with the input type. If it's an IMU, we need to do some special processing.

        if(imuData)
        {
          // 6a. For IMU data, we have to first remove all static sensor offsets (note that
          // here we assume that the transform in question is the transform that defines how
          // the sensor is mounted on the vehicle; this is not meant for handling NED->ENU
          // transformations). For now, robot_localization assumes all orientation data is in the
          // ENU frame.

          // First, convert the transform and measurement rotation to RPY
          // @todo: There must be a way to handle this with quaternions. Need to look into it.
          double rollOffset = 0;
          double pitchOffset = 0;
          double yawOffset = 0;
          double roll = 0;
          double pitch = 0;
          double yaw = 0;
          RosFilterUtilities::quatToRPY(targetFrameTrans.getRotation(), rollOffset, pitchOffset, yawOffset);
          RosFilterUtilities::quatToRPY(poseTmp.getRotation(), roll, pitch, yaw);

          // 6b. Apply the offset (making sure to bound them), and throw them in a vector
          tf::Vector3 rpyAngles(FilterUtilities::clampRotation(roll - rollOffset),
                                FilterUtilities::clampRotation(pitch - pitchOffset),
                                FilterUtilities::clampRotation(yaw - yawOffset));

          // 6c. Now we need to rotate the roll and pitch by the yaw offset value.
          // Imagine a case where an IMU is mounted facing sideways. In that case
          // pitch for the IMU's world frame is roll for the robot.
          tf::Matrix3x3 mat;
          mat.setRPY(0.0, 0.0, yawOffset);
          rpyAngles = mat * rpyAngles;
          poseTmp.getBasis().setRPY(rpyAngles.getX(), rpyAngles.getY(), rpyAngles.getZ());
        }
        else
        {
          // 6d. Apply the target frame transformation to the pose object. If
          poseTmp.mult(targetFrameTrans, poseTmp);
        }

        poseTmp.frame_id_ = finalTargetFrame;

        // 7. Finally, copy everything into our measurement and covariance objects
        measurement(StateMemberX) = poseTmp.getOrigin().x();
        measurement(StateMemberY) = poseTmp.getOrigin().y();
        measurement(StateMemberZ) = poseTmp.getOrigin().z();

        // The filter needs roll, pitch, and yaw values instead of quaternions
        double roll, pitch, yaw;
        RosFilterUtilities::quatToRPY(poseTmp.getRotation(), roll, pitch, yaw);
        measurement(StateMemberRoll) = roll;
        measurement(StateMemberPitch) = pitch;
        measurement(StateMemberYaw) = yaw;

        measurementCovariance.block(0, 0, POSE_SIZE, POSE_SIZE) = covarianceRotated.block(0, 0, POSE_SIZE, POSE_SIZE);

        // 8. If we're in 2D mode, handle that.
        if(twoDMode_)
        {
          forceTwoD(measurement, measurementCovariance, updateVector);
        }

        retVal = true;
      }
    }
    else
    {
      retVal = false;

      RF_DEBUG("Could not transform measurement into " << finalTargetFrame << ". Ignoring...");
    }

    RF_DEBUG("\n----- /RosFilter::preparePose (" << topicName << ") ------\n");

    return retVal;
  }

  template<typename T>
  bool RosFilter<T>::prepareTwist(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg,
                               const std::string &topicName,
                               const std::string &targetFrame,
                               std::vector<int> &updateVector,
                               Eigen::VectorXd &measurement,
                               Eigen::MatrixXd &measurementCovariance)
  {
    RF_DEBUG("------ RosFilter::prepareTwist (" << topicName << ") ------\n");

    // 1. Get the measurement into two separate vector objects.
    tf::Vector3 twistLin(msg->twist.twist.linear.x,
                         msg->twist.twist.linear.y,
                         msg->twist.twist.linear.z);
    tf::Vector3 measTwistRot(msg->twist.twist.angular.x,
                             msg->twist.twist.angular.y,
                             msg->twist.twist.angular.z);

    // 1a. This sensor may or may not measure rotational velocity. Regardless,
    // if it measures linear velocity, then later on, we'll need to remove "false"
    // linear velocity resulting from angular velocity and the translational offset
    // of the sensor from the vehicle origin.
    const Eigen::VectorXd &state = filter_.getState();
    tf::Vector3 stateTwistRot(state(StateMemberVroll),
                              state(StateMemberVpitch),
                              state(StateMemberVyaw));

    // Determine the frame_id of the data
    std::string msgFrame = (msg->header.frame_id == "" ? targetFrame : msg->header.frame_id);

    // 2. robot_localization lets users configure which variables from the sensor should be
    //    fused with the filter. This is specified at the sensor level. However, the data
    //    may go through transforms before being fused with the state estimate. In that case,
    //    we need to know which of the transformed variables came from the pre-transformed
    //    "approved" variables (i.e., the ones that had "true" in their xxx_config parameter).
    //    To do this, we construct matrices using the update vector values on the diagonals,
    //    pass this matrix through the rotation, and use the length of each row to determine
    //    the transformed update vector.
    tf::Matrix3x3 maskLin(updateVector[StateMemberVx], 0, 0,
                          0, updateVector[StateMemberVy], 0,
                          0, 0, updateVector[StateMemberVz]);

    tf::Matrix3x3 maskRot(updateVector[StateMemberVroll], 0, 0,
                          0, updateVector[StateMemberVpitch], 0,
                          0, 0, updateVector[StateMemberVyaw]);

    // 3. We'll need to rotate the covariance as well
    Eigen::MatrixXd covarianceRotated(TWIST_SIZE, TWIST_SIZE);
    covarianceRotated.setZero();

    copyCovariance(&(msg->twist.covariance[0]), covarianceRotated, topicName, updateVector, POSITION_V_OFFSET, TWIST_SIZE);

    RF_DEBUG("Original measurement as tf object:\nLinear: " << twistLin <<
             "Rotational: " << measTwistRot <<
             "\nOriginal update vector:\n" << updateVector <<
             "\nOriginal covariance matrix:\n" << covarianceRotated << "\n");

    // 4. We need to transform this into the target frame (probably base_link)
    tf::StampedTransform targetFrameTrans;
    bool canTransform = lookupTransformSafe(targetFrame, msgFrame, msg->header.stamp, targetFrameTrans);

    if(canTransform)
    {
      // Transform to correct frame. Note that we can get linear velocity
      // as a result of the sensor offset and rotational velocity
      measTwistRot = targetFrameTrans.getBasis() * measTwistRot;
      twistLin = targetFrameTrans.getBasis() * twistLin + targetFrameTrans.getOrigin().cross(stateTwistRot);
      maskLin = targetFrameTrans.getBasis() * maskLin;
      maskRot = targetFrameTrans.getBasis() * maskRot;

      // Now copy the mask values back into the update vector
      updateVector[StateMemberVx] = static_cast<int>(maskLin.getRow(StateMemberVx - POSITION_V_OFFSET).length() >= 1e-6);
      updateVector[StateMemberVy] = static_cast<int>(maskLin.getRow(StateMemberVy - POSITION_V_OFFSET).length() >= 1e-6);
      updateVector[StateMemberVz] = static_cast<int>(maskLin.getRow(StateMemberVz - POSITION_V_OFFSET).length() >= 1e-6);
      updateVector[StateMemberVroll] = static_cast<int>(maskRot.getRow(StateMemberVroll - ORIENTATION_V_OFFSET).length() >= 1e-6);
      updateVector[StateMemberVpitch] = static_cast<int>(maskRot.getRow(StateMemberVpitch - ORIENTATION_V_OFFSET).length() >= 1e-6);
      updateVector[StateMemberVyaw] = static_cast<int>(maskRot.getRow(StateMemberVyaw - ORIENTATION_V_OFFSET).length() >= 1e-6);

      RF_DEBUG(msg->header.frame_id << "->" << targetFrame << " transform:\n" << targetFrameTrans <<
               "\nAfter applying transform to " << targetFrame << ", update vector is:\n" << updateVector <<
               "\nAfter applying transform to " << targetFrame << ", measurement is:\n" <<
               "Linear: " << twistLin << "Rotational: " << measTwistRot << "\n");

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

      RF_DEBUG("Transformed covariance is \n" << covarianceRotated << "\n");

      // 6. Store our corrected measurement and covariance
      measurement(StateMemberVx) = twistLin.getX();
      measurement(StateMemberVy) = twistLin.getY();
      measurement(StateMemberVz) = twistLin.getZ();
      measurement(StateMemberVroll) = measTwistRot.getX();
      measurement(StateMemberVpitch) = measTwistRot.getY();
      measurement(StateMemberVyaw) = measTwistRot.getZ();

      // Copy the covariances
      measurementCovariance.block(POSITION_V_OFFSET, POSITION_V_OFFSET, TWIST_SIZE, TWIST_SIZE) = covarianceRotated.block(0, 0, TWIST_SIZE, TWIST_SIZE);

      // 7. If in 2D mode, handle that.
      if(twoDMode_)
      {
        forceTwoD(measurement, measurementCovariance, updateVector);
      }
    }
    else
    {
      RF_DEBUG("Could not transform measurement into " << targetFrame << ". Ignoring...");
    }

    RF_DEBUG("\n----- /RosFilter::prepareTwist (" << topicName << ") ------\n");

    return canTransform;
  }
}

// Instantiations of classes is required when template class code
// is placed in a .cpp file.
template class RobotLocalization::RosFilter<RobotLocalization::Ekf>;
template class RobotLocalization::RosFilter<RobotLocalization::Ukf>;

