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

#ifndef RobotLocalization_RosFilter_h
#define RobotLocalization_RosFilter_h

#include "robot_localization/ros_filter_utilities.h"
#include "robot_localization/filter_common.h"
#include "robot_localization/filter_base.h"
#include "robot_localization/SetPose.h"

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

#include <numeric>
#include <fstream>

// Some typedefs for message filter shared pointers
typedef boost::shared_ptr< message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> > poseMFSubPtr;
typedef boost::shared_ptr< message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped> > twistMFSubPtr;
typedef boost::shared_ptr< tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped> > poseMFPtr;
typedef boost::shared_ptr< tf::MessageFilter<geometry_msgs::TwistWithCovarianceStamped> > twistMFPtr;
typedef boost::shared_ptr< tf::MessageFilter<sensor_msgs::Imu> > imuMFPtr;

namespace RobotLocalization
{
  typedef std::priority_queue<Measurement, std::vector<Measurement>, Measurement> MeasurementQueue;

  template<class T> class RosFilter
  {
    public:

      //! @brief Constructor
      //!
      //! The RosFilter constructor makes sure that anyone using
      //! this template is doing so with the correct object type
      //!
      RosFilter(std::vector<double> args = std::vector<double>());

      //! @brief Destructor
      //!
      //! Clears out the message filters and topic subscribers.
      //!
      ~RosFilter();

      //! @brief Callback method for receiving all acceleration (IMU) messages
      //! @param[in] msg - The ROS IMU message to take in.
      //! @param[in] topicName - The name of the IMU topic
      //! @param[in] targetFrame - The tf frame name into which we will transform this measurement
      //! @param[in] updateVector - Specifies which variables we want to update from this measurement
      //! @param[in] mahalanobisThresh - Threshold, expressed as a Mahalanobis distance, for outliter rejection
      //!
      void accelerationCallback(const sensor_msgs::Imu::ConstPtr &msg,
                                const std::string &topicName,
                                const std::string &targetFrame,
                                const std::vector<int> &updateVector,
                                const double mahalanobisThresh);

      //! @brief Adds a measurement to the queue of measurements to be processed
      //!
      //! @param[in] topicName - The name of the measurement source (only used for debugging)
      //! @param[in] measurement - The measurement to enqueue
      //! @param[in] measurementCovariance - The covariance of the measurement
      //! @param[in] updateVector - The boolean vector that specifies which variables to update from this measurement
      //! @param[in] mahalanobisThresh - Threshold, expressed as a Mahalanobis distance, for outliter rejection
      //! @param[in] time - The time of arrival (in seconds)
      //!
      void enqueueMeasurement(const std::string &topicName,
                              const Eigen::VectorXd &measurement,
                              const Eigen::MatrixXd &measurementCovariance,
                              const std::vector<int> &updateVector,
                              const double mahalanobisThresh,
                              const ros::Time &time);

      //! @brief Method for zeroing out 3D variables within measurements
      //! @param[out] measurement - The measurement whose 3D variables will be zeroed out
      //! @param[out] measurementCovariance - The covariance of the measurement
      //! @param[out] updateVector - The boolean update vector of the measurement
      //!
      //! If we're in 2D mode, then for every measurement from every sensor, we call this.
      //! It sets the 3D variables to 0, gives those variables tiny variances, and sets
      //! their updateVector values to 1.
      //!
      void forceTwoD(Eigen::VectorXd &measurement,
                     Eigen::MatrixXd &measurementCovariance,
                     std::vector<int> &updateVector);

      //! @brief Retrieves the EKF's output for broadcasting
      //! @param[out] message - The standard ROS odometry message to be filled
      //! @return true if the filter is initialized, false otherwise
      //!
      bool getFilteredOdometryMessage(nav_msgs::Odometry &message);

      //! @brief Callback method for receiving all IMU messages
      //! @param[in] msg - The ROS IMU message to take in.
      //! @param[in] topicName - The name of the IMU data topic (we support many)
      //!
      //! This method really just separates out the absolute orientation and velocity data into two new
      //! messages and adds them to their respective pose and twist callback message filters.
      //!
      void imuCallback(const sensor_msgs::Imu::ConstPtr &msg,
                       const std::string &topicName);

      //! @brief Processes all measurements in the measurement queue, in temporal order
      //!
      //! @param[in] currentTime - The time at which to carry out integration (the current time)
      //!
      void integrateMeasurements(const double currentTime);

      //! @brief Loads all parameters from file
      //!
      void loadParams();

      //! @brief Callback method for receiving all odometry messages
      //! @param[in] msg - The ROS odometry message to take in.
      //! @param[in] topicName - The name of the odometry topic (we support many)
      //!
      //! This method really just separates out the pose and twist data into two new messages, and passes
      //! them to their respective callbacks
      //!
      void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg,
                            const std::string &topicName);

      //! @brief Callback method for receiving all pose messages
      //! @param[in] msg - The ROS stamped pose with covariance message to take in
      //! @param[in] topicName - The name of the pose topic (we support many)
      //! @param[in] targetFrame - The tf frame name into which we will transform this measurement
      //! @param[in] updateVector - Specifies which variables we want to update from this measurement
      //! @param[in] differential - Whether we integrate the pose portions of this message differentially
      //! @param[in] mahalanobisThresh - Threshold, expressed as a Mahalanobis distance, for outliter rejection
      //!
      void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                        const std::string &topicName,
                        const std::string &targetFrame,
                        const std::vector<int> &updateVector,
                        const bool differential,
                        const bool relative,
                        const bool imuData,
                        const double mahalanobisThresh);

      //! @brief Main run method
      //!
      void run();

      //! @brief Callback method for manually setting/resetting the internal pose estimate
      //! @param[in] msg - The ROS stamped pose with covariance message to take in
      //!
      void setPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

      //! @brief Service callback for manually setting/resetting the internal pose estimate
      //! @param[in] request - Custom service request with pose information
      //! @param[out] response - N/A
      //! @return boolean true if successful, false if not
      bool setPoseSrvCallback(robot_localization::SetPose::Request& request,
                              robot_localization::SetPose::Response&);

      //! @brief Converts tf message filter failures to strings
      //! @param[in] reason - The failure reason object
      //! @return a string explanation of the failure
      std::string tfFailureReasonString(const tf::FilterFailureReason reason);

      //! @brief Callback method for reporting failed IMU message transforms
      //! @param[in] msg - The ROS IMU message that failed
      //! @param[in] reason - The reason for failure
      //! @param[in] topicName - The name of the IMU topic
      //! @param[in] targetFrame - The tf target frame into which we attempted to transform the message
      //!
      void transformImuFailureCallback(const sensor_msgs::Imu::ConstPtr &msg,
                                       const tf::FilterFailureReason reason,
                                       const std::string &topicName,
                                       const std::string &targetFrame);

      //! @brief Callback method for reporting failed Pose message transforms
      //! @param[in] msg - The ROS stamped pose with covariance message that failed
      //! @param[in] reason - The reason for failure
      //! @param[in] topicName - The name of the pose topic
      //! @param[in] targetFrame - The tf target frame into which we attempted to transform the message
      //!
      void transformPoseFailureCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                                        const tf::FilterFailureReason reason,
                                        const std::string &topicName,
                                        const std::string &targetFrame);

      //! @brief Callback method for reporting failed Twist message transforms
      //! @param[in] msg - The ROS stamped twist with covariance message that failed
      //! @param[in] reason - The reason for failure
      //! @param[in] topicName - The name of the twist topic
      //! @param[in] targetFrame - The tf target frame into which we attempted to transform the message
      //!
      void transformTwistFailureCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg,
                                         const tf::FilterFailureReason reason,
                                         const std::string &topicName,
                                         const std::string &targetFrame);

      //! @brief Callback method for receiving all twist messages
      //! @param[in] msg - The ROS stamped twist with covariance message to take in.
      //! @param[in] topicName - The name of the twist topic (we support many)
      //! @param[in] targetFrame - The tf frame name into which we will transform this measurement
      //! @param[in] updateVector - Specifies which variables we want to update from this measurement
      //! @param[in] mahalanobisThresh - Threshold, expressed as a Mahalanobis distance, for outliter rejection
      //!
      void twistCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg,
                         const std::string &topicName,
                         const std::string &targetFrame,
                         const std::vector<int> &updateVector,
                         const double mahalanobisThresh);

    protected:

      //! @brief Utility method for copying covariances from ROS covariance arrays
      //! to Eigen matrices
      //!
      //! This method copies the covariances and also does some data validation, which is
      //! why it requires more parameters than just the covariance containers.
      //! @param[in] arr - The source array for the covariance data
      //! @param[in] covariance - The destination matrix for the covariance data
      //! @param[in] topicName - The name of the source data topic (for debug purposes)
      //! @param[in] updateVector - The update vector for the source topic
      //! @param[in] offset - The "starting" location within the array/update vector
      //! @param[in] dimension - The number of values to copy, starting at the offset
      //!
      void copyCovariance(const double *arr,
                          Eigen::MatrixXd &covariance,
                          const std::string &topicName,
                          const std::vector<int> &updateVector,
                          const size_t offset,
                          const size_t dimension);

      //! @brief Utility method for copying covariances from Eigen matrices to ROS
      //! covariance arrays
      //!
      //! @param[in] covariance - The source matrix for the covariance data
      //! @param[in] arr - The destination array
      //! @param[in] dimension - The number of values to copy
      //!
      void copyCovariance(const Eigen::MatrixXd &covariance,
                          double *arr,
                          const size_t dimension);

      //! @brief Loads fusion settings from the config file
      //! @param[in] topicName - The name of the topic for which to load settings
      //! @return The boolean vector of update settings for each variable for this topic
      //!
      std::vector<int> loadUpdateConfig(const std::string &topicName);

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
                               const ros::Time &time, tf::StampedTransform &targetFrameTrans);

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
                               Eigen::MatrixXd &measurementCovariance);

      //! @brief Prepares a pose message for integration into the filter
      //! @param[in] msg - The pose message to prepare
      //! @param[in] topicName - The name of the topic over which this message was received
      //! @param[in] targetFrame - The target tf frame
      //! @param[in] differential - Whether we're carrying out differential integration
      //! @param[in,out] updateVector - The update vector for the data source
      //! @param[out] measurement - The pose data converted to a measurement
      //! @param[out] measurementCovariance - The covariance of the converted measurement
      //! @return true indicates that the measurement was successfully prepared, false otherwise
      //!
      bool preparePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                       const std::string &topicName,
                       const std::string &targetFrame,
                       const bool differential,
                       const bool relative,
                       const bool imuData,
                       std::vector<int> &updateVector,
                       Eigen::VectorXd &measurement,
                       Eigen::MatrixXd &measurementCovariance);

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
                        Eigen::MatrixXd &measurementCovariance);

      //! @brief Converts frame_id's to correct form for tf2. It will strip leading slash if there is one.
      //! If tf_prefix is defined it will create new name tf_prefix/param.
      //! Example: /odom --> odom  or  /odom --> tfPrefix/odom
      //! @param[in] param - user defined frame_id with or without leading slash
      //! @return new name without leading slash or with added tf_prefix
      //!
      std::string tf2NameSanitizer(const std::string & param);

      //! @brief Vector to hold our acceleration (represented as IMU) message filters so they don't go out of scope.
      //!
      std::map<std::string, imuMFPtr> accelerationMessageFilters_;

      //! @brief tf frame name for the robot's body frame
      //!
      std::string baseLinkFrameId_;

      //! @brief Used for outputting debug messages
      //!
      std::ofstream debugStream_;

      //! @brief Our filter (EKF, UKF, etc.)
      //!
      T filter_;

      //! @brief The frequency of the run loop
      //!
      double frequency_;

      //! @brief Vector to hold our IMU message filter subscriber objects so they
      //! don't go out of scope.
      //!
      std::vector<ros::Subscriber> imuTopicSubs_;

      //! @brief Stores the first measurement from each topic for relative measurements
      //!
      //! When a given sensor is being integrated in relative mode, its first measurement
      //! is effectively treated as an offset, and future measurements have this first
      //! measurement removed before they are fused. This variable stores the initial
      //! measurements. Note that this is different from using differential mode, as in
      //! differential mode, pose data is converted to twist data, resulting in boundless
      //! error growth for the variables being fused. With relative measurements, the
      //! vehicle will start with a 0 heading and position, but the measurements are still
      //! fused absolutely.
      std::map<std::string, tf::Transform> initialMeasurements_;

      //! @brief Store the last time a message from each topic was received
      //!
      //! If we're getting messages rapidly, we may accidentally get
      //! an older message arriving after a newer one. This variable keeps
      //! track of the most recent message time for each subscribed message
      //! topic. We also use it when listening to odometry messages to
      //! determine if we should be using messages from that topic.
      //!
      std::map<std::string, ros::Time> lastMessageTimes_;

      //! @brief tf frame name for the robot's map (world-fixed) frame
      //!
      std::string mapFrameId_;

      //! @brief We process measurements by queueing them up in
      //! callbacks and processing them all at once within each
      //! iteration
      //!
      MeasurementQueue measurementQueue_;

      //! @brief Node handle
      //!
      ros::NodeHandle nh_;

      //! @brief Local node handle (for params)
      //!
      ros::NodeHandle nhLocal_;

      //! @brief tf frame name for the robot's odometry (world-fixed) frame
      //!
      std::string odomFrameId_;

      //! @brief Vector to hold our odometry message filter subscriber objects so they don't go out of scope.
      //!
      std::vector<ros::Subscriber> odomTopicSubs_;

      //! @brief Vector to hold our pose message filters so they don't go out of scope.
      //!
      std::map<std::string, poseMFPtr> poseMessageFilters_;

      //! @brief Vector to hold our pose message filter subscriber objects so they don't go out of scope.
      //!
      std::vector<poseMFSubPtr> poseTopicSubs_;

      //! @brief Stores the last measurement from a given topic for differential integration
      //!
      //! To carry out differential integration, we have to (1) transform
      //! that into the target frame (probably the frame specified by
      //! @p odomFrameId_), (2) "subtract"  the previous measurement from
      //! the current measurement, and then (3) transform it again by the previous
      //! state estimate. This holds the measurements used for step (2).
      //!
      std::map<std::string, tf::Transform> previousMeasurements_;

      //! @brief We also need the previous covariance matrix for differential data
      //!
      std::map<std::string, Eigen::MatrixXd> previousMeasurementCovariances_;

      //! @brief If true, prints diagnostic messages for potential issues
      //!
      bool printDiagnostics_;

      //! @brief If including acceleration for each IMU input, whether or not we remove acceleration due to gravity
      //!
      std::map<std::string, bool> removeGravitationalAcc_;

      //! @brief Subscribes to the set_pose topic (usually published from rviz). Message
      //! type is geometry_msgs/PoseWithCovarianceStamped.
      //!
      ros::Subscriber setPoseSub_;

      //! @brief Service that allows another node to change the current state and recieve a confirmation. Uses
      //! a custom SetPose service.
      //!
      ros::ServiceServer setPoseSrv_;

      //! @brief Contains the state vector variable names in string format
      //!
      std::vector<std::string> stateVariableNames_;

      //! @brief Transform listener for managing coordinate transforms
      //!
      tf::TransformListener tfListener_;

      //! @brief tf prefix
      //!
      std::string tfPrefix_;

      //! @brief For future (or past) dating the world_frame->base_link_frame transform
      //!
      ros::Duration tfTimeOffset_;

      //! @brief Vector to hold our twist message filters so they don't go out of scope.
      //!
      std::map<std::string, twistMFPtr> twistMessageFilters_;

      //! @brief Vector to hold our twist message filter subscriber objects so they
      //! don't go out of scope.
      //!
      std::vector<twistMFSubPtr>  twistTopicSubs_;

      //! @brief Whether or not we're in 2D mode
      //!
      //! If this is true, the filter binds all 3D variables (Z,
      //! roll, pitch, and their respective velocities) to 0 for
      //! every measurement.
      //!
      bool twoDMode_;

      //! @brief Message that contains our latest transform (i.e., state)
      //!
      //! We use the vehicle's latest state in a number of places, and often
      //! use it as a transform, so this is the most convenient variable to
      //! use as our global "current state" object
      //!
      geometry_msgs::TransformStamped worldBaseLinkTransMsg_;

      //! @brief tf frame name that is the parent frame of the transform that this node will calculate and broadcast.
      //!
      std::string worldFrameId_;

  };
}

#endif
