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

#ifndef ROBOT_LOCALIZATION__ROS_FILTER_HPP_
#define ROBOT_LOCALIZATION__ROS_FILTER_HPP_

#include <robot_localization/srv/set_pose.hpp>
#include <robot_localization/srv/toggle_filter_processing.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <robot_localization/filter_base.hpp>
#include <robot_localization/filter_common.hpp>
#include <robot_localization/ros_filter_utilities.hpp>

#include <Eigen/Dense>

#include <deque>
#include <fstream>
#include <map>
#include <numeric>
#include <queue>
#include <string>
#include <memory>
#include <vector>

namespace robot_localization
{

struct CallbackData
{
  CallbackData(
    const std::string & topic_name,
    const std::vector<bool> & update_vector, const int update_sum,
    const bool differential, const bool relative,
    const double rejection_threshold)
  : topic_name_(topic_name), update_vector_(update_vector),
    update_sum_(update_sum), differential_(differential),
    relative_(relative), rejection_threshold_(rejection_threshold) {}

  std::string topic_name_;
  std::vector<bool> update_vector_;
  int update_sum_;
  bool differential_;
  bool relative_;
  double rejection_threshold_;
};

using MeasurementQueue =
  std::priority_queue<MeasurementPtr, std::vector<MeasurementPtr>,
    Measurement>;
using MeasurementHistoryDeque = std::deque<MeasurementPtr>;
using FilterStateHistoryDeque = std::deque<FilterStatePtr>;

template<class T>
class RosFilter : public rclcpp::Node
{
public:
  //! @brief Constructor
  //!
  //! The RosFilter constructor makes sure that anyone using
  //! this template is doing so with the correct object type
  //!
  explicit RosFilter(const rclcpp::NodeOptions & options);

  //! @brief Destructor
  //!
  //! Clears out the message filters and topic subscribers.
  //!
  ~RosFilter();

  //! @brief Resets the filter to its initial state
  //!
  void reset();

  //! @brief Service callback to toggle processing measurements for a standby
  //! mode but continuing to publish
  //! @param[in] request - The state requested, on (True) or off (False)
  //! @param[out] response - status if upon success
  //! @return boolean true if successful, false if not
  //!
  void toggleFilterProcessingCallback(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<
      robot_localization::srv::ToggleFilterProcessing::Request> req,
    const std::shared_ptr<
      robot_localization::srv::ToggleFilterProcessing::Response> resp);

  //! @brief Callback method for receiving all acceleration (IMU) messages
  //! @param[in] msg - The ROS IMU message to take in.
  //! @param[in] callback_data - Relevant static callback data
  //! @param[in] target_frame - The target frame_id into which to transform the
  //! data
  //!
  void accelerationCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg,
    const CallbackData & callback_data,
    const std::string & target_frame);

  //! @brief Callback method for receiving non-stamped control input
  //! @param[in] msg - The ROS twist message to take in
  //!
  void controlCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  //! @brief Callback method for receiving stamped control input
  //! @param[in] msg - The ROS stamped twist message to take in
  //!
  void
  controlStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  //! @brief Adds a measurement to the queue of measurements to be processed
  //!
  //! @param[in] topic_name - The name of the measurement source (only used for
  //! debugging)
  //! @param[in] measurement - The measurement to enqueue
  //! @param[in] measurement_covariance - The covariance of the measurement
  //! @param[in] update_vector - The boolean vector that specifies which
  //! variables to update from this measurement
  //! @param[in] mahalanobis_thresh - Threshold, expressed as a Mahalanobis
  //! distance, for outlier rejection
  //! @param[in] time - The time of arrival (in seconds)
  //!
  void enqueueMeasurement(
    const std::string & topic_name,
    const Eigen::VectorXd & measurement,
    const Eigen::MatrixXd & measurement_covariance,
    const std::vector<bool> & update_vector,
    const double mahalanobis_thresh,
    const rclcpp::Time & time);

  //! @brief Method for zeroing out 3D variables within measurements
  //! @param[out] measurement - The measurement whose 3D variables will be
  //! zeroed out
  //! @param[out] measurement_covariance - The covariance of the measurement
  //! @param[out] updateVector - The boolean update vector of the measurement
  //!
  //! If we're in 2D mode, then for every measurement from every sensor, we call
  //! this. It sets the 3D variables to 0, gives those variables tiny variances,
  //! and sets their updateVector values to 1.
  //!
  void forceTwoD(
    Eigen::VectorXd & measurement,
    Eigen::MatrixXd & measurement_covariance,
    std::vector<bool> & update_vector);

  //! @brief Method to get filter
  //! @param[out] filter - the underlying templated filter
  //!
  T & getFilter()
  {
    return filter_;
  }

  //! @brief Retrieves the EKF's output for broadcasting
  //! @param[out] message - The standard ROS odometry message to be filled
  //! @return true if the filter is initialized, false otherwise
  //!
  bool getFilteredOdometryMessage(nav_msgs::msg::Odometry * message);

  //! @brief Retrieves the EKF's acceleration output for broadcasting
  //! @param[out] message - The standard ROS acceleration message to be filled
  //! @return true if the filter is initialized, false otherwise
  //!
  bool getFilteredAccelMessage(
    geometry_msgs::msg::AccelWithCovarianceStamped * message);

  //! @brief Callback method for receiving all IMU messages
  //! @param[in] msg - The ROS IMU message to take in.
  //! @param[in] topic_name - The topic name for the IMU message (only used for
  //! debug output)
  //! @param[in] pose_callback_data - Relevant static callback data for
  //! orientation variables
  //! @param[in] twist_callback_data - Relevant static callback data for angular
  //! velocity variables
  //! @param[in] accel_callback_data - Relevant static callback data for linear
  //! acceleration variables
  //!
  //! This method separates out the orientation, angular velocity, and linear
  //! acceleration data and passed each on to its respective callback.
  //!
  void imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg,
    const std::string & topic_name,
    const CallbackData & pose_callback_data,
    const CallbackData & twist_callback_data,
    const CallbackData & accel_callback_data);

  //! @brief Processes all measurements in the measurement queue, in temporal
  //! order
  //!
  //! @param[in] current_time - The time at which to carry out integration (the
  //! current time)
  //!
  void integrateMeasurements(const rclcpp::Time & current_time);

  //! @brief Loads all parameters from file
  //!
  void loadParams();

  //! @brief callback function which is called for periodic updates
  //!
  void periodicUpdate();

  //! @brief Callback method for receiving all odometry messages
  //! @param[in] msg - The ROS odometry message to take in.
  //! @param[in] topic_name - The topic name for the odometry message (only used
  //! for debug output)
  //! @param[in] pose_callback_data - Relevant static callback data for pose
  //! variables
  //! @param[in] twist_callback_data - Relevant static callback data for twist
  //! variables
  //!
  //! This method simply separates out the pose and twist data into two new
  //! messages, and passes them into their respective callbacks
  //!
  void odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg,
    const std::string & topic_name,
    const CallbackData & pose_callback_data,
    const CallbackData & twist_callback_data);

  //! @brief Callback method for receiving all pose messages
  //! @param[in] msg - The ROS stamped pose with covariance message to take in
  //! @param[in] callback_data - Relevant static callback data
  //! @param[in] target_frame - The target frame_id into which to transform the
  //! data
  //! @param[in] imu_data - Whether this data comes from an IMU
  //!
  void poseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg,
    const CallbackData & callback_data, const std::string & target_frame,
    const bool imu_data);

  //! @brief initialize the filter
  //!
  void initialize();

  //! @brief Callback method for manually setting/resetting the internal pose
  //! estimate
  //! @param[in] msg - The ROS stamped pose with covariance message to take in
  //!
  void setPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  //! @brief Service callback for manually setting/resetting the internal pose
  //! estimate
  //!
  //! @param[in] request - Custom service request with pose information
  //! @return true if successful, false if not
  bool setPoseSrvCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<robot_localization::srv::SetPose::Request> request,
    std::shared_ptr<robot_localization::srv::SetPose::Response> response);

  //! @brief Service callback for manually enable the filter
  //! @param[in] request - N/A
  //! @param[out] response - N/A
  //! @return boolean true if successful, false if not
  bool enableFilterSrvCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    const std::shared_ptr<std_srvs::srv::Empty::Response>);

  //! @brief Callback method for receiving all twist messages
  //! @param[in] msg - The ROS stamped twist with covariance message to take in.
  //! @param[in] callback_data - Relevant static callback data
  //! @param[in] target_frame - The target frame_id into which to transform the
  //! data
  //!
  void twistCallback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg,
    const CallbackData & callback_data, const std::string & target_frame);

  //! @brief Validates filter outputs for NaNs and Infinite values
  //! @param[out] message - The standard ROS odometry message to be validated
  //! @return true if the filter output is valid, false otherwise
  //!
  bool validateFilterOutput(nav_msgs::msg::Odometry * message);

protected:
  //! @brief Finds the latest filter state before the given timestamp and makes
  //! it the current state again.
  //!
  //! This method also inserts all measurements between the older filter
  //! timestamp and now into the measurements queue.
  //!
  //! @param[in] time - The time to which the filter state should revert
  //! @return True if restoring the filter succeeded. False if not.
  //!
  bool revertTo(const rclcpp::Time & time);

  //! @brief Saves the current filter state in the queue of previous filter
  //! states
  //!
  //! These measurements will be used in backwards smoothing in the event that
  //! older measurements come in.
  //! @param[in] filter - The filter base object whose state we want to save
  //!
  void saveFilterState(T & filter);

  //! @brief Removes measurements and filter states older than the given cutoff
  //! time.
  //! @param[in] cutoff_time - Measurements and states older than this time will
  //! be dropped.
  //!
  void clearExpiredHistory(const rclcpp::Time cutoff_time);

  //! @brief Clears measurement queue
  //!
  void clearMeasurementQueue();

  //! @brief Adds a diagnostic message to the accumulating map and updates the
  //! error level
  //! @param[in] error_level - The error level of the diagnostic
  //! @param[in] topic_and_class - The sensor topic (if relevant) and class of
  //! diagnostic
  //! @param[in] message - Details of the diagnostic
  //! @param[in] is_static - Whether or not this diagnostic information is
  //! static
  //!
  void addDiagnostic(
    const int error_level, const std::string & topic_and_class,
    const std::string & message, const bool is_static);

  //! @brief Aggregates all diagnostics so they can be published
  //! @param[in] wrapper - The diagnostic status wrapper to update
  //!
  void aggregateDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper & wrapper);

  //! @brief Utility method for copying covariances from ROS covariance arrays
  //! to Eigen matrices
  //!
  //! This method copies the covariances and also does some data validation,
  //! which is why it requires more parameters than just the covariance
  //! containers.
  //! @param[in] covariance_in - The source array for the covariance data
  //! @param[in] covariance_out - The destination matrix for the covariance data
  //! @param[in] topic_name - The name of the source data topic (for debug
  //! purposes)
  //! @param[in] update_vector - The update vector for the source topic
  //! @param[in] offset - The "starting" location within the array/update vector
  //! @param[in] dimension - The number of values to copy, starting at the
  //! offset
  //!
  void copyCovariance(
    const double * covariance_in,
    Eigen::MatrixXd & covariance_out,
    const std::string & topic_name,
    const std::vector<bool> & update_vector,
    const size_t offset, const size_t dimension);

  //! @brief Utility method for copying covariances from Eigen matrices to ROS
  //! covariance arrays
  //!
  //! @param[in] covariance_in - The source matrix for the covariance data
  //! @param[in] covariance_out - The destination array
  //! @param[in] dimension - The number of values to copy
  //!
  void copyCovariance(
    const Eigen::MatrixXd & covariance_in,
    double * covariance_out, const size_t dimension);

  //! @brief Loads fusion settings from the config file
  //! @param[in] topic_name - The name of the topic for which to load settings
  //! @return The boolean vector of update settings for each variable for this
  //! topic
  //!
  std::vector<bool> loadUpdateConfig(const std::string & topic_name);

  //! @brief Prepares an IMU message's linear acceleration for integration into
  //! the filter
  //! @param[in] msg - The IMU message to prepare
  //! @param[in] topic_name - The name of the topic over which this message was
  //! received
  //! @param[in] target_frame - The target tf frame
  //! @param[in] update_vector - The update vector for the data source
  //! @param[in] measurement - The twist data converted to a measurement
  //! @param[in] measurement_covariance - The covariance of the converted
  //! measurement
  //!
  bool prepareAcceleration(
    const sensor_msgs::msg::Imu::SharedPtr msg,
    const std::string & topic_name,
    const std::string & target_frame,
    std::vector<bool> & update_vector,
    Eigen::VectorXd & measurement,
    Eigen::MatrixXd & measurement_covariance);

  //! @brief Prepares a pose message for integration into the filter
  //! @param[in] msg - The pose message to prepare
  //! @param[in] topic_name - The name of the topic over which this message was
  //! received
  //! @param[in] target_frame - The target tf frame
  //! @param[in] differential - Whether we're carrying out differential
  //! integration
  //! @param[in] relative - Whether this measurement is processed relative to
  //! the first
  //! @param[in] imuData - Whether this measurement is from an IMU
  //! @param[in,out] updateVector - The update vector for the data source
  //! @param[out] measurement - The pose data converted to a measurement
  //! @param[out] measurementCovariance - The covariance of the converted
  //! measurement
  //! @return true indicates that the measurement was successfully prepared,
  //! false otherwise
  //!
  bool preparePose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg,
    const std::string & topic_name, const std::string & target_frame,
    const bool differential, const bool relative, const bool imu_data,
    std::vector<bool> & update_vector, Eigen::VectorXd & measurement,
    Eigen::MatrixXd & measurement_covariance);

  //! @brief Prepares a twist message for integration into the filter
  //! @param[in] msg - The twist message to prepare
  //! @param[in] topicName - The name of the topic over which this message was
  //! received
  //! @param[in] targetFrame - The target tf frame
  //! @param[in,out] updateVector - The update vector for the data source
  //! @param[out] measurement - The twist data converted to a measurement
  //! @param[out] measurementCovariance - The covariance of the converted
  //! measurement
  //! @return true indicates that the measurement was successfully prepared,
  //! false otherwise
  //!
  bool prepareTwist(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg,
    const std::string & topicName, const std::string & targetFrame,
    std::vector<bool> & updateVector, Eigen::VectorXd & measurement,
    Eigen::MatrixXd & measurementCovariance);

  //! @brief Whether or not we print diagnostic messages to the /diagnostics
  //! topic
  //!
  bool print_diagnostics_;

  //! @brief Whether we publish the acceleration
  //!
  bool publish_acceleration_;

  //! @brief Whether we publish the transform from the world_frame to the
  //! base_link_frame
  //!
  bool publish_transform_;

  //! @brief Whether to reset the filters when backwards jump in time is
  //! detected
  //!
  //! This is usually the case when logs are being used and a jump in the logi
  //! is done or if a log files restarts from the beginning.
  //!
  bool reset_on_time_jump_;

  //! @brief Whether or not we use smoothing
  //!
  bool smooth_lagged_data_;

  //! @brief Whether the filter should process new measurements or not.
  //!
  bool toggled_on_;

  //! @brief Whether or not we're in 2D mode
  //!
  //! If this is true, the filter binds all 3D variables (Z,
  //! roll, pitch, and their respective velocities) to 0 for
  //! every measurement.
  //!
  bool two_d_mode_;

  //! @brief Whether or not we use a control term
  //!
  bool use_control_;

  //! @brief Start the Filter disabled at startup
  //!
  //! If this is true, the filter reads parameters and prepares publishers and subscribes
  //! but does not integrate new messages into the state vector.
  //! The filter can be enabled later using a service.
  bool disabled_at_startup_;

  //! @brief Whether the filter is enabled or not. See disabledAtStartup_.
  bool enabled_;

  //! @brief Whether we'll allow old measurements to cause a re-publication of the updated state
  bool permit_corrected_publication_;

  //! @brief The max (worst) dynamic diagnostic level.
  //!
  int dynamic_diag_error_level_;

  //! @brief The max (worst) static diagnostic level.
  //!
  int static_diag_error_level_;

  //! @brief The frequency of the run loop
  //!
  double frequency_;

  //! @brief What is the acceleration in Z due to gravity (m/s^2)? Default is
  //! +9.80665.
  //!
  double gravitational_acceleration_;

  //! @brief The depth of the history we track for smoothing/delayed measurement
  //! processing
  //!
  //! This is the guaranteed minimum buffer size for which previous states and
  //! measurements are kept.
  //!
  rclcpp::Duration history_length_;

  //! @brief tf frame name for the robot's body frame
  //!
  std::string base_link_frame_id_;

  //! @brief tf frame name for the robot's body frame
  //!
  //! When the final state is computed, we "override" the output transform and
  //! message to have this frame for its child_frame_id. This helps to enable
  //! disconnected TF trees when multiple EKF instances are being run.
  //!
  std::string base_link_output_frame_id_;

  //! @brief tf frame name for the robot's map (world-fixed) frame
  //!
  std::string map_frame_id_;

  //! @brief tf frame name for the robot's odometry (world-fixed) frame
  //!
  std::string odom_frame_id_;

  //! @brief tf frame name that is the parent frame of the transform that this
  //! node will calculate and broadcast.
  //!
  std::string world_frame_id_;

  //! @brief Used for outputting debug messages
  //!
  std::ofstream debug_stream_;

  //! @brief The most recent control input
  //!
  Eigen::VectorXd latest_control_;

  //! @brief Message that contains our latest transform (i.e., state)
  //!
  //! We use the vehicle's latest state in a number of places, and often
  //! use it as a transform, so this is the most convenient variable to
  //! use as our global "current state" object
  //!
  geometry_msgs::msg::TransformStamped world_base_link_trans_msg_;

  //! @brief last call of periodicUpdate
  //!
  rclcpp::Time last_diag_time_;

  //! @brief The time of the most recent published state
  //!
  rclcpp::Time last_published_stamp_;

  //! @brief We process measurements by queueing them up in
  //! callbacks and processing them all at once within each
  //! iteration
  //!
  MeasurementQueue measurement_queue_;

  //! @brief Contains the state vector variable names in string format
  //!
  std::vector<std::string> state_variable_names_;

  //! @brief This object accumulates dynamic diagnostics, e.g., diagnostics
  //! relating to sensor data.
  //!
  //! The values are considered transient and are cleared at every iteration.
  //!
  std::map<std::string, std::string> dynamic_diagnostics_;

  //! @brief This object accumulates static diagnostics, e.g., diagnostics
  //! relating to the configuration parameters.
  //!
  //! The values are treated as static and always reported (i.e., this object is
  //! never cleared)
  //!
  std::map<std::string, std::string> static_diagnostics_;

  //! @brief Store the last time a message from each topic was received
  //!
  //! If we're getting messages rapidly, we may accidentally get
  //! an older message arriving after a newer one. This variable keeps
  //! track of the most recent message time for each subscribed message
  //! topic. We also use it when listening to odometry messages to
  //! determine if we should be using messages from that topic.
  //!
  std::map<std::string, rclcpp::Time> last_message_times_;

  //! @brief Stores the first measurement from each topic for relative
  //! measurements
  //!
  //! When a given sensor is being integrated in relative mode, its first
  //! measurement is effectively treated as an offset, and future measurements
  //! have this first measurement removed before they are fused. This variable
  //! stores the initial measurements. Note that this is different from using
  //! differential mode, as in differential mode, pose data is converted to
  //! twist data, resulting in boundless error growth for the variables being
  //! fused. With relative measurements, the vehicle will start with a 0 heading
  //! and position, but the measurements are still fused absolutely.
  std::map<std::string, tf2::Transform> initial_measurements_;

  //! @brief If including acceleration for each IMU input, whether or not we
  //! remove acceleration due to gravity
  //!
  std::map<std::string, bool> remove_gravitational_acceleration_;

  //! @brief An implicitly time ordered queue of past filter states used for
  //! smoothing.
  //
  // front() refers to the filter state with the earliest timestamp.
  // back() refers to the filter state with the latest timestamp.
  FilterStateHistoryDeque filter_state_history_;

  //! @brief A deque of previous measurements which is implicitly ordered from
  //! earliest to latest measurement.
  // when popped from the measurement priority queue.
  // front() refers to the measurement with the earliest timestamp.
  // back() refers to the measurement with the latest timestamp.
  MeasurementHistoryDeque measurement_history_;

  //! @brief Vector to hold our subscribers until they go out of scope
  //!
  std::vector<rclcpp::SubscriptionBase::SharedPtr> topic_subs_;

  //! @brief Stores the last measurement from a given topic for differential
  //! integration
  //!
  //! To carry out differential integration, we have to (1) transform
  //! that into the target frame (probably the frame specified by
  //! @p odomFrameId_), (2) "subtract"  the previous measurement from
  //! the current measurement, and then (3) transform it again by the previous
  //! state estimate. This holds the measurements used for step (2).
  //!
  std::map<std::string, tf2::Transform> previous_measurements_;

  //! @brief We also need the previous covariance matrix for differential data
  //!
  std::map<std::string, Eigen::MatrixXd> previous_measurement_covariances_;

  //! @brief By default, the filter predicts and corrects up to the time of the
  //! latest measurement. If this is set to true, the filter does the same, but
  //! then also predicts up to the current time step.
  //!
  bool predict_to_current_time_;

  //! @brief Store the last time set pose message was received
  //!
  //! If we receive a setPose message to reset the filter, we can get in
  //! strange situations wherein we process the pose reset, but then even
  //! after another spin cycle or two, we can get a new message with a time
  //! stamp that is *older* than the setPose message's time stamp. This means
  //! we have to check the message's time stamp against the lastSetPoseTime_.
  rclcpp::Time last_set_pose_time_;

  //! @brief The time of the most recent control input
  //!
  rclcpp::Time latest_control_time_;

  //! @brief Parameter that specifies the how long we wait for a transform to
  //! become available.
  //!
  rclcpp::Duration tf_timeout_;

  //! @brief For future (or past) dating the world_frame->base_link_frame
  //! transform
  //!
  rclcpp::Duration tf_time_offset_;

  //! @brief Service that allows another node to toggle on/off filter
  //! processing while still publishing.
  //! Uses a robot_localization ToggleFilterProcessing service.
  //!
  rclcpp::Service<robot_localization::srv::ToggleFilterProcessing>::SharedPtr
    toggle_filter_processing_srv_;

  //! @brief Subscribes to the control input topic
  //!
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_sub_;

  //! @brief Subscribes to the set_pose topic (usually published from rviz).
  //! Message type is geometry_msgs/PoseWithCovarianceStamped.
  //!
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    set_pose_sub_;

  //! @brief Service that allows another node to change the current state and
  //! recieve a confirmation. Uses a custom SetPose service.
  //!
  rclcpp::Service<robot_localization::srv::SetPose>::SharedPtr
    set_pose_service_;

  //! @brief Service that allows another node to enable the filter. Uses a
  //! standard Empty service.
  //!
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr enable_filter_srv_;

  //! @brief Transform buffer for managing coordinate transforms
  //!
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  //! @brief Transform listener for receiving transforms
  //!
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  //! @brief broadcaster of worldTransform tfs
  //!
  std::shared_ptr<tf2_ros::TransformBroadcaster> world_transform_broadcaster_;

  //! @brief Used for updating the diagnostics
  //!
  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;

  //! @brief Position publisher
  //!
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr position_pub_;

  //! Acceleration publisher
  //!
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr
    accel_pub_;

  //! @brief Our filter (EKF, UKF, etc.)
  //!
  T filter_;

  //! @brief Timer for filter updates
  //!
  rclcpp::TimerBase::SharedPtr timer_;

  //! @brief optional signaling diagnostic frequency
  //!
  std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> freq_diag_;

  //! @brief minimum frequency threshold for frequency diagnostic
  //! Must be on heap since pointer is passed to diagnostic_updater::FrequencyStatusParam
  //!
  double min_frequency_;

  //! @brief maximum frequency threshold for frequency diagnostic
  //! Must be on heap since pointer is passed to diagnostic_updater::FrequencyStatusParam
  //!
  double max_frequency_;
};

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__ROS_FILTER_HPP_
