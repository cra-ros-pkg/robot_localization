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

#include <robot_localization/ekf.hpp>
#include <robot_localization/filter_utilities.hpp>
#include <robot_localization/ros_filter.hpp>
#include <robot_localization/ros_filter_utilities.hpp>
#include <robot_localization/ukf.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rcl/time.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <memory>
#include <vector>

namespace robot_localization
{
using namespace std::chrono_literals;

template<typename T>
RosFilter<T>::RosFilter(const rclcpp::NodeOptions & options)
: Node(options.arguments()[0], options),
  print_diagnostics_(true),
  publish_acceleration_(false),
  publish_transform_(true),
  reset_on_time_jump_(false),
  smooth_lagged_data_(false),
  toggled_on_(true),
  two_d_mode_(false),
  use_control_(false),
  disabled_at_startup_(false),
  enabled_(false),
  permit_corrected_publication_(false),
  dynamic_diag_error_level_(diagnostic_msgs::msg::DiagnosticStatus::OK),
  static_diag_error_level_(diagnostic_msgs::msg::DiagnosticStatus::OK),
  frequency_(30.0),
  gravitational_acceleration_(9.80665),
  history_length_(0ns),
  latest_control_(),
  last_diag_time_(0, 0, RCL_ROS_TIME),
  last_published_stamp_(0, 0, RCL_ROS_TIME),
  last_set_pose_time_(0, 0, RCL_ROS_TIME),
  latest_control_time_(0, 0, RCL_ROS_TIME),
  tf_timeout_(0ns),
  tf_time_offset_(0ns)
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  state_variable_names_.push_back("X");
  state_variable_names_.push_back("Y");
  state_variable_names_.push_back("Z");
  state_variable_names_.push_back("ROLL");
  state_variable_names_.push_back("PITCH");
  state_variable_names_.push_back("YAW");
  state_variable_names_.push_back("X_VELOCITY");
  state_variable_names_.push_back("Y_VELOCITY");
  state_variable_names_.push_back("Z_VELOCITY");
  state_variable_names_.push_back("ROLL_VELOCITY");
  state_variable_names_.push_back("PITCH_VELOCITY");
  state_variable_names_.push_back("YAW_VELOCITY");
  state_variable_names_.push_back("X_ACCELERATION");
  state_variable_names_.push_back("Y_ACCELERATION");
  state_variable_names_.push_back("Z_ACCELERATION");
}

template<typename T>
RosFilter<T>::~RosFilter()
{
  topic_subs_.clear();
  timer_.reset();
  set_pose_sub_.reset();
  control_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  diagnostic_updater_.reset();
  world_transform_broadcaster_.reset();
  set_pose_service_.reset();
  freq_diag_.reset();
  accel_pub_.reset();
  position_pub_.reset();
}

template<typename T>
void RosFilter<T>::reset()
{
  // Get rid of any initial poses (pretend we've never had a measurement)
  initial_measurements_.clear();
  previous_measurements_.clear();
  previous_measurement_covariances_.clear();

  clearMeasurementQueue();

  filter_state_history_.clear();
  measurement_history_.clear();

  // Also set the last set pose time, so we ignore all messages
  // that occur before it
  last_set_pose_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_diag_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  latest_control_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_published_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // clear tf buffer to avoid TF_OLD_DATA errors
  tf_buffer_->clear();

  // clear last message timestamp, so older messages will be accepted
  last_message_times_.clear();

  // reset filter to uninitialized state
  filter_.reset();
}

template<typename T>
void RosFilter<T>::toggleFilterProcessingCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<
    robot_localization::srv::ToggleFilterProcessing::Request> req,
  const std::shared_ptr<
    robot_localization::srv::ToggleFilterProcessing::Response> resp)
{
  if (req->on == toggled_on_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Service was called to toggle filter processing but state was already as "
      "requested.");
    resp->status = false;
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Toggling filter measurement filtering to %s.", req->on ? "On" : "Off");
    toggled_on_ = req->on;
    resp->status = true;
  }
}

// @todo: Replace with AccelWithCovarianceStamped
template<typename T>
void RosFilter<T>::accelerationCallback(
  const sensor_msgs::msg::Imu::SharedPtr msg,
  const CallbackData & callback_data,
  const std::string & target_frame)
{
  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (last_set_pose_time_ >= msg->header.stamp) {
    return;
  }

  const std::string & topic_name = callback_data.topic_name_;

  RF_DEBUG(
    "------ RosFilter<T>::accelerationCallback (" << topic_name <<
      ") ------\n")
  // "Twist message:\n" << *msg);

  if (last_message_times_.count(topic_name) == 0) {
    last_message_times_.insert(
      std::pair<std::string, rclcpp::Time>(topic_name, msg->header.stamp));
  }

  // Make sure this message is newer than the last one
  if (last_message_times_[topic_name] <= msg->header.stamp) {
    RF_DEBUG("Update vector for " << topic_name << " is:\n" << topic_name);

    Eigen::VectorXd measurement(STATE_SIZE);
    Eigen::MatrixXd measurement_covariance(STATE_SIZE, STATE_SIZE);

    measurement.setZero();
    measurement_covariance.setZero();

    // Make sure we're actually updating at least one of these variables
    std::vector<bool> update_vector_corrected = callback_data.update_vector_;

    // Prepare the twist data for inclusion in the filter
    if (prepareAcceleration(
        msg, topic_name, target_frame,
        update_vector_corrected, measurement,
        measurement_covariance))
    {
      // Store the measurement. Add an "acceleration" suffix so we know what
      // kind of measurement we're dealing with when we debug the core filter
      // logic.
      enqueueMeasurement(
        topic_name, measurement, measurement_covariance,
        update_vector_corrected,
        callback_data.rejection_threshold_, msg->header.stamp);

      RF_DEBUG(
        "Enqueued new measurement for " << topic_name <<
          "_acceleration\n");
    } else {
      RF_DEBUG(
        "Did *not* enqueue measurement for " << topic_name <<
          "_acceleration\n");
    }

    last_message_times_[topic_name] = msg->header.stamp;

    RF_DEBUG(
      "Last message time for " <<
        topic_name << " is now " <<
        filter_utilities::toSec(last_message_times_[topic_name]) <<
        "\n");
  } else {
    // else if (reset_on_time_jump_ && rclcpp::Time::isSimTime())
    //{
    //  reset();
    //}

    std::stringstream stream;
    stream << "The " << topic_name << " message has a timestamp before that of "
      "the previous message received," << " this message will be ignored. This may"
      " indicate a bad timestamp. (message time: " << msg->header.stamp.nanosec <<
      ")";

    addDiagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, topic_name +
      "_timestamp", stream.str(), false);


    RF_DEBUG(
      "Message is too old. Last message time for " <<
        topic_name << " is " <<
        filter_utilities::toSec(last_message_times_[topic_name]) <<
        ", current message time is " <<
        filter_utilities::toSec(msg->header.stamp) << ".\n");
  }

  RF_DEBUG(
    "\n----- /RosFilter<T>::accelerationCallback (" << topic_name <<
      ") ------\n");
}

template<typename T>
void RosFilter<T>::controlCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  geometry_msgs::msg::TwistStamped::SharedPtr twist_stamped_ptr =
    std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_stamped_ptr->twist = *msg;
  twist_stamped_ptr->header.frame_id = base_link_frame_id_;
  twist_stamped_ptr->header.stamp = this->now();
  controlStampedCallback(twist_stamped_ptr);
}

template<typename T>
void RosFilter<T>::controlStampedCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  if (msg->header.frame_id == base_link_frame_id_ ||
    msg->header.frame_id == "")
  {
    latest_control_(ControlMemberVx) = msg->twist.linear.x;
    latest_control_(ControlMemberVy) = msg->twist.linear.y;
    latest_control_(ControlMemberVz) = msg->twist.linear.z;
    latest_control_(ControlMemberVroll) = msg->twist.angular.x;
    latest_control_(ControlMemberVpitch) = msg->twist.angular.y;
    latest_control_(ControlMemberVyaw) = msg->twist.angular.z;
    latest_control_time_ = msg->header.stamp;

    // Update the filter with this control term
    filter_.setControl(latest_control_, msg->header.stamp);
  } else {
    // ROS_WARN_STREAM_THROTTLE(5.0, "Commanded velocities must be given in the
    // robot's body frame (" << base_link_frame_id_ << "). Message frame was " <<
    // msg->header.frame_id);
    std::cerr <<
      "Commanded velocities must be given in the robot's body frame (" <<
      base_link_frame_id_ << "). Message frame was " <<
      msg->header.frame_id << "\n";
  }
}

template<typename T>
void RosFilter<T>::enqueueMeasurement(
  const std::string & topic_name, const Eigen::VectorXd & measurement,
  const Eigen::MatrixXd & measurement_covariance,
  const std::vector<bool> & update_vector, const double mahalanobis_thresh,
  const rclcpp::Time & time)
{
  MeasurementPtr meas = MeasurementPtr(new Measurement());

  meas->topic_name_ = topic_name;
  meas->measurement_ = measurement;
  meas->covariance_ = measurement_covariance;
  meas->update_vector_ = update_vector;
  meas->time_ = time;
  meas->mahalanobis_thresh_ = mahalanobis_thresh;
  meas->latest_control_ = latest_control_;
  meas->latest_control_time_ = latest_control_time_;
  measurement_queue_.push(meas);
}

template<typename T>
void RosFilter<T>::forceTwoD(
  Eigen::VectorXd & measurement,
  Eigen::MatrixXd & measurement_covariance,
  std::vector<bool> & update_vector)
{
  measurement(StateMemberZ) = 0.0;
  measurement(StateMemberRoll) = 0.0;
  measurement(StateMemberPitch) = 0.0;
  measurement(StateMemberVz) = 0.0;
  measurement(StateMemberVroll) = 0.0;
  measurement(StateMemberVpitch) = 0.0;
  measurement(StateMemberAz) = 0.0;

  measurement_covariance(StateMemberZ, StateMemberZ) = 1e-6;
  measurement_covariance(StateMemberRoll, StateMemberRoll) = 1e-6;
  measurement_covariance(StateMemberPitch, StateMemberPitch) = 1e-6;
  measurement_covariance(StateMemberVz, StateMemberVz) = 1e-6;
  measurement_covariance(StateMemberVroll, StateMemberVroll) = 1e-6;
  measurement_covariance(StateMemberVpitch, StateMemberVpitch) = 1e-6;
  measurement_covariance(StateMemberAz, StateMemberAz) = 1e-6;

  update_vector[StateMemberZ] = 1;
  update_vector[StateMemberRoll] = 1;
  update_vector[StateMemberPitch] = 1;
  update_vector[StateMemberVz] = 1;
  update_vector[StateMemberVroll] = 1;
  update_vector[StateMemberVpitch] = 1;
  update_vector[StateMemberAz] = 1;
}

template<typename T>
bool RosFilter<T>::getFilteredOdometryMessage(nav_msgs::msg::Odometry * message)
{
  // If the filter has received a measurement at some point...
  if (filter_.getInitializedStatus()) {
    // Grab our current state and covariance estimates
    const Eigen::VectorXd & state = filter_.getState();
    const Eigen::MatrixXd & estimate_error_covariance =
      filter_.getEstimateErrorCovariance();

    // Convert from roll, pitch, and yaw back to quaternion for
    // orientation values
    tf2::Quaternion quat;
    quat.setRPY(
      state(StateMemberRoll), state(StateMemberPitch),
      state(StateMemberYaw));

    // Fill out the message
    message->pose.pose.position.x = state(StateMemberX);
    message->pose.pose.position.y = state(StateMemberY);
    message->pose.pose.position.z = state(StateMemberZ);
    message->pose.pose.orientation.x = quat.x();
    message->pose.pose.orientation.y = quat.y();
    message->pose.pose.orientation.z = quat.z();
    message->pose.pose.orientation.w = quat.w();
    message->twist.twist.linear.x = state(StateMemberVx);
    message->twist.twist.linear.y = state(StateMemberVy);
    message->twist.twist.linear.z = state(StateMemberVz);
    message->twist.twist.angular.x = state(StateMemberVroll);
    message->twist.twist.angular.y = state(StateMemberVpitch);
    message->twist.twist.angular.z = state(StateMemberVyaw);

    // Our covariance matrix layout doesn't quite match
    for (size_t i = 0; i < POSE_SIZE; i++) {
      for (size_t j = 0; j < POSE_SIZE; j++) {
        message->pose.covariance[POSE_SIZE * i + j] =
          estimate_error_covariance(i, j);
      }
    }

    // POSE_SIZE and TWIST_SIZE are currently the same size, but we can spare a
    // few cycles to be meticulous and not index a twist covariance array on the
    // size of a pose covariance array
    for (size_t i = 0; i < TWIST_SIZE; i++) {
      for (size_t j = 0; j < TWIST_SIZE; j++) {
        message->twist.covariance[TWIST_SIZE * i + j] =
          estimate_error_covariance(
          i + POSITION_V_OFFSET,
          j + POSITION_V_OFFSET);
      }
    }

    message->header.stamp = filter_.getLastMeasurementTime();
    message->header.frame_id = world_frame_id_;
    message->child_frame_id = base_link_output_frame_id_;
  }

  return filter_.getInitializedStatus();
}

template<typename T>
bool RosFilter<T>::getFilteredAccelMessage(
  geometry_msgs::msg::AccelWithCovarianceStamped * message)
{
  // If the filter has received a measurement at some point...
  if (filter_.getInitializedStatus()) {
    // Grab our current state and covariance estimates
    const Eigen::VectorXd & state = filter_.getState();
    const Eigen::MatrixXd & estimate_error_covariance =
      filter_.getEstimateErrorCovariance();

    //! Fill out the accel_msg
    message->accel.accel.linear.x = state(StateMemberAx);
    message->accel.accel.linear.y = state(StateMemberAy);
    message->accel.accel.linear.z = state(StateMemberAz);

    // Fill the covariance (only the left-upper matrix since we are not
    // estimating the rotational accelerations arround the axes
    for (size_t i = 0; i < ACCELERATION_SIZE; i++) {
      for (size_t j = 0; j < ACCELERATION_SIZE; j++) {
        // We use the POSE_SIZE since the accel cov matrix of ROS is 6x6
        message->accel.covariance[POSE_SIZE * i + j] = estimate_error_covariance(
          i + POSITION_A_OFFSET, j + POSITION_A_OFFSET);
      }
    }

    // Fill header information
    message->header.stamp = rclcpp::Time(filter_.getLastMeasurementTime());
    message->header.frame_id = base_link_output_frame_id_;
  }

  return filter_.getInitializedStatus();
}

template<typename T>
void RosFilter<T>::imuCallback(
  const sensor_msgs::msg::Imu::SharedPtr msg,
  const std::string & topic_name,
  const CallbackData & pose_callback_data,
  const CallbackData & twist_callback_data,
  const CallbackData & accel_callback_data)
{
  RF_DEBUG(
    "------ RosFilter<T>::imuCallback (" <<
      topic_name << ") ------\n")         // << "IMU message:\n" << *msg);

  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (last_set_pose_time_ >= msg->header.stamp) {
    std::stringstream stream;
    stream << "The " << topic_name << " message has a timestamp equal to or"
      " before the last filter reset, " << "this message will be ignored. This may"
      "indicate an empty or bad timestamp. (message time: " << msg->header.stamp.nanosec <<
      ")";
    addDiagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      topic_name + "_timestamp", stream.str(), false);


    RF_DEBUG(
      "Received message that preceded the most recent pose reset. "
      "Ignoring...");

    return;
  }

  // As with the odometry message, we can separate out the pose- and
  // twist-related variables in the IMU message and pass them to the pose and
  // twist callbacks (filters)
  if (pose_callback_data.update_sum_ > 0) {
    // Per the IMU message specification, if the IMU does not provide
    // orientation, then its first covariance value should be set to -1, and we
    // should ignore that portion of the message. robot_localization allows
    // users to explicitly ignore data using its parameters, but we should also
    // be compliant with message specs.
    if (::fabs(msg->orientation_covariance[0] + 1) < 1e-9) {
      RF_DEBUG(
        "Received IMU message with -1 as its first covariance value for "
        "orientation. "
        "Ignoring orientation...");
    } else {
      // Extract the pose (orientation) data, pass it to its filter
      geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pos_ptr =
        std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
      pos_ptr->header = msg->header;
      pos_ptr->pose.pose.orientation = msg->orientation;

      // Copy the covariance for roll, pitch, and yaw
      for (size_t i = 0; i < ORIENTATION_SIZE; i++) {
        for (size_t j = 0; j < ORIENTATION_SIZE; j++) {
          pos_ptr->pose.covariance[POSE_SIZE * (i + ORIENTATION_SIZE) +
            (j + ORIENTATION_SIZE)] =
            msg->orientation_covariance[ORIENTATION_SIZE * i + j];
        }
      }

      // IMU data gets handled a bit differently, since the message is ambiguous
      // and has only a single frame_id, even though the data in it is reported
      // in two different frames. As we assume users will specify a base_link to
      // imu transform, we make the target frame base_link_frame_id_ and tell
      // the poseCallback that it is working with IMU data. This will cause it
      // to apply different logic to the data.
      poseCallback(pos_ptr, pose_callback_data, base_link_frame_id_, true);
    }
  }

  if (twist_callback_data.update_sum_ > 0) {
    // Ignore rotational velocity if the first covariance value is -1
    if (::fabs(msg->angular_velocity_covariance[0] + 1) < 1e-9) {
      RF_DEBUG(
        "Received IMU message with -1 as its first covariance value for "
        "angular "
        "velocity. Ignoring angular velocity...");
    } else {
      // Repeat for velocity
      geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist_ptr =
        std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
      twist_ptr->header = msg->header;
      twist_ptr->twist.twist.angular = msg->angular_velocity;

      // Copy the covariance
      for (size_t i = 0; i < ORIENTATION_SIZE; i++) {
        for (size_t j = 0; j < ORIENTATION_SIZE; j++) {
          twist_ptr->twist.covariance[TWIST_SIZE * (i + ORIENTATION_SIZE) +
            (j + ORIENTATION_SIZE)] =
            msg->angular_velocity_covariance[ORIENTATION_SIZE * i + j];
        }
      }

      twistCallback(twist_ptr, twist_callback_data, base_link_frame_id_);
    }
  }

  if (accel_callback_data.update_sum_ > 0) {
    // Ignore linear acceleration if the first covariance value is -1
    if (::fabs(msg->linear_acceleration_covariance[0] + 1) < 1e-9) {
      RF_DEBUG(
        "Received IMU message with -1 as its first covariance value for "
        "linear "
        "acceleration. Ignoring linear acceleration...");
    } else {
      // Pass the message on
      accelerationCallback(msg, accel_callback_data, base_link_frame_id_);
    }
  }

  RF_DEBUG("\n----- /RosFilter<T>::imuCallback (" << topic_name << ") ------\n");
}

template<typename T>
void RosFilter<T>::integrateMeasurements(const rclcpp::Time & current_time)
{
  RF_DEBUG(
    "------ RosFilter<T>::integrateMeasurements ------\n\n"
    "Integration time is " <<
      std::setprecision(20) << filter_utilities::toSec(current_time) <<
      "\n" <<
      measurement_queue_.size() << " measurements in queue.\n");

  bool predict_to_current_time = predict_to_current_time_;

  // If we have any measurements in the queue, process them
  if (!measurement_queue_.empty()) {
    // Check if the first measurement we're going to process is older than the
    // filter's last measurement. This means we have received an out-of-sequence
    // message (one with an old timestamp), and we need to revert both the
    // filter state and measurement queue to the first state that preceded the
    // time stamp of our first measurement.
    const MeasurementPtr & first_measurement = measurement_queue_.top();
    int restored_measurement_count = 0;
    if (smooth_lagged_data_ &&
      first_measurement->time_ < filter_.getLastMeasurementTime())
    {
      RF_DEBUG(
        "Received a measurement that was " <<
          filter_utilities::toSec(
          filter_.getLastMeasurementTime() -
          first_measurement->time_) <<
          " seconds in the past. Reverting filter state and "
          "measurement queue...");

      int original_count = static_cast<int>(measurement_queue_.size());
      const rclcpp::Time first_measurement_time = first_measurement->time_;
      const std::string first_measurement_topic =
        first_measurement->topic_name_;
      // revertTo may invalidate first_measurement
      if (!revertTo(first_measurement_time - rclcpp::Duration(1ns))) {
        RF_DEBUG(
          "ERROR: history interval is too small to revert to time " <<
            filter_utilities::toSec(first_measurement_time) << "\n");
        // ROS_WARN_STREAM_DELAYED_THROTTLE(history_length_,
        //   "Received old measurement for topic " << first_measurement_topic <<
        //   ", but history interval is insufficiently sized. "
        //   "Measurement time is " << std::setprecision(20) <<
        //   first_measurement_time <<
        //   ", current time is " << current_time <<
        //   ", history length is " << history_length_ << ".");
        restored_measurement_count = 0;
      }

      restored_measurement_count =
        static_cast<int>(measurement_queue_.size()) - original_count;
    }

    while (!measurement_queue_.empty() && rclcpp::ok()) {
      MeasurementPtr measurement = measurement_queue_.top();

      // If we've reached a measurement that has a time later than now, it
      // should wait until a future iteration. Since measurements are stored in
      // a priority queue, all remaining measurements will be in the future.
      if (current_time < measurement->time_) {
        break;
      }

      measurement_queue_.pop();

      // When we receive control messages, we call this directly in the control
      // callback. However, we also associate a control with each sensor message
      // so that we can support lagged smoothing. As we cannot guarantee that
      // the new control callback will fire before a new measurement, we should
      // only perform this operation if we are processing messages from the
      // history. Otherwise, we may get a new measurement, store the "old"
      // latest control, then receive a control, call setControl, and then
      // overwrite that value with this one (i.e., with the "old" control we
      // associated with the measurement).
      if (use_control_ && restored_measurement_count > 0) {
        filter_.setControl(
          measurement->latest_control_,
          measurement->latest_control_time_);
        restored_measurement_count--;
      }

      // This will call predict and, if necessary, correct
      filter_.processMeasurement(*(measurement.get()));

      // Store old states and measurements if we're smoothing
      if (smooth_lagged_data_) {
        // Invariant still holds: measurementHistoryDeque_.back().time_ <
        // measurement_queue_.top().time_
        measurement_history_.push_back(measurement);

        // We should only save the filter state once per unique timstamp
        if (measurement_queue_.empty() ||
          measurement_queue_.top()->time_ !=
          filter_.getLastMeasurementTime())
        {
          saveFilterState(filter_);
        }
      }
    }
  } else if (filter_.getInitializedStatus()) {
    // In the event that we don't get any measurements for a long time,
    // we still need to continue to estimate our state. Therefore, we
    // should project the state forward here.
    rclcpp::Duration last_update_delta =
      current_time - filter_.getLastMeasurementTime();

    // If we get a large delta, then continuously predict until
    if (last_update_delta >= filter_.getSensorTimeout()) {
      predict_to_current_time = true;

      RF_DEBUG(
        "Sensor timeout! Last measurement time was " <<
          filter_utilities::toSec(filter_.getLastMeasurementTime()) <<
          ", current time is " << filter_utilities::toSec(current_time) <<
          ", delta is " << filter_utilities::toSec(last_update_delta) <<
          "\n");
    }
  } else {
    RF_DEBUG("Filter not yet initialized.\n");
  }

  if (filter_.getInitializedStatus() && predict_to_current_time) {
    rclcpp::Duration last_update_delta =
      current_time - filter_.getLastMeasurementTime();

    filter_.validateDelta(last_update_delta);
    filter_.predict(current_time, last_update_delta);

    // Update the last measurement time and last update time
    filter_.setLastMeasurementTime(
      filter_.getLastMeasurementTime() +
      last_update_delta);
  }

  RF_DEBUG("\n----- /RosFilter<T>::integrateMeasurements ------\n");
}

template<typename T>
void RosFilter<T>::loadParams()
{
  /* For diagnostic purposes, collect information about how many different
   * sources are measuring each absolute pose variable and do not have
   * differential integration enabled.
   */
  std::map<StateMembers, int> abs_pose_var_counts;
  abs_pose_var_counts[StateMemberX] = 0;
  abs_pose_var_counts[StateMemberY] = 0;
  abs_pose_var_counts[StateMemberZ] = 0;
  abs_pose_var_counts[StateMemberRoll] = 0;
  abs_pose_var_counts[StateMemberPitch] = 0;
  abs_pose_var_counts[StateMemberYaw] = 0;

  // Same for twist variables
  std::map<StateMembers, int> twist_var_counts;
  twist_var_counts[StateMemberVx] = 0;
  twist_var_counts[StateMemberVy] = 0;
  twist_var_counts[StateMemberVz] = 0;
  twist_var_counts[StateMemberVroll] = 0;
  twist_var_counts[StateMemberVpitch] = 0;
  twist_var_counts[StateMemberVyaw] = 0;

  // Determine if we'll be printing diagnostic information
  print_diagnostics_ = this->declare_parameter("print_diagnostics", false);

  // Check for custom gravitational acceleration value
  gravitational_acceleration_ = this->declare_parameter(
    "gravitational_acceleration",
    gravitational_acceleration_);

  // Grab the debug param. If true, the node will produce a LOT of output.
  bool debug = this->declare_parameter("debug", false);
  std::string debug_out_file = "robot_localization_debug.txt";
  if (debug) {
    try {
      debug_out_file = this->declare_parameter("debug_out_file", debug_out_file);
      debug_stream_.open(debug_out_file.c_str());

      // Make sure we succeeded
      if (debug_stream_.is_open()) {
        filter_.setDebug(debug, &debug_stream_);
      } else {
        std::cerr <<
          "RosFilter<T>::loadParams() - unable to create debug output file " <<
          debug_out_file << "\n";
      }
    } catch (const std::exception & e) {
      std::cerr <<
        "RosFilter<T>::loadParams() - unable to create debug output file" <<
        debug_out_file << ". Error was " << e.what() << "\n";
    }
  }

  // These params specify the name of the robot's body frame (typically
  // base_link) and odometry frame (typically odom)
  map_frame_id_ = this->declare_parameter("map_frame", std::string("map"));
  odom_frame_id_ = this->declare_parameter("odom_frame", std::string("odom"));
  base_link_frame_id_ = this->declare_parameter(
    "base_link_frame",
    std::string("base_link"));
  base_link_output_frame_id_ = this->declare_parameter(
    "base_link_frame_output",
    base_link_frame_id_);

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
   * The state estimation nodes in robot_localization therefore have two
   * "modes." If your world_frame parameter value matches the odom_frame
   * parameter value, then robot_localization will assume someone else is
   * broadcasting a transform from odom_frame->base_link_frame, and it will
   * compute the map_frame->odom_frame transform. Otherwise, it will simply
   * compute the odom_frame->base_link_frame transform.
   *
   * The default is the latter behavior (broadcast of odom->base_link).
   */
  world_frame_id_ = this->declare_parameter("world_frame", odom_frame_id_);

  if (map_frame_id_ == odom_frame_id_ ||
    odom_frame_id_ == base_link_frame_id_ ||
    map_frame_id_ == base_link_frame_id_ ||
    odom_frame_id_ == base_link_output_frame_id_ ||
    map_frame_id_ == base_link_output_frame_id_)
  {
    std::cerr <<
      "Invalid frame configuration! The values for map_frame, odom_frame, "
      "and base_link_frame must be unique. If using a base_link_frame_output "
      "values, it must not match the map_frame or odom_frame."
      "\n";
  }

  // Try to resolve tf_prefix
  std::string tf_prefix = "";
  std::string tf_prefix_path = "";
  this->declare_parameter("tf_prefix", rclcpp::PARAMETER_STRING);
  if (this->get_parameter("tf_prefix", tf_prefix_path)) {
    // Append the tf prefix in a tf2-friendly manner
    filter_utilities::appendPrefix(tf_prefix, map_frame_id_);
    filter_utilities::appendPrefix(tf_prefix, odom_frame_id_);
    filter_utilities::appendPrefix(tf_prefix, base_link_frame_id_);
    filter_utilities::appendPrefix(tf_prefix, base_link_output_frame_id_);
    filter_utilities::appendPrefix(tf_prefix, world_frame_id_);
  }

  // Whether we're publshing the world_frame->base_link_frame transform
  publish_transform_ = this->declare_parameter("publish_tf", true);

  // Whether we're publishing the acceleration state transform
  publish_acceleration_ = this->declare_parameter("publish_acceleration", false);

  // Whether we'll allow old measurements to cause a re-publication of the updated state
  permit_corrected_publication_ = this->declare_parameter("permit_corrected_publication", false);

  // Transform future dating
  double offset_tmp = this->declare_parameter("transform_time_offset", 0.0);
  tf_time_offset_ = rclcpp::Duration::from_seconds(offset_tmp);

  // Transform timeout
  double timeout_tmp = this->declare_parameter("transform_timeout", 0.0);
  tf_timeout_ = rclcpp::Duration::from_seconds(timeout_tmp);

  // Update frequency and sensor timeout
  frequency_ = this->declare_parameter("frequency", 30.0);

  double sensor_timeout = this->declare_parameter("sensor_timeout", 1.0 / frequency_);
  filter_.setSensorTimeout(rclcpp::Duration::from_seconds(sensor_timeout));

  // Determine if we're in 2D mode
  two_d_mode_ = this->declare_parameter("two_d_mode", false);

  // Smoothing window size
  smooth_lagged_data_ = this->declare_parameter("smooth_lagged_data", false);
  double history_length_double = this->declare_parameter("history_length", 0.0);

  if (!smooth_lagged_data_ && std::abs(history_length_double) > 0) {
    std::cerr << "Filter history interval of " << history_length_double <<
      " specified, but smooth_lagged_data is set to false. Lagged "
      "data will not be smoothed.";
  }

  if (smooth_lagged_data_ && history_length_double < 0) {
    std::cerr << "Negative history interval of " << history_length_double <<
      " specified. Absolute value will be assumed.";
  }

  history_length_ = rclcpp::Duration::from_seconds(std::abs(history_length_double));

  // Whether we reset filter on jump back in time
  reset_on_time_jump_ = this->declare_parameter("reset_on_time_jump", false);

  // Determine if we're using a control term
  double control_timeout = sensor_timeout;
  std::vector<bool> control_update_vector;
  std::vector<double> acceleration_limits;
  std::vector<double> acceleration_gains;
  std::vector<double> deceleration_limits;
  std::vector<double> deceleration_gains;

  use_control_ = this->declare_parameter("use_control", false);
  control_timeout = this->declare_parameter("control_timeout", 0.0);

  if (use_control_) {
    this->declare_parameter("control_config", rclcpp::PARAMETER_BOOL_ARRAY);
    if (this->get_parameter("control_config", control_update_vector)) {
      if (control_update_vector.size() != TWIST_SIZE) {
        std::cerr << "Control configuration must be of size " << TWIST_SIZE <<
          ". Provided config was of "
          "size " <<
          control_update_vector.size() <<
          ". No control term will be used.\n";
        use_control_ = false;
      }
    } else {
      std::cerr << "use_control is set to true, but control_config is missing. "
        "No control term will be used.\n";
      use_control_ = false;
    }

    this->declare_parameter("acceleration_limits", rclcpp::PARAMETER_DOUBLE_ARRAY);
    if (this->get_parameter("acceleration_limits", acceleration_limits)) {
      if (acceleration_limits.size() != TWIST_SIZE) {
        std::cerr << "Acceleration configuration must be of size " << TWIST_SIZE <<
          ". Provided config was of "
          "size " <<
          acceleration_limits.size() <<
          ". No control term will be used.\n";
        use_control_ = false;
      }
    } else {
      std::cerr << "use_control is set to true, but acceleration_limits is "
        "missing. Will use default values.\n";
      acceleration_limits.resize(TWIST_SIZE, 1.0);
    }

    this->declare_parameter("acceleration_gains", rclcpp::PARAMETER_DOUBLE_ARRAY);
    if (this->get_parameter("acceleration_gains", acceleration_gains)) {
      const int size = acceleration_gains.size();
      if (size != TWIST_SIZE) {
        std::cerr << "Acceleration gain configuration must be of size " <<
          TWIST_SIZE << ". Provided config was of size " << size <<
          ". All gains will be assumed to be 1.\n";
        std::fill_n(
          acceleration_gains.begin(), std::min(size, TWIST_SIZE),
          1.0);
        acceleration_gains.resize(TWIST_SIZE, 1.0);
      }
    }

    this->declare_parameter("deceleration_limits", rclcpp::PARAMETER_DOUBLE_ARRAY);
    if (this->get_parameter("deceleration_limits", deceleration_limits)) {
      if (deceleration_limits.size() != TWIST_SIZE) {
        std::cerr << "Deceleration configuration must be of size " << TWIST_SIZE <<
          ". Provided config was of size " <<
          deceleration_limits.size() <<
          ". No control term will be used.\n";
        use_control_ = false;
      }
    } else {
      std::cout << "use_control is set to true, but no deceleration_limits "
        "specified. Will use acceleration "
        "limits.\n";
      deceleration_limits = acceleration_limits;
    }

    this->declare_parameter("deceleration_gains", rclcpp::PARAMETER_DOUBLE_ARRAY);
    if (this->get_parameter("deceleration_gains", deceleration_gains)) {
      const int size = deceleration_gains.size();
      if (size != TWIST_SIZE) {
        std::cerr << "Deceleration gain configuration must be of size " <<
          TWIST_SIZE << ". Provided config was of size " << size <<
          ". All gains will be assumed to be 1.\n";
        std::fill_n(
          deceleration_gains.begin(), std::min(size, TWIST_SIZE),
          1.0);
        deceleration_gains.resize(TWIST_SIZE, 1.0);
      }
    } else {
      std::cout << "use_control is set to true, but no deceleration_gains "
        "specified. Will use acceleration "
        "gains.\n";
      deceleration_gains = acceleration_gains;
    }
  } else {
    control_update_vector.resize(TWIST_SIZE, 0);
    acceleration_limits.resize(TWIST_SIZE, 1.0);
    acceleration_gains.resize(TWIST_SIZE, 1.0);
    deceleration_limits.resize(TWIST_SIZE, 1.0);
    deceleration_gains.resize(TWIST_SIZE, 1.0);
  }

  bool dynamic_process_noise_covariance = this->declare_parameter(
    "dynamic_process_noise_covariance", false);
  filter_.setUseDynamicProcessNoiseCovariance(
    dynamic_process_noise_covariance);

  std::vector<double> initial_state;
  this->declare_parameter("initial_state", rclcpp::PARAMETER_DOUBLE_ARRAY);
  if (this->get_parameter("initial_state", initial_state)) {
    if (initial_state.size() != STATE_SIZE) {
      std::cerr << "Initial state must be of size " << STATE_SIZE <<
        ". Provided config was of size " << initial_state.size() <<
        ". The initial state will be ignored.\n";
    } else {
      Eigen::Map<Eigen::VectorXd> eigen_state(initial_state.data(),
        initial_state.size());
      filter_.setState(eigen_state);
    }
  }

  // Check if the filter should start or not
  disabled_at_startup_ = this->declare_parameter<bool>("disabled_at_startup", false);
  enabled_ = !disabled_at_startup_;

  // Debugging writes to file
  RF_DEBUG(
    std::boolalpha <<
      "tf_prefix is " << tf_prefix <<
      "\nmap_frame is " << map_frame_id_ <<
      "\nodom_frame is " << odom_frame_id_ <<
      "\nbase_link_frame is " << base_link_frame_id_ <<
      "\nbase_link_output_frame is " << base_link_output_frame_id_ <<
      "\nworld_frame is " << world_frame_id_ <<
      "\ntransform_time_offset is " << filter_utilities::toSec(tf_time_offset_) <<
      "\ntransform_timeout is " << filter_utilities::toSec(tf_timeout_) <<
      "\nfrequency is " << frequency_ <<
      "\nsensor_timeout is " << filter_utilities::toSec(filter_.getSensorTimeout()) <<
      "\ntwo_d_mode is " << (two_d_mode_ ? "true" : "false") <<
      "\nsmooth_lagged_data is " << (smooth_lagged_data_ ? "true" : "false") <<
      "\nhistory_length is " << filter_utilities::toSec(history_length_) <<
      "\nuse_control is " << use_control_ <<
      "\ncontrol_config is " << control_update_vector <<
      "\ncontrol_timeout is " << control_timeout <<
      "\nacceleration_limits are " << acceleration_limits <<
      "\nacceleration_gains are " << acceleration_gains <<
      "\ndeceleration_limits are " << deceleration_limits <<
      "\ndeceleration_gains are " << deceleration_gains <<
      "\ninitial state is " << filter_.getState() <<
      "\ndynamic_process_noise_covariance is " << dynamic_process_noise_covariance <<
      "\npermit_corrected_publication is " << permit_corrected_publication_ <<
      "\nprint_diagnostics is " << print_diagnostics_ << "\n");

  // Create a subscriber for manually setting/resetting pose
  set_pose_sub_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "set_pose", rclcpp::QoS(1),
    std::bind(&RosFilter<T>::setPoseCallback, this, std::placeholders::_1));

  // Create a service for manually setting/resetting pose
  set_pose_service_ =
    this->create_service<robot_localization::srv::SetPose>(
    "set_pose", std::bind(
      &RosFilter<T>::setPoseSrvCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Create a service for manually enabling the filter
  enable_filter_srv_ =
    this->create_service<std_srvs::srv::Empty>(
    "enable", std::bind(
      &RosFilter::enableFilterSrvCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Create a service for toggling processing new measurements while still
  // publishing
  toggle_filter_processing_srv_ =
    this->create_service<robot_localization::srv::ToggleFilterProcessing>(
    "toggle", std::bind(
      &RosFilter<T>::toggleFilterProcessingCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Init the last measurement time so we don't get a huge initial delta
  filter_.setLastMeasurementTime(this->now());

  // Now pull in each topic to which we want to subscribe.
  // Start with odom.
  size_t topic_ind = 0;
  bool more_params = false;
  do {
    // Build the string in the form of "odomX", where X is the odom topic
    // number, then check if we have any parameters with that value. Users need
    // to make sure they don't have gaps in their configs (e.g., odom0 and then
    // odom2)
    std::stringstream ss;
    ss << "odom" << topic_ind++;
    std::string odom_topic_name = ss.str();
    std::string odom_topic;
    this->declare_parameter(odom_topic_name, rclcpp::PARAMETER_STRING);

    rclcpp::Parameter parameter;
    if (this->get_parameter(odom_topic_name, parameter)) {
      more_params = true;
      odom_topic = parameter.as_string();
    } else {
      more_params = false;
    }

    if (more_params) {
      // Determine if we want to integrate this sensor differentially
      bool differential = this->declare_parameter(
        odom_topic_name + std::string("_differential"),
        false);

      // Determine if we want to integrate this sensor relatively
      bool relative = this->declare_parameter(odom_topic_name + std::string("_relative"), false);

      if (relative && differential) {
        std::cerr << "Both " << odom_topic_name << "_differential" <<
          " and " << odom_topic_name <<
          "_relative were set to true. Using differential mode.\n";

        relative = false;
      }

      // Check for pose rejection threshold
      double pose_mahalanobis_thresh = this->declare_parameter(
        odom_topic_name +
        std::string("_pose_rejection_threshold"),
        std::numeric_limits<double>::max());

      // Check for twist rejection threshold
      double twist_mahalanobis_thresh = this->declare_parameter(
        odom_topic_name +
        std::string("_twist_rejection_threshold"),
        std::numeric_limits<double>::max());

      // Set optional custom queue size
      int queue_size = this->declare_parameter(
        odom_topic_name +
        std::string("_queue_size"), 10);

      // Now pull in its boolean update vector configuration. Create separate
      // vectors for pose and twist data, and then zero out the opposite values
      // in each vector (no pose data in the twist update vector and
      // vice-versa).
      std::vector<bool> update_vec = loadUpdateConfig(odom_topic_name);
      std::vector<bool> pose_update_vec = update_vec;
      std::fill(
        pose_update_vec.begin() + POSITION_V_OFFSET,
        pose_update_vec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
      std::vector<bool> twist_update_vec = update_vec;
      std::fill(
        twist_update_vec.begin() + POSITION_OFFSET,
        twist_update_vec.begin() + POSITION_OFFSET + POSE_SIZE, 0);

      int pose_update_sum =
        std::accumulate(pose_update_vec.begin(), pose_update_vec.end(), 0);
      int twist_update_sum =
        std::accumulate(twist_update_vec.begin(), twist_update_vec.end(), 0);

      const CallbackData pose_callback_data(
        odom_topic_name + "_pose", pose_update_vec, pose_update_sum,
        differential, relative, pose_mahalanobis_thresh);

      const CallbackData twist_callback_data(
        odom_topic_name + "_twist", twist_update_vec, twist_update_sum, false,
        false, twist_mahalanobis_thresh);

      // Store the odometry topic subscribers so they don't go out of scope.
      if (pose_update_sum + twist_update_sum > 0) {
        std::function<void(const std::shared_ptr<nav_msgs::msg::Odometry>)>
        odom_callback = std::bind(
          &RosFilter<T>::odometryCallback, this,
          std::placeholders::_1, odom_topic_name,
          pose_callback_data, twist_callback_data);

        auto custom_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(queue_size));
        topic_subs_.push_back(
          this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, custom_qos,
            odom_callback));
      } else {
        std::stringstream stream;
        stream << odom_topic << " is listed as an input topic, but all update "
          "variables are false";

        addDiagnostic(
          diagnostic_msgs::msg::DiagnosticStatus::WARN,
          odom_topic + "_configuration", stream.str(), true);
      }

      if (pose_update_sum > 0) {
        if (differential) {
          twist_var_counts[StateMemberVx] += pose_update_vec[StateMemberX];
          twist_var_counts[StateMemberVy] += pose_update_vec[StateMemberY];
          twist_var_counts[StateMemberVz] += pose_update_vec[StateMemberZ];
          twist_var_counts[StateMemberVroll] +=
            pose_update_vec[StateMemberRoll];
          twist_var_counts[StateMemberVpitch] +=
            pose_update_vec[StateMemberPitch];
          twist_var_counts[StateMemberVyaw] += pose_update_vec[StateMemberYaw];
        } else {
          abs_pose_var_counts[StateMemberX] += pose_update_vec[StateMemberX];
          abs_pose_var_counts[StateMemberY] += pose_update_vec[StateMemberY];
          abs_pose_var_counts[StateMemberZ] += pose_update_vec[StateMemberZ];
          abs_pose_var_counts[StateMemberRoll] +=
            pose_update_vec[StateMemberRoll];
          abs_pose_var_counts[StateMemberPitch] +=
            pose_update_vec[StateMemberPitch];
          abs_pose_var_counts[StateMemberYaw] +=
            pose_update_vec[StateMemberYaw];
        }
      }

      if (twist_update_sum > 0) {
        twist_var_counts[StateMemberVx] += twist_update_vec[StateMemberVx];
        twist_var_counts[StateMemberVy] += twist_update_vec[StateMemberVx];
        twist_var_counts[StateMemberVz] += twist_update_vec[StateMemberVz];
        twist_var_counts[StateMemberVroll] +=
          twist_update_vec[StateMemberVroll];
        twist_var_counts[StateMemberVpitch] +=
          twist_update_vec[StateMemberVpitch];
        twist_var_counts[StateMemberVyaw] += twist_update_vec[StateMemberVyaw];
      }

      RF_DEBUG(
        "Subscribed to " <<
          odom_topic << " (" << odom_topic_name << ")\n\t" <<
          odom_topic_name << "_differential is " <<
        (differential ? "true" : "false") << "\n\t" << odom_topic_name <<
          "_pose_rejection_threshold is " << pose_mahalanobis_thresh <<
          "\n\t" << odom_topic_name << "_twist_rejection_threshold is " <<
          twist_mahalanobis_thresh << "\n\t" << odom_topic_name <<
          " pose update vector is " << pose_update_vec << "\t" <<
          odom_topic_name << " twist update vector is " <<
          twist_update_vec);
    }
  } while (more_params);

  // Repeat for pose
  topic_ind = 0;
  more_params = false;
  do {
    std::stringstream ss;
    ss << "pose" << topic_ind++;
    std::string pose_topic_name = ss.str();
    std::string pose_topic;
    this->declare_parameter(pose_topic_name, rclcpp::PARAMETER_STRING);

    rclcpp::Parameter parameter;
    if (this->get_parameter(pose_topic_name, parameter)) {
      more_params = true;
      pose_topic = parameter.as_string();
    } else {
      more_params = false;
    }

    if (more_params) {
      bool differential = this->declare_parameter(
        pose_topic_name + std::string("_differential"),
        false);

      // Determine if we want to integrate this sensor relatively
      bool relative = this->declare_parameter(
        pose_topic_name + std::string("_relative"),
        false);

      if (relative && differential) {
        std::cerr << "Both " << pose_topic_name << "_differential" <<
          " and " << pose_topic_name <<
          "_relative were set to true. Using differential mode.\n";

        relative = false;
      }

      // Check for pose rejection threshold
      double pose_mahalanobis_thresh = this->declare_parameter(
        pose_topic_name +
        std::string("_rejection_threshold"),
        std::numeric_limits<double>::max());

      // Set optional custom queue size
      int queue_size = this->declare_parameter(
        pose_topic_name +
        std::string("_queue_size"), 10);

      // Pull in the sensor's config, zero out values that are invalid for the
      // pose type
      std::vector<bool> pose_update_vec = loadUpdateConfig(pose_topic_name);
      std::fill(
        pose_update_vec.begin() + POSITION_V_OFFSET,
        pose_update_vec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
      std::fill(
        pose_update_vec.begin() + POSITION_A_OFFSET,
        pose_update_vec.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE,
        0);

      int pose_update_sum =
        std::accumulate(pose_update_vec.begin(), pose_update_vec.end(), 0);

      if (pose_update_sum > 0) {
        const CallbackData callback_data(pose_topic_name, pose_update_vec,
          pose_update_sum, differential,
          relative, pose_mahalanobis_thresh);

        std::function<void(const std::shared_ptr<
            geometry_msgs::msg::PoseWithCovarianceStamped>)>
        pose_callback =
          std::bind(
          &RosFilter<T>::poseCallback, this, std::placeholders::_1,
          callback_data, world_frame_id_, false);

        auto custom_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(queue_size));

        topic_subs_.push_back(
          this->create_subscription<
            geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic, custom_qos, pose_callback));

        if (differential) {
          twist_var_counts[StateMemberVx] += pose_update_vec[StateMemberX];
          twist_var_counts[StateMemberVy] += pose_update_vec[StateMemberY];
          twist_var_counts[StateMemberVz] += pose_update_vec[StateMemberZ];
          twist_var_counts[StateMemberVroll] +=
            pose_update_vec[StateMemberRoll];
          twist_var_counts[StateMemberVpitch] +=
            pose_update_vec[StateMemberPitch];
          twist_var_counts[StateMemberVyaw] += pose_update_vec[StateMemberYaw];
        } else {
          abs_pose_var_counts[StateMemberX] += pose_update_vec[StateMemberX];
          abs_pose_var_counts[StateMemberY] += pose_update_vec[StateMemberY];
          abs_pose_var_counts[StateMemberZ] += pose_update_vec[StateMemberZ];
          abs_pose_var_counts[StateMemberRoll] +=
            pose_update_vec[StateMemberRoll];
          abs_pose_var_counts[StateMemberPitch] +=
            pose_update_vec[StateMemberPitch];
          abs_pose_var_counts[StateMemberYaw] +=
            pose_update_vec[StateMemberYaw];
        }
      } else {
        std::cerr << "Warning: " << pose_topic <<
          " is listed as an input topic, "
          "but all pose update variables are false\n";
      }

      RF_DEBUG(
        "Subscribed to " <<
          pose_topic << " (" << pose_topic_name << ")\n\t" <<
          pose_topic_name << "_differential is " <<
        (differential ? "true" : "false") << "\n\t" << pose_topic_name <<
          "_rejection_threshold is " << pose_mahalanobis_thresh <<
          "\n\t" << pose_topic_name << " update vector is " <<
          pose_update_vec);
    }
  } while (more_params);

  // Repeat for twist
  topic_ind = 0;
  more_params = false;
  do {
    std::stringstream ss;
    ss << "twist" << topic_ind++;
    std::string twist_topic_name = ss.str();
    std::string twist_topic;
    this->declare_parameter(twist_topic_name, rclcpp::PARAMETER_STRING);

    rclcpp::Parameter parameter;
    if (this->get_parameter(twist_topic_name, parameter)) {
      more_params = true;
      twist_topic = parameter.as_string();
    } else {
      more_params = false;
    }

    if (more_params) {
      // Check for twist rejection threshold
      double twist_mahalanobis_thresh = this->declare_parameter(
        twist_topic_name +
        std::string("_rejection_threshold"),
        std::numeric_limits<double>::max());

      // Set optional custom queue size
      int queue_size = this->declare_parameter(
        twist_topic_name +
        std::string("_queue_size"), 10);

      // Pull in the sensor's config, zero out values that are invalid for the
      // twist type
      std::vector<bool> twist_update_vec = loadUpdateConfig(twist_topic_name);
      std::fill(
        twist_update_vec.begin() + POSITION_OFFSET,
        twist_update_vec.begin() + POSITION_OFFSET + POSE_SIZE, 0);

      int twist_update_sum =
        std::accumulate(twist_update_vec.begin(), twist_update_vec.end(), 0);

      if (twist_update_sum > 0) {
        const CallbackData callback_data(twist_topic_name, twist_update_vec,
          twist_update_sum, false, false,
          twist_mahalanobis_thresh);

        std::function<void(const std::shared_ptr<
            geometry_msgs::msg::TwistWithCovarianceStamped>)>
        twist_callback = std::bind(
          &RosFilter<T>::twistCallback, this,
          std::placeholders::_1, callback_data,
          base_link_frame_id_);

        auto custom_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(queue_size));
        topic_subs_.push_back(
          this->create_subscription<
            geometry_msgs::msg::TwistWithCovarianceStamped>(
            twist_topic, custom_qos, twist_callback));

        twist_var_counts[StateMemberVx] += twist_update_vec[StateMemberVx];
        twist_var_counts[StateMemberVy] += twist_update_vec[StateMemberVy];
        twist_var_counts[StateMemberVz] += twist_update_vec[StateMemberVz];
        twist_var_counts[StateMemberVroll] +=
          twist_update_vec[StateMemberVroll];
        twist_var_counts[StateMemberVpitch] +=
          twist_update_vec[StateMemberVpitch];
        twist_var_counts[StateMemberVyaw] += twist_update_vec[StateMemberVyaw];
      } else {
        std::cerr << "Warning: " << twist_topic <<
          " is listed as an input topic, "
          "but all twist update variables are false\n";
      }

      RF_DEBUG(
        "Subscribed to " <<
          twist_topic << " (" << twist_topic_name << ")\n\t" <<
          twist_topic_name << "_rejection_threshold is " <<
          twist_mahalanobis_thresh << "\n\t" << twist_topic_name <<
          " update vector is " << twist_update_vec);
    }
  } while (more_params);

  // Repeat for IMU
  topic_ind = 0;
  more_params = false;
  do {
    std::stringstream ss;
    ss << "imu" << topic_ind++;
    std::string imu_topic_name = ss.str();
    std::string imu_topic;
    this->declare_parameter(imu_topic_name, rclcpp::PARAMETER_STRING);

    rclcpp::Parameter parameter;
    if (this->get_parameter(imu_topic_name, parameter)) {
      more_params = true;
      imu_topic = parameter.as_string();
    } else {
      more_params = false;
    }

    if (more_params) {
      bool differential = this->declare_parameter(
        imu_topic_name + std::string("_differential"),
        false);

      // Determine if we want to integrate this sensor relatively
      bool relative = this->declare_parameter(imu_topic_name + std::string("_relative"), false);

      if (relative && differential) {
        std::cerr << "Both " << imu_topic_name << "_differential" <<
          " and " << imu_topic_name <<
          "_relative were set to true. Using differential mode.\n";

        relative = false;
      }

      // Check for pose rejection threshold
      double pose_mahalanobis_thresh = this->declare_parameter(
        imu_topic_name +
        std::string("_pose_rejection_threshold"),
        std::numeric_limits<double>::max());

      // Check for angular velocity rejection threshold
      std::string imu_twist_rejection_name =
        imu_topic_name + std::string("_twist_rejection_threshold");
      double twist_mahalanobis_thresh = this->declare_parameter(
        imu_twist_rejection_name,
        std::numeric_limits<double>::max());

      // Check for acceleration rejection threshold
      double accel_mahalanobis_thresh = this->declare_parameter(
        imu_topic_name +
        std::string("_linear_acceleration_rejection_threshold"),
        std::numeric_limits<double>::max());

      bool remove_grav_acc = this->declare_parameter(
        imu_topic_name +
        "_remove_gravitational_acceleration",
        false);
      remove_gravitational_acceleration_[imu_topic_name + "_acceleration"] =
        remove_grav_acc;

      // Set optional custom queue size
      int queue_size = this->declare_parameter(
        imu_topic_name +
        std::string("_queue_size"), 10);

      // Now pull in its boolean update vector configuration and differential
      // update configuration (as this contains pose information)
      std::vector<bool> update_vec = loadUpdateConfig(imu_topic_name);

      // sanity checks for update config settings
      std::vector<int> position_update_vec(update_vec.begin() + POSITION_OFFSET,
        update_vec.begin() + POSITION_OFFSET + POSITION_SIZE);
      int position_update_sum = std::accumulate(
        position_update_vec.begin(),
        position_update_vec.end(), 0);
      if (position_update_sum > 0) {
        RCLCPP_WARN(
          this->get_logger(),
          "Warning: Some position entries in parameter %s_config are listed "
          "true, but sensor_msgs/Imu contains no information about position",
          imu_topic_name.c_str());
      }
      std::vector<int> linear_velocity_update_vec(
        update_vec.begin() + POSITION_V_OFFSET,
        update_vec.begin() + POSITION_V_OFFSET + LINEAR_VELOCITY_SIZE);
      int linear_velocity_update_sum = std::accumulate(
        linear_velocity_update_vec.begin(), linear_velocity_update_vec.end(),
        0);
      if (linear_velocity_update_sum > 0) {
        RCLCPP_WARN(
          this->get_logger(),
          "Warning: Some linear velocity entries in parameter %s_config are "
          "listed true, but an sensor_msgs/Imu contains no information about "
          "linear velocities", imu_topic_name.c_str());
      }

      std::vector<bool> pose_update_vec = update_vec;
      // IMU message contains no information about position, filter everything
      // except orientation
      std::fill(
        pose_update_vec.begin() + POSITION_OFFSET,
        pose_update_vec.begin() + POSITION_OFFSET + POSITION_SIZE, 0);
      std::fill(
        pose_update_vec.begin() + POSITION_V_OFFSET,
        pose_update_vec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);
      std::fill(
        pose_update_vec.begin() + POSITION_A_OFFSET,
        pose_update_vec.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE, 0);

      std::vector<bool> twist_update_vec = update_vec;
      // IMU message contains no information about linear speeds, filter
      // everything except angular velocity
      std::fill(
        twist_update_vec.begin() + POSITION_OFFSET,
        twist_update_vec.begin() + POSITION_OFFSET + POSE_SIZE, 0);
      std::fill(
        twist_update_vec.begin() + POSITION_V_OFFSET,
        twist_update_vec.begin() + POSITION_V_OFFSET + LINEAR_VELOCITY_SIZE, 0);
      std::fill(
        twist_update_vec.begin() + POSITION_A_OFFSET,
        twist_update_vec.begin() + POSITION_A_OFFSET + ACCELERATION_SIZE, 0);

      std::vector<bool> accel_update_vec = update_vec;
      std::fill(
        accel_update_vec.begin() + POSITION_OFFSET,
        accel_update_vec.begin() + POSITION_OFFSET + POSE_SIZE, 0);
      std::fill(
        accel_update_vec.begin() + POSITION_V_OFFSET,
        accel_update_vec.begin() + POSITION_V_OFFSET + TWIST_SIZE, 0);

      int pose_update_sum =
        std::accumulate(pose_update_vec.begin(), pose_update_vec.end(), 0);
      int twist_update_sum =
        std::accumulate(twist_update_vec.begin(), twist_update_vec.end(), 0);
      int accelUpdateSum =
        std::accumulate(accel_update_vec.begin(), accel_update_vec.end(), 0);

      // Check if we're using control input for any of the acceleration
      // variables; turn off if so
      if (control_update_vector[ControlMemberVx] &&
        static_cast<bool>(accel_update_vec[StateMemberAx]))
      {
        std::cerr << "X acceleration is being measured from IMU; X velocity "
          "control input is disabled\n";
        control_update_vector[ControlMemberVx] = 0;
      }
      if (control_update_vector[ControlMemberVy] &&
        static_cast<bool>(accel_update_vec[StateMemberAy]))
      {
        std::cerr << "Y acceleration is being measured from IMU; Y velocity "
          "control input is disabled\n";
        control_update_vector[ControlMemberVy] = 0;
      }
      if (control_update_vector[ControlMemberVz] &&
        static_cast<bool>(accel_update_vec[StateMemberAz]))
      {
        std::cerr << "Z acceleration is being measured from IMU; Z velocity "
          "control input is disabled\n";
        control_update_vector[ControlMemberVz] = 0;
      }

      if (pose_update_sum + twist_update_sum + accelUpdateSum > 0) {
        const CallbackData pose_callback_data(
          imu_topic_name + "_pose", pose_update_vec, pose_update_sum,
          differential, relative, pose_mahalanobis_thresh);
        const CallbackData twist_callback_data(
          imu_topic_name + "_twist", twist_update_vec, twist_update_sum,
          differential, relative, twist_mahalanobis_thresh);
        const CallbackData accel_callback_data(
          imu_topic_name + "_acceleration", accel_update_vec, accelUpdateSum,
          differential, relative, accel_mahalanobis_thresh);

        std::function<void(const std::shared_ptr<sensor_msgs::msg::Imu>)>
        imu_callback =
          std::bind(
          &RosFilter<T>::imuCallback, this, std::placeholders::_1,
          imu_topic_name, pose_callback_data,
          twist_callback_data, accel_callback_data);

        auto custom_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(queue_size));
        topic_subs_.push_back(
          this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, custom_qos, imu_callback));
      } else {
        std::cerr << "Warning: " << imu_topic <<
          " is listed as an input topic, "
          "but all its update variables are false\n";
      }

      if (pose_update_sum > 0) {
        if (differential) {
          twist_var_counts[StateMemberVroll] +=
            pose_update_vec[StateMemberRoll];
          twist_var_counts[StateMemberVpitch] +=
            pose_update_vec[StateMemberPitch];
          twist_var_counts[StateMemberVyaw] += pose_update_vec[StateMemberYaw];
        } else {
          abs_pose_var_counts[StateMemberRoll] +=
            pose_update_vec[StateMemberRoll];
          abs_pose_var_counts[StateMemberPitch] +=
            pose_update_vec[StateMemberPitch];
          abs_pose_var_counts[StateMemberYaw] +=
            pose_update_vec[StateMemberYaw];
        }
      }

      if (twist_update_sum > 0) {
        twist_var_counts[StateMemberVroll] +=
          twist_update_vec[StateMemberVroll];
        twist_var_counts[StateMemberVpitch] +=
          twist_update_vec[StateMemberVpitch];
        twist_var_counts[StateMemberVyaw] += twist_update_vec[StateMemberVyaw];
      }

      RF_DEBUG(
        "Subscribed to " <<
          imu_topic << " (" << imu_topic_name << ")\n\t" <<
          imu_topic_name << "_differential is " <<
        (differential ? "true" : "false") << "\n\t" << imu_topic_name <<
          "_pose_rejection_threshold is " << pose_mahalanobis_thresh <<
          "\n\t" << imu_topic_name << "_twist_rejection_threshold is " <<
          twist_mahalanobis_thresh << "\n\t" << imu_topic_name <<
          "_linear_acceleration_rejection_threshold is " <<
          accel_mahalanobis_thresh << "\n\t" << imu_topic_name <<
          "_remove_gravitational_acceleration is " <<
        (remove_grav_acc ? "true" : "false") << "\n\t" <<
          imu_topic_name << " pose update vector is " << pose_update_vec <<
          "\t" << imu_topic_name << " twist update vector is " <<
          twist_update_vec << "\t" << imu_topic_name <<
          " acceleration update vector is " << accel_update_vec);
    }
  } while (more_params);

  // Now that we've checked if IMU linear acceleration is being used, we can
  // determine our final control parameters
  if (use_control_ && std::accumulate(
      control_update_vector.begin(),
      control_update_vector.end(), 0) == 0)
  {
    std::cerr << "use_control is set to true, but control_config has only "
      "false values. No control term "
      "will be used.\n";
    use_control_ = false;
  }

  // If we're using control, set the parameters and create the necessary
  // subscribers
  if (use_control_) {
    latest_control_.resize(TWIST_SIZE);
    latest_control_.setZero();

    filter_.setControlParams(
      control_update_vector,
      rclcpp::Duration::from_seconds(control_timeout),
      acceleration_limits, acceleration_gains, deceleration_limits,
      deceleration_gains);

    control_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(1),
      std::bind(&RosFilter<T>::controlCallback, this, std::placeholders::_1));
  }

  /* Warn users about:
   *    1. Multiple non-differential input sources
   *    2. No absolute *or* velocity measurements for pose variables
   */
  if (print_diagnostics_) {
    for (int state_var = StateMemberX; state_var <= StateMemberYaw; ++state_var) {
      if (abs_pose_var_counts[static_cast<StateMembers>(state_var)] > 1) {
        std::stringstream stream;
        stream << abs_pose_var_counts[static_cast<StateMembers>(state_var -
          POSITION_OFFSET)] << " absolute pose inputs detected for " <<
          state_variable_names_[state_var] <<
          ". This may result in oscillations. Please ensure that your"
          "variances for each measured variable are set appropriately.";

        addDiagnostic(
          diagnostic_msgs::msg::DiagnosticStatus::WARN,
          state_variable_names_[state_var] + "_configuration",
          stream.str(), true);
      } else if (abs_pose_var_counts[static_cast<StateMembers>(state_var)] == 0) {
        if ((static_cast<StateMembers>(state_var) == StateMemberX &&
          twist_var_counts[static_cast<StateMembers>(StateMemberVx)] == 0) ||
          (static_cast<StateMembers>(state_var) == StateMemberY &&
          twist_var_counts[static_cast<StateMembers>(StateMemberVy)] == 0) ||
          (static_cast<StateMembers>(state_var) == StateMemberZ &&
          twist_var_counts[static_cast<StateMembers>(StateMemberVz)] == 0 &&
          two_d_mode_ == false) ||
          (static_cast<StateMembers>(state_var) == StateMemberRoll &&
          twist_var_counts[static_cast<StateMembers>(StateMemberVroll)] == 0 &&
          two_d_mode_ == false) || (static_cast<StateMembers>(state_var) ==
          StateMemberPitch &&
          twist_var_counts[static_cast<StateMembers>(StateMemberVpitch)] == 0 &&
          two_d_mode_ == false) || (static_cast<StateMembers>(state_var) ==
          StateMemberYaw && twist_var_counts[static_cast<StateMembers>(StateMemberVyaw)] ==
          0))
        {
          std::stringstream stream;
          stream << "Neither " << state_variable_names_[state_var] << " nor its "
            "velocity is being measured. This will result in unbounded"
            "error growth and erratic filter behavior.";

          addDiagnostic(
            diagnostic_msgs::msg::DiagnosticStatus::ERROR,
            state_variable_names_[state_var] + "_configuration",
            stream.str(), true);
        }
      }
    }
  }

  // Load up the process noise covariance (from the launch file/parameter
  // server)
  Eigen::MatrixXd process_noise_covariance(STATE_SIZE, STATE_SIZE);
  process_noise_covariance.setZero();
  std::vector<double> process_noise_covar_flat;

  this->declare_parameter("process_noise_covariance", rclcpp::PARAMETER_DOUBLE_ARRAY);
  if (this->get_parameter(
      "process_noise_covariance",
      process_noise_covar_flat))
  {
    assert(process_noise_covar_flat.size() == STATE_SIZE * STATE_SIZE);

    for (int i = 0; i < STATE_SIZE; i++) {
      for (int j = 0; j < STATE_SIZE; j++) {
        process_noise_covariance(i, j) =
          process_noise_covar_flat[i * STATE_SIZE + j];
      }
    }

    RF_DEBUG(
      "Process noise covariance is:\n" <<
        process_noise_covariance << "\n");

    filter_.setProcessNoiseCovariance(process_noise_covariance);
  }

  // Load up the process noise covariance (from the launch file/parameter
  // server)
  Eigen::MatrixXd initial_estimate_error_covariance(STATE_SIZE, STATE_SIZE);
  initial_estimate_error_covariance.setZero();
  std::vector<double> estimate_error_covar_flat;

  this->declare_parameter("initial_estimate_covariance", rclcpp::PARAMETER_DOUBLE_ARRAY);
  if (this->get_parameter(
      "initial_estimate_covariance",
      estimate_error_covar_flat))
  {
    assert(estimate_error_covar_flat.size() == STATE_SIZE * STATE_SIZE);

    for (int i = 0; i < STATE_SIZE; i++) {
      for (int j = 0; j < STATE_SIZE; j++) {
        initial_estimate_error_covariance(i, j) =
          estimate_error_covar_flat[i * STATE_SIZE + j];
      }
    }

    RF_DEBUG(
      "Initial estimate error covariance is:\n" <<
        estimate_error_covar_flat << "\n");

    filter_.setEstimateErrorCovariance(initial_estimate_error_covariance);
  }
}

template<typename T>
void RosFilter<T>::odometryCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg,
  const std::string & topic_name,
  const CallbackData & pose_callback_data,
  const CallbackData & twist_callback_data)
{
  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (last_set_pose_time_ >= msg->header.stamp) {
    std::stringstream stream;
    stream <<
      "The " << topic_name <<
      " message has a timestamp equal to or before the last filter reset, " <<
      "this message will be ignored. This may indicate an empty or bad "
      "timestamp. (message time: " <<
      filter_utilities::toSec(msg->header.stamp) << ")";
    addDiagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      topic_name + "_timestamp", stream.str(), false);
    RF_DEBUG(
      "Received message that preceded the most recent pose reset. "
      "Ignoring...");

    return;
  }

  RF_DEBUG(
    "------ RosFilter<T>::odometryCallback (" <<
      topic_name << ") ------\n")         // << "Odometry message:\n" << *msg);

  if (pose_callback_data.update_sum_ > 0) {
    // Grab the pose portion of the message and pass it to the poseCallback
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pos_ptr =
      std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    pos_ptr->header = msg->header;
    pos_ptr->pose = msg->pose;  // Entire pose object, also copies covariance

    poseCallback(pos_ptr, pose_callback_data, world_frame_id_, false);
  }

  if (twist_callback_data.update_sum_ > 0) {
    // Grab the twist portion of the message and pass it to the twistCallback
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist_ptr =
      std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
    twist_ptr->header = msg->header;
    twist_ptr->header.frame_id = msg->child_frame_id;
    twist_ptr->twist =
      msg->twist;   // Entire twist object, also copies covariance

    twistCallback(twist_ptr, twist_callback_data, base_link_frame_id_);
  }

  RF_DEBUG(
    "\n----- /RosFilter<T>::odometryCallback (" << topic_name <<
      ") ------\n");
}

template<typename T>
void RosFilter<T>::poseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg,
  const CallbackData & callback_data, const std::string & target_frame,
  const bool imu_data)
{
  const std::string & topic_name = callback_data.topic_name_;

  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (last_set_pose_time_ >= msg->header.stamp) {
    std::stringstream stream;
    stream <<
      "The " << topic_name <<
      " message has a timestamp equal to or before the last filter reset, " <<
      "this message will be ignored. This may indicate an empty or bad "
      "timestamp. (message time: " <<
      filter_utilities::toSec(msg->header.stamp) << ")";
    addDiagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      topic_name + "_timestamp", stream.str(), false);
    return;
  }

  RF_DEBUG(
    "------ RosFilter<T>::poseCallback (" << topic_name << ") ------\n"
      "Pose message:\n" << msg);

  //  Put the initial value in the lastMessagTimes_ for this variable if it's
  //  empty
  if (last_message_times_.count(topic_name) == 0) {
    last_message_times_.insert(
      std::pair<std::string, rclcpp::Time>(topic_name, msg->header.stamp));
  }

  // Make sure this message is newer than the last one
  if (last_message_times_[topic_name] <= msg->header.stamp) {
    RF_DEBUG(
      "Update vector for " << topic_name << " is:\n" <<
        callback_data.update_vector_);

    Eigen::VectorXd measurement(STATE_SIZE);
    Eigen::MatrixXd measurement_covariance(STATE_SIZE, STATE_SIZE);

    measurement.setZero();
    measurement_covariance.setZero();

    // Make sure we're actually updating at least one of these variables
    std::vector<bool> update_vector_corrected = callback_data.update_vector_;

    // Prepare the pose data for inclusion in the filter
    if (preparePose(
        msg, topic_name, target_frame, callback_data.differential_,
        callback_data.relative_, imu_data, update_vector_corrected,
        measurement, measurement_covariance))
    {
      // Store the measurement. Add a "pose" suffix so we know what kind of
      // measurement we're dealing with when we debug the core filter logic.
      enqueueMeasurement(
        topic_name, measurement, measurement_covariance,
        update_vector_corrected,
        callback_data.rejection_threshold_, msg->header.stamp);

      RF_DEBUG("Enqueued new measurement for " << topic_name << "\n");
    } else {
      RF_DEBUG("Did *not* enqueue measurement for " << topic_name << "\n");
    }

    last_message_times_[topic_name] = msg->header.stamp;

    RF_DEBUG(
      "Last message time for " <<
        topic_name << " is now " <<
        filter_utilities::toSec(last_message_times_[topic_name]) <<
        "\n");
  } else {
    // else if (reset_on_time_jump_ && rclcpp::Time::isSimTime())
    //{
    //  reset();
    // }

    std::stringstream stream;
    stream << "The " << topic_name << " message has a timestamp before that of "
      "the previous message received," << " this message will be ignored. This may "
      "indicate a bad timestamp. (message time: " << msg->header.stamp.nanosec <<
      ")";
    addDiagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      topic_name + "_timestamp", stream.str(), false);

    RF_DEBUG(
      "Message is too old. Last message time for " << topic_name << " is " <<
        filter_utilities::toSec(last_message_times_[topic_name]) <<
        ", current message time is " << filter_utilities::toSec(msg->header.stamp) <<
        ".\n");
  }

  RF_DEBUG("\n----- /RosFilter<T>::poseCallback (" << topic_name << ") ------\n");
}

template<typename T>
void RosFilter<T>::initialize()
{
  diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(
    shared_from_this());
  diagnostic_updater_->setHardwareID("none");

  world_transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(
    shared_from_this());

  loadParams();

  if (print_diagnostics_) {
    diagnostic_updater_->add(
      "Filter diagnostic updater", this,
      &RosFilter<T>::aggregateDiagnostics);
  }

  // Set up the frequency diagnostic
  min_frequency_ = frequency_ - 2;
  max_frequency_ = frequency_ + 2;
  freq_diag_ =
    std::make_unique<diagnostic_updater::HeaderlessTopicDiagnostic>(
    "odometry/filtered",
    *diagnostic_updater_,
    diagnostic_updater::FrequencyStatusParam(
      &min_frequency_,
      &max_frequency_, 0.1, 10));

  last_diag_time_ = this->now();

  // Clear out the transforms
  world_base_link_trans_msg_.transform =
    tf2::toMsg(tf2::Transform::getIdentity());

  // Position publisher
  position_pub_ =
    this->create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", rclcpp::QoS(10));

  // Optional acceleration publisher
  if (publish_acceleration_) {
    accel_pub_ =
      this->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      "accel/filtered", rclcpp::QoS(10));
  }

  const std::chrono::duration<double> timespan{1.0 / frequency_};
  timer_ = rclcpp::GenericTimer<rclcpp::VoidCallbackType>::make_shared(
    this->get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(timespan),
    std::bind(&RosFilter<T>::periodicUpdate, this), this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

template<typename T>
void RosFilter<T>::periodicUpdate()
{
  // Wait for the filter to be enabled
  if (!enabled_) {
    RCLCPP_INFO_ONCE(
      this->get_logger(),
      "Filter is disabled. To enable it call the %s service",
      enable_filter_srv_->get_service_name());
    return;
  }

  rclcpp::Time cur_time = this->now();

  if (toggled_on_) {
    // Now we'll integrate any measurements we've received
    integrateMeasurements(cur_time);
  } else {
    // Clear out measurements since we're not currently processing new entries
    clearMeasurementQueue();

    // Reset last measurement time so we don't get a large time delta on toggle
    if (filter_.getInitializedStatus()) {
      filter_.setLastMeasurementTime(this->now());
    }
  }

  // Get latest state and publish it
  auto filtered_position = std::make_unique<nav_msgs::msg::Odometry>();

  bool corrected_data = false;

  if (getFilteredOdometryMessage(filtered_position.get())) {
    world_base_link_trans_msg_.header.stamp =
      static_cast<rclcpp::Time>(filtered_position->header.stamp) + tf_time_offset_;
    world_base_link_trans_msg_.header.frame_id =
      filtered_position->header.frame_id;
    world_base_link_trans_msg_.child_frame_id =
      filtered_position->child_frame_id;

    world_base_link_trans_msg_.transform.translation.x =
      filtered_position->pose.pose.position.x;
    world_base_link_trans_msg_.transform.translation.y =
      filtered_position->pose.pose.position.y;
    world_base_link_trans_msg_.transform.translation.z =
      filtered_position->pose.pose.position.z;
    world_base_link_trans_msg_.transform.rotation =
      filtered_position->pose.pose.orientation;

    // The filtered_position is the message containing the state and covariances:
    // nav_msgs Odometry
    if (!validateFilterOutput(filtered_position.get())) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Critical Error, NaNs were detected in the output state of the filter. "
        "This was likely due to poorly coniditioned process, noise, or sensor "
        "covariances.");
    }

    // If we're trying to publish with the same time stamp, it means that we had a measurement get
    // inserted into the filter history, and our state estimate was updated after it was already
    // published. As of ROS Noetic, TF2 will issue warnings whenever this occurs, so we make this
    // behavior optional. Just for safety, we also check for the condition where the last published
    // stamp is *later* than this stamp. This should never happen, but we should handle the case
    // anyway.
    corrected_data = (!permit_corrected_publication_ &&
      last_published_stamp_ >= filtered_position->header.stamp);

    // If the world_frame_id_ is the odom_frame_id_ frame, then we can just
    // send the transform. If the world_frame_id_ is the map_frame_id_ frame,
    // we'll have some work to do.
    if (publish_transform_ && !corrected_data) {
      if (filtered_position->header.frame_id == odom_frame_id_) {
        world_transform_broadcaster_->sendTransform(world_base_link_trans_msg_);
      } else if (filtered_position->header.frame_id == map_frame_id_) {
        try {
          tf2::Transform world_base_link_trans;
          tf2::fromMsg(
            world_base_link_trans_msg_.transform,
            world_base_link_trans);

          tf2::Transform base_link_odom_trans;
          tf2::fromMsg(
            tf_buffer_
            ->lookupTransform(
              base_link_frame_id_,
              odom_frame_id_,
              tf2::TimePointZero)
            .transform,
            base_link_odom_trans);

          /*
           * First, see these two references:
           * http://wiki.ros.org/tf/Overview/Using%20Published%20Transforms#lookupTransform
           * http://wiki.ros.org/geometry/CoordinateFrameConventions#Transform_Direction
           * We have a transform from map_frame_id_->base_link_frame_id_, but
           * it would actually transform a given pose from
           * base_link_frame_id_->map_frame_id_. We then used lookupTransform,
           * whose first two arguments are target frame and source frame, to
           * get a transform from base_link_frame_id_->odom_frame_id_.
           * However, this transform would actually transform data from
           * odom_frame_id_->base_link_frame_id_. Now imagine that we have a
           * position in the map_frame_id_ frame. First, we multiply it by the
           * inverse of the map_frame_id_->baseLinkFrameId, which will
           * transform that data from map_frame_id_ to base_link_frame_id_.
           * Now we want to go from base_link_frame_id_->odom_frame_id_, but
           * the transform we have takes data from
           * odom_frame_id_->base_link_frame_id_, so we need its inverse as
           * well. We have now transformed our data from map_frame_id_ to
           * odom_frame_id_. However, if we want other users to be able to do
           * the same, we need to broadcast the inverse of that entire
           * transform.
           */
          tf2::Transform map_odom_trans;
          map_odom_trans.mult(world_base_link_trans, base_link_odom_trans);

          geometry_msgs::msg::TransformStamped map_odom_trans_msg;
          map_odom_trans_msg.transform = tf2::toMsg(map_odom_trans);
          map_odom_trans_msg.header.stamp =
            static_cast<rclcpp::Time>(filtered_position->header.stamp) + tf_time_offset_;
          map_odom_trans_msg.header.frame_id = map_frame_id_;
          map_odom_trans_msg.child_frame_id = odom_frame_id_;

          world_transform_broadcaster_->sendTransform(map_odom_trans_msg);
        } catch (...) {
          // ROS_ERROR_STREAM_DELAYED_THROTTLE(5.0, "Could not obtain
          // transform from "
          //                                  << odom_frame_id_ << "->" <<
          //                                  base_link_frame_id_);
        }
      } else {
        std::cerr << "Odometry message frame_id was " <<
          filtered_position->header.frame_id << ", expected " <<
          map_frame_id_ << " or " << odom_frame_id_ << "\n";
      }
    }

    // Retain the last published stamp so we can detect repeated transforms in future cycles
    last_published_stamp_ = filtered_position->header.stamp;

    // Fire off the position and the transform
    if (!corrected_data) {
      position_pub_->publish(std::move(filtered_position));
    }

    if (print_diagnostics_) {
      freq_diag_->tick();
    }
  }

  // Publish the acceleration if desired and filter is initialized
  auto filtered_acceleration = std::make_unique<geometry_msgs::msg::AccelWithCovarianceStamped>();
  if (!corrected_data && publish_acceleration_ &&
    getFilteredAccelMessage(filtered_acceleration.get()))
  {
    accel_pub_->publish(std::move(filtered_acceleration));
  }

  /* Diagnostics can behave strangely when playing back from bag
   * files and using simulated time, so we have to check for
   * time suddenly moving backwards as well as the standard
   * timeout criterion before publishing. */

  double diag_duration = (cur_time - last_diag_time_).nanoseconds();
  if (print_diagnostics_ &&
    (diag_duration >= diagnostic_updater_->getPeriod().nanoseconds() ||
    diag_duration < 0.0))
  {
    diagnostic_updater_->force_update();
    last_diag_time_ = cur_time;
  }

  // Clear out expired history data
  if (smooth_lagged_data_) {
    clearExpiredHistory(filter_.getLastMeasurementTime() - history_length_);
  }

  // Warn the user if the update took too long
  const double loop_elapsed = (this->now() - cur_time).seconds();
  if (loop_elapsed > 1. / frequency_) {
    std::cerr <<
      "Failed to meet update rate! Took " << std::setprecision(20) <<
      loop_elapsed << "seconds. Try decreasing the rate, limiting "
      "sensor output frequency, or limiting the number of sensors.\n";
  }
}

template<typename T>
void RosFilter<T>::setPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RF_DEBUG(
    "------ RosFilter<T>::setPoseCallback ------\nPose message:\n" << msg);

  // ROS_INFO_STREAM("Received set_pose request with value\n" << *msg);

  std::string topic_name("set_pose");

  // Get rid of any initial poses (pretend we've never had a measurement)
  initial_measurements_.clear();
  previous_measurements_.clear();
  previous_measurement_covariances_.clear();

  clearMeasurementQueue();

  filter_state_history_.clear();
  measurement_history_.clear();

  // Also set the last set pose time, so we ignore all messages
  // that occur before it
  last_set_pose_time_ = msg->header.stamp;

  // Set the state vector to the reported pose
  Eigen::VectorXd measurement(STATE_SIZE);
  Eigen::MatrixXd measurement_covariance(STATE_SIZE, STATE_SIZE);
  std::vector<bool> update_vector(STATE_SIZE, true);

  // We only measure pose variables, so initialize the vector to 0
  measurement.setZero();

  // Set this to the identity and let the message reset it
  measurement_covariance.setIdentity();
  measurement_covariance *= 1e-6;

  // Prepare the pose data (really just using this to transform it into the
  // target frame). Twist data is going to get zeroed out.
  preparePose(
    msg, topic_name, world_frame_id_, false, false, false,
    update_vector, measurement, measurement_covariance);

  // For the state
  filter_.setState(measurement);
  filter_.setEstimateErrorCovariance(measurement_covariance);

  filter_.setLastMeasurementTime(this->now());

  RF_DEBUG("\n------ /RosFilter<T>::setPoseCallback ------\n");
}

template<typename T>
bool RosFilter<T>::setPoseSrvCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<robot_localization::srv::SetPose::Request> request,
  std::shared_ptr<robot_localization::srv::SetPose::Response>/*response*/)
{
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg =
    std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(
    request->pose);
  setPoseCallback(msg);

  return true;
}

template<typename T>
bool RosFilter<T>::enableFilterSrvCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<std_srvs::srv::Empty::Request>,
  const std::shared_ptr<std_srvs::srv::Empty::Response>)
{
  RF_DEBUG(
    "\n[" << this->get_name() << ":]" <<
      " ------ /RosFilter::enableFilterSrvCallback ------\n");
  if (enabled_) {
    RCLCPP_WARN(
      this->get_logger(), "[%s:] Asking for enabling filter service, "
      "but the filter was already enabled! Use param disabled_at_startup.",
      this->get_name());
  } else {
    RCLCPP_INFO(
      this->get_logger(), "[%s:] Enabling filter...",
      this->get_name());
    enabled_ = true;
  }

  return true;
}

template<typename T>
void RosFilter<T>::twistCallback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg,
  const CallbackData & callback_data, const std::string & target_frame)
{
  const std::string & topic_name = callback_data.topic_name_;

  // If we've just reset the filter, then we want to ignore any messages
  // that arrive with an older timestamp
  if (last_set_pose_time_ >= msg->header.stamp) {
    std::stringstream stream;
    stream <<
      "The " << topic_name <<
      " message has a timestamp equal to or before the last filter reset, " <<
      "this message will be ignored. This may indicate an empty or bad "
      "timestamp. (message time: " <<
      filter_utilities::toSec(msg->header.stamp) << ")";
    addDiagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      topic_name + "_timestamp", stream.str(), false);
    return;
  }

  RF_DEBUG(
    "------ RosFilter<T>::twistCallback (" << topic_name << ") ------\n"
      "Twist message:\n" << msg);

  if (last_message_times_.count(topic_name) == 0) {
    last_message_times_.insert(
      std::pair<std::string, rclcpp::Time>(topic_name, msg->header.stamp));
  }

  // Make sure this message is newer than the last one
  if (last_message_times_[topic_name] <= msg->header.stamp) {
    RF_DEBUG(
      "Update vector for " << topic_name << " is:\n" <<
        callback_data.update_vector_);

    Eigen::VectorXd measurement(STATE_SIZE);
    Eigen::MatrixXd measurement_covariance(STATE_SIZE, STATE_SIZE);

    measurement.setZero();
    measurement_covariance.setZero();

    // Make sure we're actually updating at least one of these variables
    std::vector<bool> update_vector_corrected = callback_data.update_vector_;

    // Prepare the twist data for inclusion in the filter
    if (prepareTwist(
        msg, topic_name, target_frame, update_vector_corrected,
        measurement, measurement_covariance))
    {
      // Store the measurement. Add a "twist" suffix so we know what kind of
      // measurement we're dealing with when we debug the core filter logic.
      enqueueMeasurement(
        topic_name, measurement, measurement_covariance,
        update_vector_corrected,
        callback_data.rejection_threshold_, msg->header.stamp);

      RF_DEBUG("Enqueued new measurement for " << topic_name << "_twist\n");
    } else {
      RF_DEBUG(
        "Did *not* enqueue measurement for " << topic_name <<
          "_twist\n");
    }

    last_message_times_[topic_name] = msg->header.stamp;

    RF_DEBUG(
      "Last message time for " <<
        topic_name << " is now " <<
        filter_utilities::toSec(last_message_times_[topic_name]) <<
        "\n");
  } else {
    std::stringstream stream;
    stream << "The " << topic_name << " message has a timestamp before that of "
      "the previous message received," << " this message will be ignored. This may "
      "indicate a bad timestamp. (message time: " << msg->header.stamp.nanosec << ")";
    addDiagnostic(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      topic_name + "_timestamp", stream.str(), false);

    RF_DEBUG(
      "Message is too old. Last message time for " << topic_name << " is" <<
        filter_utilities::toSec(last_message_times_[topic_name]) <<
        ", current message time is " << filter_utilities::toSec(msg->header.stamp) <<
        ".\n");
  }

  RF_DEBUG("\n----- /RosFilter<T>::twistCallback (" << topic_name << ") ------\n");
}

template<typename T>
void RosFilter<T>::addDiagnostic(
  const int errLevel,
  const std::string & topicAndClass,
  const std::string & message,
  const bool staticDiag)
{
  if (staticDiag) {
    static_diagnostics_[topicAndClass] = message;
    static_diag_error_level_ = std::max(static_diag_error_level_, errLevel);
  } else {
    dynamic_diagnostics_[topicAndClass] = message;
    dynamic_diag_error_level_ = std::max(dynamic_diag_error_level_, errLevel);
  }
}

template<typename T>
void RosFilter<T>::aggregateDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & wrapper)
{
  wrapper.clear();
  wrapper.clearSummary();

  int maxErrLevel = std::max(static_diag_error_level_, dynamic_diag_error_level_);

  // Report the overall status
  switch (maxErrLevel) {
    case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
      wrapper.summary(
        maxErrLevel,
        "Erroneous data or settings detected for a "
        "robot_localization state estimation this->");
      break;
    case
      diagnostic_msgs::msg::DiagnosticStatus::WARN: wrapper.summary(
        maxErrLevel,
        "Potentially erroneous data or settings detected for "
        "a robot_localization state estimation this->");
      break;
    case diagnostic_msgs::msg::DiagnosticStatus::STALE:
      wrapper.summary(
        maxErrLevel,
        "The state of the robot_localization state estimation "
        "node is stale.");
      break;
    case diagnostic_msgs::msg::DiagnosticStatus::OK:
      wrapper.summary(
        maxErrLevel,
        "The robot_localization state estimation node appears to "
        "be functioning properly.");
      break;
    default:
      break;
  }

  // Aggregate all the static messages
  for (std::map<std::string, std::string>::iterator diagIt =
    static_diagnostics_.begin(); diagIt != static_diagnostics_.end();
    ++diagIt)
  {
    wrapper.add(diagIt->first, diagIt->second);
  }

  // Aggregate all the dynamic messages, then clear them
  for (std::map<std::string, std::string>::iterator diagIt =
    dynamic_diagnostics_.begin(); diagIt != dynamic_diagnostics_.end();
    ++diagIt)
  {
    wrapper.add(diagIt->first, diagIt->second);
  }
  dynamic_diagnostics_.clear();

  // Reset the warning level for the dynamic diagnostic messages
  dynamic_diag_error_level_ = diagnostic_msgs::msg::DiagnosticStatus::OK;
}

template<typename T>
void RosFilter<T>::copyCovariance(
  const double * arr, Eigen::MatrixXd & covariance,
  const std::string & topic_name,
  const std::vector<bool> & update_vector,
  const size_t offset, const size_t dimension)
{
  for (size_t i = 0; i < dimension; i++) {
    for (size_t j = 0; j < dimension; j++) {
      covariance(i, j) = arr[dimension * i + j];

      if (print_diagnostics_) {
        std::string iVar = state_variable_names_[offset + i];

        if (covariance(i, j) > 1e3 && (update_vector[offset + i] ||
          update_vector[offset + j]))
        {
          std::string jVar = state_variable_names_[offset + j];

          std::stringstream stream;
          stream << "The covariance at position (" << dimension * i + j <<
            "), which corresponds to " << (i == j ? iVar + " variance" : iVar + " and " +
          jVar + " covariance") <<
            ", the value is extremely large (" << covariance(i, j) << "), but "
            "the update vector for " << (i == j ? iVar : iVar + " and/or " + jVar) <<
            "is set to true. This may produce undesirable results.";

          addDiagnostic(
            diagnostic_msgs::msg::DiagnosticStatus::WARN,
            topic_name + "_covariance", stream.str(), false);
        } else if (update_vector[i] && i == j && covariance(i, j) == 0) {
          std::stringstream stream;
          stream << "The covariance at position (" << dimension * i + j <<
            "), which corresponds to " << iVar << " variance, was zero. This"
            "will be replaced with a small value to maintain filter stability, "
            "but should be corrected at the message origin this->";

          addDiagnostic(
            diagnostic_msgs::msg::DiagnosticStatus::WARN,
            topic_name + "_covariance", stream.str(), false);
        } else if (update_vector[i] && i == j && covariance(i, j) < 0) {
          std::stringstream stream;
          stream << "The covariance at position (" << dimension * i + j <<
            "), which corresponds to " << iVar << " variance, was"
            "negative. This will be replaced with a small positive value to maintain"
            "filter stability, but should be corrected at the message origin this->";

          addDiagnostic(
            diagnostic_msgs::msg::DiagnosticStatus::WARN,
            topic_name + "_covariance", stream.str(), false);
        }
      }
    }
  }
}

template<typename T>
void RosFilter<T>::copyCovariance(
  const Eigen::MatrixXd & covariance, double * arr,
  const size_t dimension)
{
  for (size_t i = 0; i < dimension; i++) {
    for (size_t j = 0; j < dimension; j++) {
      arr[dimension * i + j] = covariance(i, j);
    }
  }
}

template<typename T>
std::vector<bool> RosFilter<T>::loadUpdateConfig(const std::string & topic_name)
{
  std::vector<bool> update_vector(STATE_SIZE, 0);
  const std::string topic_config_name = topic_name + "_config";

  update_vector = this->declare_parameter(topic_config_name, update_vector);

  return update_vector;
}

template<typename T>
bool RosFilter<T>::prepareAcceleration(
  const sensor_msgs::msg::Imu::SharedPtr msg,
  const std::string & topic_name,
  const std::string & target_frame,
  std::vector<bool> & update_vector,
  Eigen::VectorXd & measurement,
  Eigen::MatrixXd & measurement_covariance)
{
  RF_DEBUG(
    "------ RosFilter<T>::prepareAcceleration (" << topic_name <<
      ") ------\n");

  // 1. Get the measurement into a vector
  tf2::Vector3 acc_tmp(msg->linear_acceleration.x, msg->linear_acceleration.y,
    msg->linear_acceleration.z);

  // Set relevant header info
  std::string msg_frame =
    (msg->header.frame_id == "" ? base_link_frame_id_ : msg->header.frame_id);

  // 2. robot_localization lets users configure which variables from the sensor
  // should be
  //    fused with the filter. This is specified at the sensor level. However,
  //    the data may go through transforms before being fused with the state
  //    estimate. In that case, we need to know which of the transformed
  //    variables came from the pre-transformed "approved" variables (i.e., the
  //    ones that had "true" in their xxx_config parameter). To do this, we
  //    create a pose from the original upate vector, which contains only zeros
  //    and ones. This pose goes through the same transforms as the measurement.
  //    The non-zero values that result will be used to modify the
  //    update_vector.
  tf2::Matrix3x3 maskAcc(update_vector[StateMemberAx], 0, 0, 0,
    update_vector[StateMemberAy], 0, 0, 0,
    update_vector[StateMemberAz]);

  // 3. We'll need to rotate the covariance as well
  Eigen::MatrixXd covariance_rotated(ACCELERATION_SIZE, ACCELERATION_SIZE);
  covariance_rotated.setZero();

  this->copyCovariance(
    &(msg->linear_acceleration_covariance[0]),
    covariance_rotated, topic_name, update_vector,
    POSITION_A_OFFSET, ACCELERATION_SIZE);

  RF_DEBUG(
    "Original measurement as tf object: " <<
      acc_tmp << "\nOriginal update vector:\n" <<
      update_vector << "\nOriginal covariance matrix:\n" <<
      covariance_rotated << "\n");

  // 4. We need to transform this into the target frame (probably base_link)
  // It's unlikely that we'll get a velocity measurement in another frame, but
  // we have to handle the situation.
  tf2::Transform target_frame_trans;
  bool can_transform = ros_filter_utilities::lookupTransformSafe(
    tf_buffer_.get(), target_frame, msg_frame, msg->header.stamp, tf_timeout_,
    target_frame_trans);

  if (can_transform) {
    // We don't know if the user has already handled the removal
    // of normal forces, so we use a parameter
    if (remove_gravitational_acceleration_[topic_name]) {
      tf2::Vector3 normAcc(0, 0, gravitational_acceleration_);
      tf2::Transform trans;

      if (::fabs(msg->orientation_covariance[0] + 1) < 1e-9) {
        // Imu message contains no orientation, so we should use orientation
        // from filter state to transform and remove acceleration
        const Eigen::VectorXd & state = filter_.getState();
        tf2::Matrix3x3 stateTmp;
        stateTmp.setRPY(state(StateMemberRoll),
                        state(StateMemberPitch),
                        state(StateMemberYaw));

        // transform state orientation to IMU frame
        tf2::Transform imuFrameTrans;
        ros_filter_utilities::lookupTransformSafe(
          tf_buffer_.get(), target_frame, msg_frame, msg->header.stamp, tf_timeout_,
          imuFrameTrans);
        trans.setBasis(stateTmp * imuFrameTrans.getBasis());
      } else {
        tf2::Quaternion curAttitude;
        tf2::fromMsg(msg->orientation, curAttitude);
        if (fabs(curAttitude.length() - 1.0) > 0.01) {
          RCLCPP_WARN_ONCE(
            this->get_logger(),
            "An input was not normalized, this should NOT happen, but will normalize.");
          curAttitude.normalize();
        }
        trans.setRotation(curAttitude);
      }
      tf2::Vector3 rotNorm = trans.getBasis().inverse() * normAcc;
      acc_tmp.setX(acc_tmp.getX() - rotNorm.getX());
      acc_tmp.setY(acc_tmp.getY() - rotNorm.getY());
      acc_tmp.setZ(acc_tmp.getZ() - rotNorm.getZ());

      RF_DEBUG(
        "Orientation is " <<
          trans.getRotation() << "Acceleration due to gravity is " << rotNorm <<
          "After removing acceleration due to gravity, acceleration is " <<
          acc_tmp << "\n");
    }

    // Transform to correct frame
    // @todo: This needs to take into account offsets from the origin. Right
    // now, it assumes that if the sensor is placed at some non-zero offset from
    // the vehicle's center, that the vehicle turns with constant velocity. This
    // needs to be something like acc_tmp = target_frame_trans.getBasis() *
    // acc_tmp - target_frame_trans.getOrigin().cross(rotation_acceleration); We
    // can get rotational acceleration by differentiating the rotational
    // velocity (if it's available)
    acc_tmp = target_frame_trans.getBasis() * acc_tmp;
    maskAcc = target_frame_trans.getBasis() * maskAcc;

    // Now use the mask values to determine which update vector values should be
    // true
    update_vector[StateMemberAx] = static_cast<int>(
      maskAcc.getRow(StateMemberAx - POSITION_A_OFFSET).length() >= 1e-6);
    update_vector[StateMemberAy] = static_cast<int>(
      maskAcc.getRow(StateMemberAy - POSITION_A_OFFSET).length() >= 1e-6);
    update_vector[StateMemberAz] = static_cast<int>(
      maskAcc.getRow(StateMemberAz - POSITION_A_OFFSET).length() >= 1e-6);

    RF_DEBUG(
      msg->header.frame_id <<
        "->" << target_frame << " transform:\n" <<
        target_frame_trans << "\nAfter applying transform to " <<
        target_frame << ", update vector is:\n" <<
        update_vector << "\nAfter applying transform to " <<
        target_frame << ", measurement is:\n" <<
        acc_tmp << "\n");

    // 5. Now rotate the covariance: create an augmented
    // matrix that contains a 3D rotation matrix in the
    // upper-left and lower-right quadrants, and zeros
    // elsewhere
    tf2::Matrix3x3 rot(target_frame_trans.getRotation());
    Eigen::MatrixXd rot3d(ACCELERATION_SIZE, ACCELERATION_SIZE);
    rot3d.setIdentity();

    for (size_t r_ind = 0; r_ind < ACCELERATION_SIZE; ++r_ind) {
      rot3d(r_ind, 0) = rot.getRow(r_ind).getX();
      rot3d(r_ind, 1) = rot.getRow(r_ind).getY();
      rot3d(r_ind, 2) = rot.getRow(r_ind).getZ();
    }

    // Carry out the rotation
    covariance_rotated = rot3d * covariance_rotated.eval() * rot3d.transpose();

    RF_DEBUG("Transformed covariance is \n" << covariance_rotated << "\n");

    // 6. Store our corrected measurement and covariance
    measurement(StateMemberAx) = acc_tmp.getX();
    measurement(StateMemberAy) = acc_tmp.getY();
    measurement(StateMemberAz) = acc_tmp.getZ();

    // Copy the covariances
    measurement_covariance.block(
      POSITION_A_OFFSET, POSITION_A_OFFSET,
      ACCELERATION_SIZE, ACCELERATION_SIZE) =
      covariance_rotated.block(0, 0, ACCELERATION_SIZE, ACCELERATION_SIZE);

    // 7. Handle 2D mode
    if (two_d_mode_) {
      forceTwoD(measurement, measurement_covariance, update_vector);
    }
  } else {
    RF_DEBUG(
      "Could not transform measurement into " << target_frame <<
        ". Ignoring...\n");
  }

  RF_DEBUG(
    "\n----- /RosFilter<T>::prepareAcceleration(" << topic_name <<
      ") ------\n");

  return can_transform;
}

template<typename T>
bool RosFilter<T>::preparePose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg,
  const std::string & topic_name, const std::string & target_frame,
  const bool differential, const bool relative, const bool imu_data,
  std::vector<bool> & update_vector, Eigen::VectorXd & measurement,
  Eigen::MatrixXd & measurement_covariance)
{
  bool retVal = false;

  RF_DEBUG("------ RosFilter<T>::preparePose (" << topic_name << ") ------\n");

  // 1. Get the measurement into a tf-friendly transform (pose) object
  tf2::Stamped<tf2::Transform> pose_tmp;

  // We'll need this later for storing this measurement for differential
  // integration
  tf2::Transform cur_measurement;

  // Handle issues where frame_id data is not filled out properly
  // @todo: verify that this is necessary still. New IMU handling may
  // have rendered this obsolete.
  std::string final_target_frame;
  if (target_frame == "" && msg->header.frame_id == "") {
    // Blank target and message frames mean we can just
    // use our world_frame
    final_target_frame = world_frame_id_;
    pose_tmp.frame_id_ = final_target_frame;
  } else if (target_frame == "") {
    // A blank target frame means we shouldn't bother
    // transforming the data
    final_target_frame = msg->header.frame_id;
    pose_tmp.frame_id_ = final_target_frame;
  } else {
    // Otherwise, we should use our target frame
    final_target_frame = target_frame;
    pose_tmp.frame_id_ =
      (differential && !imu_data ? final_target_frame : msg->header.frame_id);
  }

  RF_DEBUG(
    "Final target frame for " << topic_name << " is " <<
      final_target_frame << "\n");

  pose_tmp.stamp_ = tf2::timeFromSec(
    static_cast<double>(msg->header.stamp.sec) +
    static_cast<double>(msg->header.stamp.sec) / 1000000000.0);

  // Fill out the position data
  pose_tmp.setOrigin(
    tf2::Vector3(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z));

  tf2::Quaternion orientation;

  // Handle bad (empty) quaternions
  if (msg->pose.pose.orientation.x == 0 && msg->pose.pose.orientation.y == 0 &&
    msg->pose.pose.orientation.z == 0 && msg->pose.pose.orientation.w == 0)
  {
    orientation.setValue(0.0, 0.0, 0.0, 1.0);

    if (update_vector[StateMemberRoll] ||
      update_vector[StateMemberPitch] ||
      update_vector[StateMemberYaw])
    {
      std::stringstream stream;
      stream << "The " << topic_name <<
        " message contains an invalid orientation quaternion, " <<
        "but its configuration is such that orientation data is being used."
        " Correcting...";

      addDiagnostic(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        topic_name + "_orientation", stream.str(), false);
    }
  } else {
    tf2::fromMsg(msg->pose.pose.orientation, orientation);
    if (fabs(orientation.length() - 1.0) > 0.01) {
      RCLCPP_WARN_ONCE(
        this->get_logger(),
        "An input was not normalized, this should NOT happen, but will normalize.");
      orientation.normalize();
    }
  }

  // Fill out the orientation data
  pose_tmp.setRotation(orientation);

  // 2. Get the target frame transformation
  tf2::Transform target_frame_trans;
  bool can_transform = ros_filter_utilities::lookupTransformSafe(
    tf_buffer_.get(), final_target_frame, pose_tmp.frame_id_,
    rclcpp::Time(tf2::timeToSec(pose_tmp.stamp_)), tf_timeout_,
    target_frame_trans);

  // 3. Make sure we can work with this data before carrying on
  if (can_transform) {
    /* 4. robot_localization lets users configure which variables from the
     * sensor should be fused with the filter. This is specified at the sensor
     * level. However, the data may go through transforms before being fused
     * with the state estimate. In that case, we need to know which of the
     * transformed variables came from the pre-transformed "approved" variables
     * (i.e., the ones that had "true" in their xxx_config parameter). To do
     * this, we construct matrices using the update vector values on the
     * diagonals, pass this matrix through the rotation, and use the length of
     * each row to determine the transformed update vector. The process is
     * slightly different for IMUs, as the coordinate frame transform is really
     * the base_link->imu_frame transform, and not a transform from some other
     * world-fixed frame (even though the IMU data itself *is* reported in a
     * world fixed frame). */
    tf2::Matrix3x3 mask_position(update_vector[StateMemberX], 0, 0, 0,
      update_vector[StateMemberY], 0, 0, 0,
      update_vector[StateMemberZ]);

    tf2::Matrix3x3 mask_orientation(update_vector[StateMemberRoll], 0, 0, 0,
      update_vector[StateMemberPitch], 0, 0, 0,
      update_vector[StateMemberYaw]);

    if (imu_data) {
      /* We have to treat IMU orientation data differently. Even though we are
       * dealing with pose data when we work with orientations, for IMUs, the
       * frame_id is the frame in which the sensor is mounted, and not the
       * coordinate frame of the IMU. Imagine an IMU that is mounted facing
       * sideways. The pitch in the IMU frame becomes roll for the vehicle. This
       * means that we need to rotate roll and pitch angles by the IMU's
       * mounting yaw offset, and we must apply similar treatment to its update
       * mask and covariance.
       * */

      double dummy, yaw;
      target_frame_trans.getBasis().getRPY(dummy, dummy, yaw);
      tf2::Matrix3x3 trans_tmp;
      trans_tmp.setRPY(0.0, 0.0, yaw);

      mask_position = trans_tmp * mask_position;
      mask_orientation = trans_tmp * mask_orientation;
    } else {
      mask_position = target_frame_trans.getBasis() * mask_position;
      mask_orientation = target_frame_trans.getBasis() * mask_orientation;
    }

    // Now copy the mask values back into the update vector: any row with a
    // significant vector length indicates that we want to set that variable to
    // true in the update vector.
    update_vector[StateMemberX] = static_cast<int>(
      mask_position.getRow(StateMemberX - POSITION_OFFSET).length() >= 1e-6);
    update_vector[StateMemberY] = static_cast<int>(
      mask_position.getRow(StateMemberY - POSITION_OFFSET).length() >= 1e-6);
    update_vector[StateMemberZ] = static_cast<int>(
      mask_position.getRow(StateMemberZ - POSITION_OFFSET).length() >= 1e-6);
    update_vector[StateMemberRoll] = static_cast<int>(
      mask_orientation.getRow(StateMemberRoll - ORIENTATION_OFFSET)
      .length() >= 1e-6);
    update_vector[StateMemberPitch] = static_cast<int>(
      mask_orientation.getRow(StateMemberPitch - ORIENTATION_OFFSET)
      .length() >= 1e-6);
    update_vector[StateMemberYaw] = static_cast<int>(
      mask_orientation.getRow(StateMemberYaw - ORIENTATION_OFFSET).length() >=
      1e-6);

    // 5a. We'll need to rotate the covariance as well. Create a container and
    // copy over the covariance data
    Eigen::MatrixXd covariance(POSE_SIZE, POSE_SIZE);
    covariance.setZero();
    copyCovariance(
      &(msg->pose.covariance[0]), covariance, topic_name,
      update_vector, POSITION_OFFSET, POSE_SIZE);

    // 5b. Now rotate the covariance: create an augmented matrix that
    // contains a 3D rotation matrix in the upper-left and lower-right
    // quadrants, with zeros elsewhere.
    tf2::Matrix3x3 rot;
    Eigen::MatrixXd rot6d(POSE_SIZE, POSE_SIZE);
    rot6d.setIdentity();
    Eigen::MatrixXd covariance_rotated;

    if (imu_data) {
      // Apply the same special logic to the IMU covariance rotation
      double dummy, yaw;
      target_frame_trans.getBasis().getRPY(dummy, dummy, yaw);
      rot.setRPY(0.0, 0.0, yaw);
    } else {
      rot.setRotation(target_frame_trans.getRotation());
    }

    for (size_t r_ind = 0; r_ind < POSITION_SIZE; ++r_ind) {
      rot6d(r_ind, 0) = rot.getRow(r_ind).getX();
      rot6d(r_ind, 1) = rot.getRow(r_ind).getY();
      rot6d(r_ind, 2) = rot.getRow(r_ind).getZ();
      rot6d(r_ind + POSITION_SIZE, 3) = rot.getRow(r_ind).getX();
      rot6d(r_ind + POSITION_SIZE, 4) = rot.getRow(r_ind).getY();
      rot6d(r_ind + POSITION_SIZE, 5) = rot.getRow(r_ind).getZ();
    }

    // Now carry out the rotation
    covariance_rotated = rot6d * covariance * rot6d.transpose();

    RF_DEBUG(
      "After rotating into the " << final_target_frame <<
        " frame, covariance is \n" <<
        covariance_rotated << "\n");

    /* 6a. For IMU data, the transform that we get is the transform from the
     * body frame of the robot (e.g., base_link) to the mounting frame of the
     * robot. It is *not* the coordinate frame in which the IMU orientation data
     * is reported. If the IMU is mounted in a non-neutral orientation, we need
     * to remove those offsets, and then we need to potentially "swap" roll and
     * pitch. Note that this transform does NOT handle NED->ENU conversions.
     * Data is assumed to be in the ENU frame when it is received.
     * */
    if (imu_data) {
      // First, convert the transform and measurement rotation to RPY
      // @todo: There must be a way to handle this with quaternions. Need to
      // look into it.
      double rollOffset = 0;
      double pitchOffset = 0;
      double yaw_offset = 0;
      double roll = 0;
      double pitch = 0;
      double yaw = 0;
      ros_filter_utilities::quatToRPY(
        target_frame_trans.getRotation(),
        rollOffset, pitchOffset, yaw_offset);
      ros_filter_utilities::quatToRPY(pose_tmp.getRotation(), roll, pitch, yaw);

      // 6b. Apply the offset (making sure to bound them), and throw them in a
      // vector
      tf2::Vector3 rpy_angles(
        filter_utilities::clampRotation(roll - rollOffset),
        filter_utilities::clampRotation(pitch - pitchOffset),
        filter_utilities::clampRotation(yaw - yaw_offset));

      // 6c. Now we need to rotate the roll and pitch by the yaw offset value.
      // Imagine a case where an IMU is mounted facing sideways. In that case
      // pitch for the IMU's world frame is roll for the robot.
      tf2::Matrix3x3 mat;
      mat.setRPY(0.0, 0.0, yaw_offset);
      rpy_angles = mat * rpy_angles;
      pose_tmp.getBasis().setRPY(
        rpy_angles.getX(), rpy_angles.getY(),
        rpy_angles.getZ());

      // We will use this target transformation later on, but
      // we've already transformed this data as if the IMU
      // were mounted neutrall on the robot, so we can just
      // make the transform the identity.
      target_frame_trans.setIdentity();
    }

    // 7. Two cases: if we're in differential mode, we need to generate a twist
    // message. Otherwise, we just transform it to the target frame.
    if (differential) {
      bool success = false;

      // We're going to be playing with pose_tmp, so store it,
      // as we'll need to save its current value for the next
      // measurement.
      cur_measurement = pose_tmp;

      // Make sure we have previous measurements to work with
      if (previous_measurements_.count(topic_name) > 0 &&
        previous_measurement_covariances_.count(topic_name) > 0)
      {
        // 7a. If we are carrying out differential integration and
        // we have a previous measurement for this sensor,then we
        // need to apply the inverse of that measurement to this new
        // measurement to produce a "delta" measurement between the two.
        // Even if we're not using all of the variables from this sensor,
        // we need to use the whole measurement to determine the delta
        // to the new measurement
        tf2::Transform prev_measurement = previous_measurements_[topic_name];
        pose_tmp.setData(prev_measurement.inverseTimes(pose_tmp));

        RF_DEBUG(
          "Previous measurement:\n" <<
            previous_measurements_[topic_name] <<
            "\nAfter removing previous measurement, measurement delta is:\n" <<
            pose_tmp << "\n");

        // 7b. Now we we have a measurement delta in the frame_id of the
        // message, but we want that delta to be in the target frame, so
        // we need to apply the rotation of the target frame transform.
        target_frame_trans.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
        pose_tmp.mult(target_frame_trans, pose_tmp);

        RF_DEBUG(
          "After rotating to the target frame, measurement delta is:\n" <<
            pose_tmp << "\n");

        // 7c. Now use the time difference from the last message to compute
        // translational and rotational velocities
        double dt = filter_utilities::toSec(msg->header.stamp) -
          filter_utilities::toSec(last_message_times_[topic_name]);
        double xVel = pose_tmp.getOrigin().getX() / dt;
        double yVel = pose_tmp.getOrigin().getY() / dt;
        double zVel = pose_tmp.getOrigin().getZ() / dt;

        double rollVel = 0;
        double pitchVel = 0;
        double yawVel = 0;

        ros_filter_utilities::quatToRPY(
          pose_tmp.getRotation(), rollVel,
          pitchVel, yawVel);
        rollVel /= dt;
        pitchVel /= dt;
        yawVel /= dt;

        RF_DEBUG(
          "Previous message time was " <<
            filter_utilities::toSec(last_message_times_[topic_name]) <<
            ", current message time is " <<
            filter_utilities::toSec(msg->header.stamp) << ", delta is " <<
            dt << ", velocity is (vX, vY, vZ): (" << xVel << ", " <<
            yVel << ", " << zVel << ")\n" <<
            "(vRoll, vPitch, vYaw): (" << rollVel << ", " << pitchVel <<
            ", " << yawVel << ")\n");

        // 7d. Fill out the velocity data in the message
        geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist_ptr =
          std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
        twist_ptr->header = msg->header;
        twist_ptr->header.frame_id = base_link_frame_id_;
        twist_ptr->twist.twist.linear.x = xVel;
        twist_ptr->twist.twist.linear.y = yVel;
        twist_ptr->twist.twist.linear.z = zVel;
        twist_ptr->twist.twist.angular.x = rollVel;
        twist_ptr->twist.twist.angular.y = pitchVel;
        twist_ptr->twist.twist.angular.z = yawVel;
        std::vector<bool> twist_update_vec(STATE_SIZE, false);
        std::copy(
          update_vector.begin() + POSITION_OFFSET,
          update_vector.begin() + POSE_SIZE,
          twist_update_vec.begin() + POSITION_V_OFFSET);
        std::copy(
          twist_update_vec.begin(), twist_update_vec.end(),
          update_vector.begin());

        // 7e. Now rotate the previous covariance for this measurement to get it
        // into the target frame, and add the current measurement's rotated
        // covariance to the previous measurement's rotated covariance, and
        // multiply by the time delta.
        Eigen::MatrixXd prev_covar_rotated =
          rot6d * previous_measurement_covariances_[topic_name] *
          rot6d.transpose();
        covariance_rotated =
          (covariance_rotated.eval() + prev_covar_rotated) * dt;
        copyCovariance(
          covariance_rotated, &(twist_ptr->twist.covariance[0]),
          POSE_SIZE);

        RF_DEBUG(
          "Previous measurement covariance:\n" <<
            previous_measurement_covariances_[topic_name] <<
            "\nPrevious measurement covariance rotated:\n" <<
            prev_covar_rotated << "\nFinal twist covariance:\n" <<
            covariance_rotated << "\n");

        // Now pass this on to prepareTwist, which will convert it to the
        // required frame
        success = prepareTwist(
          twist_ptr, topic_name + "_twist",
          twist_ptr->header.frame_id, update_vector,
          measurement, measurement_covariance);
      }

      // 7f. Update the previous measurement and measurement covariance
      previous_measurements_[topic_name] = cur_measurement;
      previous_measurement_covariances_[topic_name] = covariance;

      retVal = success;
    } else {
      // 7g. If we're in relative mode, remove the initial measurement
      if (relative) {
        if (initial_measurements_.count(topic_name) == 0) {
          initial_measurements_.insert(
            std::pair<std::string, tf2::Transform>(topic_name, pose_tmp));
        }

        tf2::Transform initial_measurement = initial_measurements_[topic_name];
        pose_tmp.setData(initial_measurement.inverseTimes(pose_tmp));
      }

      // 7h. Apply the target frame transformation to the pose object.
      pose_tmp.mult(target_frame_trans, pose_tmp);
      pose_tmp.frame_id_ = final_target_frame;

      // 7i. Finally, copy everything into our measurement and covariance
      // objects
      measurement(StateMemberX) = pose_tmp.getOrigin().x();
      measurement(StateMemberY) = pose_tmp.getOrigin().y();
      measurement(StateMemberZ) = pose_tmp.getOrigin().z();

      // The filter needs roll, pitch, and yaw values instead of quaternions
      double roll, pitch, yaw;
      ros_filter_utilities::quatToRPY(pose_tmp.getRotation(), roll, pitch, yaw);
      measurement(StateMemberRoll) = roll;
      measurement(StateMemberPitch) = pitch;
      measurement(StateMemberYaw) = yaw;

      measurement_covariance.block(0, 0, POSE_SIZE, POSE_SIZE) =
        covariance_rotated.block(0, 0, POSE_SIZE, POSE_SIZE);

      // 8. Handle 2D mode
      if (two_d_mode_) {
        forceTwoD(measurement, measurement_covariance, update_vector);
      }

      retVal = true;
    }
  } else {
    retVal = false;

    RF_DEBUG(
      "Could not transform measurement into " << final_target_frame <<
        ". Ignoring...");
  }

  RF_DEBUG("\n----- /RosFilter<T>::preparePose (" << topic_name << ") ------\n");

  return retVal;
}

template<typename T>
bool RosFilter<T>::prepareTwist(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg,
  const std::string & topic_name, const std::string & target_frame,
  std::vector<bool> & update_vector, Eigen::VectorXd & measurement,
  Eigen::MatrixXd & measurement_covariance)
{
  RF_DEBUG("------ RosFilter<T>::prepareTwist (" << topic_name << ") ------\n");

  // 1. Get the measurement into two separate vector objects.
  tf2::Vector3 twist_lin(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
    msg->twist.twist.linear.z);
  tf2::Vector3 meas_twist_rot(msg->twist.twist.angular.x,
    msg->twist.twist.angular.y,
    msg->twist.twist.angular.z);

  // 1a. This sensor may or may not measure rotational velocity. Regardless,
  // if it measures linear velocity, then later on, we'll need to remove "false"
  // linear velocity resulting from angular velocity and the translational
  // offset of the sensor from the vehicle origin.
  const Eigen::VectorXd & state = filter_.getState();
  tf2::Vector3 state_twist_rot(state(StateMemberVroll),
    state(StateMemberVpitch),
    state(StateMemberVyaw));

  // Determine the frame_id of the data
  std::string msg_frame =
    (msg->header.frame_id == "" ? target_frame : msg->header.frame_id);

  // 2. robot_localization lets users configure which variables from the sensor
  // should be
  //    fused with the filter. This is specified at the sensor level. However,
  //    the data may go through transforms before being fused with the state
  //    estimate. In that case, we need to know which of the transformed
  //    variables came from the pre-transformed "approved" variables (i.e., the
  //    ones that had "true" in their xxx_config parameter). To do this, we
  //    construct matrices using the update vector values on the diagonals, pass
  //    this matrix through the rotation, and use the length of each row to
  //    determine the transformed update vector.
  tf2::Matrix3x3 maskLin(update_vector[StateMemberVx], 0, 0, 0,
    update_vector[StateMemberVy], 0, 0, 0,
    update_vector[StateMemberVz]);

  tf2::Matrix3x3 maskRot(update_vector[StateMemberVroll], 0, 0, 0,
    update_vector[StateMemberVpitch], 0, 0, 0,
    update_vector[StateMemberVyaw]);

  // 3. We'll need to rotate the covariance as well
  Eigen::MatrixXd covariance_rotated(TWIST_SIZE, TWIST_SIZE);
  covariance_rotated.setZero();

  copyCovariance(
    &(msg->twist.covariance[0]), covariance_rotated, topic_name,
    update_vector, POSITION_V_OFFSET, TWIST_SIZE);

  RF_DEBUG(
    "Original measurement as tf object:\nLinear: " <<
      twist_lin << "Rotational: " << meas_twist_rot <<
      "\nOriginal update vector:\n" <<
      update_vector << "\nOriginal covariance matrix:\n" <<
      covariance_rotated << "\n");

  // 4. We need to transform this into the target frame (probably base_link)
  tf2::Transform target_frame_trans;
  bool can_transform = ros_filter_utilities::lookupTransformSafe(
    tf_buffer_.get(), target_frame, msg_frame, msg->header.stamp, tf_timeout_,
    target_frame_trans);

  if (can_transform) {
    // Transform to correct frame. Note that we can get linear velocity
    // as a result of the sensor offset and rotational velocity
    meas_twist_rot = target_frame_trans.getBasis() * meas_twist_rot;
    twist_lin = target_frame_trans.getBasis() * twist_lin +
      target_frame_trans.getOrigin().cross(state_twist_rot);
    maskLin = target_frame_trans.getBasis() * maskLin;
    maskRot = target_frame_trans.getBasis() * maskRot;

    // Now copy the mask values back into the update vector
    update_vector[StateMemberVx] = static_cast<int>(
      maskLin.getRow(StateMemberVx - POSITION_V_OFFSET).length() >= 1e-6);
    update_vector[StateMemberVy] = static_cast<int>(
      maskLin.getRow(StateMemberVy - POSITION_V_OFFSET).length() >= 1e-6);
    update_vector[StateMemberVz] = static_cast<int>(
      maskLin.getRow(StateMemberVz - POSITION_V_OFFSET).length() >= 1e-6);
    update_vector[StateMemberVroll] = static_cast<int>(
      maskRot.getRow(StateMemberVroll - ORIENTATION_V_OFFSET).length() >=
      1e-6);
    update_vector[StateMemberVpitch] = static_cast<int>(
      maskRot.getRow(StateMemberVpitch - ORIENTATION_V_OFFSET).length() >=
      1e-6);
    update_vector[StateMemberVyaw] = static_cast<int>(
      maskRot.getRow(StateMemberVyaw - ORIENTATION_V_OFFSET).length() >=
      1e-6);

    RF_DEBUG(
      msg->header.frame_id <<
        "->" << target_frame << " transform:\n" <<
        target_frame_trans << "\nAfter applying transform to " <<
        target_frame << ", update vector is:\n" <<
        update_vector << "\nAfter applying transform to " <<
        target_frame << ", measurement is:\n" <<
        "Linear: " << twist_lin << "Rotational: " << meas_twist_rot <<
        "\n");

    // 5. Now rotate the covariance: create an augmented
    // matrix that contains a 3D rotation matrix in the
    // upper-left and lower-right quadrants, and zeros
    // elsewhere
    tf2::Matrix3x3 rot(target_frame_trans.getRotation());
    Eigen::MatrixXd rot6d(TWIST_SIZE, TWIST_SIZE);
    rot6d.setIdentity();

    for (size_t r_ind = 0; r_ind < POSITION_SIZE; ++r_ind) {
      rot6d(r_ind, 0) = rot.getRow(r_ind).getX();
      rot6d(r_ind, 1) = rot.getRow(r_ind).getY();
      rot6d(r_ind, 2) = rot.getRow(r_ind).getZ();
      rot6d(r_ind + POSITION_SIZE, 3) = rot.getRow(r_ind).getX();
      rot6d(r_ind + POSITION_SIZE, 4) = rot.getRow(r_ind).getY();
      rot6d(r_ind + POSITION_SIZE, 5) = rot.getRow(r_ind).getZ();
    }

    // Carry out the rotation
    covariance_rotated = rot6d * covariance_rotated.eval() * rot6d.transpose();

    RF_DEBUG("Transformed covariance is \n" << covariance_rotated << "\n");

    // 6. Store our corrected measurement and covariance
    measurement(StateMemberVx) = twist_lin.getX();
    measurement(StateMemberVy) = twist_lin.getY();
    measurement(StateMemberVz) = twist_lin.getZ();
    measurement(StateMemberVroll) = meas_twist_rot.getX();
    measurement(StateMemberVpitch) = meas_twist_rot.getY();
    measurement(StateMemberVyaw) = meas_twist_rot.getZ();

    // Copy the covariances
    measurement_covariance.block(
      POSITION_V_OFFSET, POSITION_V_OFFSET,
      TWIST_SIZE, TWIST_SIZE) =
      covariance_rotated.block(0, 0, TWIST_SIZE, TWIST_SIZE);

    // 7. Handle 2D mode
    if (two_d_mode_) {
      forceTwoD(measurement, measurement_covariance, update_vector);
    }
  } else {
    RF_DEBUG(
      "Could not transform measurement into " << target_frame <<
        ". Ignoring...");
  }

  RF_DEBUG("\n----- /RosFilter<T>::prepareTwist (" << topic_name << ") ------\n");

  return can_transform;
}

template<typename T>
void RosFilter<T>::saveFilterState(T & filter)
{
  FilterStatePtr state = FilterStatePtr(new FilterState());
  state->state_ = Eigen::VectorXd(filter.getState());
  state->estimate_error_covariance_ =
    Eigen::MatrixXd(filter.getEstimateErrorCovariance());
  state->last_measurement_time_ = filter.getLastMeasurementTime();
  state->latest_control_ = Eigen::VectorXd(filter.getControl());
  state->latest_control_time_ = filter.getControlTime();
  filter_state_history_.push_back(state);
  RF_DEBUG(
    "Saved state with timestamp " <<
      std::setprecision(20) <<
      filter_utilities::toSec(state->last_measurement_time_) <<
      " to history. " << filter_state_history_.size() <<
      " measurements are in the queue.\n");
}

template<typename T>
bool RosFilter<T>::revertTo(const rclcpp::Time & time)
{
  RF_DEBUG("\n----- RosFilter<T>::revertTo -----\n");
  RF_DEBUG(
    "\nRequested time was " << std::setprecision(20) <<
      filter_utilities::toSec(time) << "\n")

  // size_t history_size = filter_state_history_.size();

  // Walk back through the queue until we reach a filter state whose time stamp
  // is less than or equal to the requested time. Since every saved state after
  // that time will be overwritten/corrected, we can pop from the queue. If the
  // history is insufficiently short, we just take the oldest state we have.
  FilterStatePtr last_history_state;
  while (!filter_state_history_.empty() &&
    filter_state_history_.back()->last_measurement_time_ > time)
  {
    last_history_state = filter_state_history_.back();
    filter_state_history_.pop_back();
  }

  // If the state history is not empty at this point, it means that our history
  // was large enough, and we should revert to the state at the back of the
  // history deque.
  bool ret_val = false;
  if (!filter_state_history_.empty()) {
    ret_val = true;
    last_history_state = filter_state_history_.back();
  } else {
    RF_DEBUG(
      "Insufficient history to revert to time " <<
        filter_utilities::toSec(time) << "\n");

    if (last_history_state) {
      RF_DEBUG(
        "Will revert to oldest state at " <<
          filter_utilities::toSec(last_history_state->latest_control_time_) <<
          ".\n");

      // ROS_WARN_STREAM_DELAYED_THROTTLE(history_length_, "Could not revert "
      //   "to state with time " << std::setprecision(20) << time <<
      //   ". Instead reverted to state with time " <<
      //   lastHistoryState->lastMeasurementTime_ << ". History size was " <<
      //   history_size);
    }
  }

  // If we have a valid reversion state, revert
  if (last_history_state) {
    // Reset filter to the latest state from the queue.
    const FilterStatePtr & state = filter_state_history_.back();
    filter_.setState(state->state_);
    filter_.setEstimateErrorCovariance(state->estimate_error_covariance_);
    filter_.setLastMeasurementTime(state->last_measurement_time_);

    RF_DEBUG(
      "Reverted to state with time " <<
        filter_utilities::toSec(state->last_measurement_time_) << "\n");

    // Repeat for measurements, but push every measurement onto the measurement
    // queue as we go
    int restored_measurements = 0;
    while (!measurement_history_.empty() &&
      measurement_history_.back()->time_ > time)
    {
      // Don't need to restore measurements that predate our earliest state time
      if (state->last_measurement_time_ <= measurement_history_.back()->time_) {
        measurement_queue_.push(measurement_history_.back());
        restored_measurements++;
      }

      measurement_history_.pop_back();
    }

    RF_DEBUG(
      "Restored " << restored_measurements << " to measurement queue."
        "\n");
  }

  RF_DEBUG("\n----- /RosFilter<T>::revertTo\n");

  return ret_val;
}

template<typename T>
bool RosFilter<T>::validateFilterOutput(nav_msgs::msg::Odometry * message)
{
  return !std::isnan(message->pose.pose.position.x) &&
         !std::isinf(message->pose.pose.position.x) &&
         !std::isnan(message->pose.pose.position.y) &&
         !std::isinf(message->pose.pose.position.y) &&
         !std::isnan(message->pose.pose.position.z) &&
         !std::isinf(message->pose.pose.position.z) &&
         !std::isnan(message->pose.pose.orientation.x) &&
         !std::isinf(message->pose.pose.orientation.x) &&
         !std::isnan(message->pose.pose.orientation.y) &&
         !std::isinf(message->pose.pose.orientation.y) &&
         !std::isnan(message->pose.pose.orientation.z) &&
         !std::isinf(message->pose.pose.orientation.z) &&
         !std::isnan(message->pose.pose.orientation.w) &&
         !std::isinf(message->pose.pose.orientation.w) &&
         !std::isnan(message->twist.twist.linear.x) &&
         !std::isinf(message->twist.twist.linear.x) &&
         !std::isnan(message->twist.twist.linear.y) &&
         !std::isinf(message->twist.twist.linear.y) &&
         !std::isnan(message->twist.twist.linear.z) &&
         !std::isinf(message->twist.twist.linear.z) &&
         !std::isnan(message->twist.twist.angular.x) &&
         !std::isinf(message->twist.twist.angular.x) &&
         !std::isnan(message->twist.twist.angular.y) &&
         !std::isinf(message->twist.twist.angular.y) &&
         !std::isnan(message->twist.twist.angular.z) &&
         !std::isinf(message->twist.twist.angular.z);
}

template<typename T>
void RosFilter<T>::clearExpiredHistory(const rclcpp::Time cutoff_time)
{
  RF_DEBUG(
    "\n----- RosFilter<T>::clearExpiredHistory -----" <<
      "\nCutoff time is " << filter_utilities::toSec(cutoff_time) <<
      "\n");

  int popped_measurements = 0;
  int popped_states = 0;

  while (!measurement_history_.empty() &&
    measurement_history_.front()->time_ < cutoff_time)
  {
    measurement_history_.pop_front();
    popped_measurements++;
  }

  while (!filter_state_history_.empty() &&
    filter_state_history_.front()->last_measurement_time_ < cutoff_time)
  {
    filter_state_history_.pop_front();
    popped_states++;
  }

  RF_DEBUG(
    "\nPopped " << popped_measurements << " measurements and " <<
      popped_states <<
      " states from their respective queues." <<
      "\n---- /RosFilter<T>::clearExpiredHistory ----\n");
}

template<typename T>
void RosFilter<T>::clearMeasurementQueue()
{
  // Clear the measurement queue.
  // This prevents us from immediately undoing our reset.
  while (!measurement_queue_.empty() && rclcpp::ok()) {
    measurement_queue_.pop();
  }
}
}  // namespace robot_localization

template class robot_localization::RosFilter<robot_localization::Ekf>;
template class robot_localization::RosFilter<robot_localization::Ukf>;
