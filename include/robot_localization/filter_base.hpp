/*
 * Copyright (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * Copyright (c) 2017, Locus Robotics, Inc.
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

#ifndef ROBOT_LOCALIZATION__FILTER_BASE_HPP_
#define ROBOT_LOCALIZATION__FILTER_BASE_HPP_

#include <robot_localization/filter_common.hpp>
#include <robot_localization/filter_utilities.hpp>
#include <robot_localization/measurement.hpp>
#include <robot_localization/filter_state.hpp>

#include <algorithm>
#include <memory>
#include <ostream>
#include <string>
#include <vector>
#include <limits>

namespace robot_localization
{

class FilterBase
{
public:
  /**
   * @brief Constructor for the FilterBase class
   */
  FilterBase();

  /**
   * @brief Destructor for the FilterBase class
   */
  virtual ~FilterBase();

  /**
   * @brief Resets filter to its unintialized state
   */
  void reset();

  /**
   * @brief Computes a dynamic process noise covariance matrix using the
   * parameterized state This allows us to, e.g., not increase the pose
   * covariance values when the vehicle is not moving
   * @param[in] state - The STATE_SIZE state vector that is used to generate the
   * dynamic process noise covariance
   */
  void computeDynamicProcessNoiseCovariance(const Eigen::VectorXd & state);

  /**
   * @brief Carries out the correct step in the predict/update cycle. This
   * method must be implemented by subclasses.
   * @param[in] measurement - The measurement to fuse with the state estimate
   */
  virtual void correct(const Measurement & measurement) = 0;

  /**
   * @brief Returns the control vector currently being used
   *
   * @return The control vector
   */
  const Eigen::VectorXd & getControl();

  /**
   * @brief Returns the time at which the control term was issued
   *
   * @return The time the control vector was issued
   */
  const rclcpp::Time & getControlTime();

  /**
   * @brief Gets the value of the debug_ variable.
   *
   * @return True if in debug mode, false otherwise
   */
  bool getDebug();

  /**
   * @brief Gets the estimate error covariance
   *
   * @return A copy of the estimate error covariance matrix
   */
  const Eigen::MatrixXd & getEstimateErrorCovariance();

  /**
   * @brief Gets the filter's initialized status
   *
   * @return True if we've received our first measurement, false otherwise
   */
  bool getInitializedStatus();

  /**
   * @brief Gets the most recent measurement time
   *
   * @return The time at which we last received a measurement
   */
  const rclcpp::Time & getLastMeasurementTime();

  /**
   * @brief Gets the filter's predicted state, i.e., the state estimate before
   * correct() is called.
   *
   * @return A constant reference to the predicted state
   */
  const Eigen::VectorXd & getPredictedState();

  /**
   * @brief Gets the filter's process noise covariance
   *
   * @return A constant reference to the process noise covariance
   */
  const Eigen::MatrixXd & getProcessNoiseCovariance();

  /**
   * @brief Gets the sensor timeout value (in seconds)
   *
   * @return The sensor timeout value
   */
  const rclcpp::Duration & getSensorTimeout();

  /**
   * @brief Gets the filter state
   *
   * @return A constant reference to the current state
   */
  const Eigen::VectorXd & getState();

  /**
   * @brief Carries out the predict step in the predict/update cycle.
   *
   * Projects the state and error matrices forward using a model of the
   * vehicle's motion. This method must be implemented by subclasses.
   *
   * @param[in] reference_time - The time at which the prediction is being made
   * @param[in] delta - The time step over which to predict.
   */
  virtual void predict(
    const rclcpp::Time & reference_time,
    const rclcpp::Duration & delta) = 0;

  /**
   * @brief Does some final preprocessing, carries out the predict/update cycle
   * @param[in] measurement - The measurement object to fuse into the filter
   */
  virtual void processMeasurement(const Measurement & measurement);

  /**
   * @brief Sets the most recent control term
   * @param[in] control - The control term to be applied
   * @param[in] control_time - The time at which the control in question was
   * received
   */
  void setControl(
    const Eigen::VectorXd & control,
    const rclcpp::Time & control_time);

  /**
   * @brief Sets the control update vector and acceleration limits
   * @param[in] update_vector - The values the control term affects
   * @param[in] control_timeout - Timeout value, in seconds, after which a
   * control is considered stale
   * @param[in] acceleration_limits - The acceleration limits for the control
   * variables
   * @param[in] acceleration_gains - Gains applied to the control term-derived
   * acceleration
   * @param[in] deceleration_limits - The deceleration limits for the control
   * variables
   * @param[in] deceleration_gains - Gains applied to the control term-derived
   * deceleration
   */
  void setControlParams(
    const std::vector<bool> & update_vector,
    const rclcpp::Duration & control_timeout,
    const std::vector<double> & acceleration_limits,
    const std::vector<double> & acceleration_gains,
    const std::vector<double> & deceleration_limits,
    const std::vector<double> & deceleration_gains);

  /**
   * @brief Sets the filter into debug mode
   *
   * NOTE: this will generates a lot of debug output to the provided stream. The
   * value must be a pointer to a valid ostream object.
   *
   * @param[in] debug - Whether or not to place the filter in debug mode
   * @param[in] out_stream - If debug is true, then this must have a valid
   * pointer. If the pointer is invalid, the filter will not enter debug mode.
   * If debug is false, outStream is ignored.
   */
  void setDebug(const bool debug, std::ostream * out_stream = NULL);

  /**
   * @brief Enables dynamic process noise covariance calculation
   * @param[in] dynamic_process_noise_covariance - Whether or not to compute
   * dynamic process noise covariance matrices
   */
  void setUseDynamicProcessNoiseCovariance(
    const bool dynamic_process_noise_covariance);

  /**
   * @brief Manually sets the filter's estimate error covariance
   * @param[in] estimate_error_covariance - The state to set as the filter's
   * current state
   */
  void
  setEstimateErrorCovariance(const Eigen::MatrixXd & estimate_error_covariance);

  /**
   * @brief Sets the filter's last measurement time.
   * @param[in] last_measurement_time - The last measurement time of the filter
   */
  void setLastMeasurementTime(const rclcpp::Time & last_measurement_time);

  /**
   * @brief Sets the process noise covariance for the filter.
   *
   * This enables external initialization, which is important, as this matrix
   * can be difficult to tune for a given implementation.
   *
   * @param[in] process_noise_covariance - The STATE_SIZExSTATE_SIZE process
   * noise covariance matrix to use for the filter
   */
  void
  setProcessNoiseCovariance(const Eigen::MatrixXd & process_noise_covariance);

  /**
   * @brief Sets the sensor timeout
   * @param[in] sensor_timeout - The time, in seconds, for a sensor to be
   * considered having timed out
   */
  void setSensorTimeout(const rclcpp::Duration & sensor_timeout);

  /**
   * @brief Manually sets the filter's state
   *
   * @param[in] state - The state to set as the filter's current state
   */
  void setState(const Eigen::VectorXd & state);

  /**
   * @brief Ensures a given time delta is valid (helps with bag file playback
   * issues)
   *
   * @param[in, out] delta - The time delta to validate
   */
  void validateDelta(rclcpp::Duration & delta);

protected:
  /**
   * @brief Method for settings bounds on acceleration values derived from
   * controls
   * @param[in] state - The current state variable (e.g., linear X velocity)
   * @param[in] control - The current control commanded velocity corresponding
   * to the state variable
   * @param[in] acceleration_limit - Limit for acceleration (regardless of
   * driving direction)
   * @param[in] acceleration_gain - Gain applied to acceleration control error
   * @param[in] deceleration_limit - Limit for deceleration (moving towards
   * zero, regardless of driving direction)
   * @param[in] deceleration_gain - Gain applied to deceleration control error
   * @return a usable acceleration estimate for the control vector
   */
  double computeControlAcceleration(
    const double state,
    const double control,
    const double acceleration_limit,
    const double acceleration_gain,
    const double deceleration_limit,
    const double deceleration_gain);

  /**
   * @brief Keeps the state Euler angles in the range [-pi, pi]
   */
  virtual void wrapStateAngles();

  /**
   * @brief Tests if innovation is within N-sigmas of covariance. Returns true
   * if passed the test.
   * @param[in] innovation - The difference between the measurement and the
   * state
   * @param[in] innovation_covariance - The innovation error
   * @param[in] n_sigmas - Number of standard deviations that are considered
   * acceptable
   */
  virtual bool
  checkMahalanobisThreshold(
    const Eigen::VectorXd & innovation,
    const Eigen::MatrixXd & innovation_covariance,
    const double n_sigmas);

  /**
   * @brief Converts the control term to an acceleration to be applied in the
   * prediction step
   * @param[in] reference_time - The time of the update (measurement used in the
   * prediction step)
   */
  void prepareControl(
    const rclcpp::Time & reference_time,
    const rclcpp::Duration &);

  /**
   * @brief Whether or not we've received any measurements
   */
  bool initialized_;

  /**
   * @brief Whether or not we apply the control term
   */
  bool use_control_;

  /**
   * @brief If true, uses the robot's vehicle state and the static process noise
   * covariance matrix to generate a dynamic process noise covariance matrix
   */
  bool use_dynamic_process_noise_covariance_;

  /**
   * @brief Timeout value, in seconds, after which a control is considered stale
   */
  rclcpp::Duration control_timeout_;

  /**
   * @brief Tracks the time the filter was last updated using a measurement.
   *
   * This value is used to monitor sensor readings with respect to the
   * sensorTimeout_. We also use it to compute the time delta values for our
   * prediction step.
   */
  rclcpp::Time last_measurement_time_;

  /**
   * @brief The time of reception of the most recent control term
   */
  rclcpp::Time latest_control_time_;

  /**
   * @brief The updates to the filter - both predict and correct - are driven by
   * measurements. If we get a gap in measurements for some reason, we want the
   * filter to continue estimating. When this gap occurs, as specified by this
   * timeout, we will continue to call predict() at the filter's frequency.
   */
  rclcpp::Duration sensor_timeout_;

  /**
   * @brief Used for outputting debug messages
   */
  std::ostream * debug_stream_;

  /**
   * @brief Gains applied to acceleration derived from control term
   */
  std::vector<double> acceleration_gains_;

  /**
   * @brief Caps the acceleration we apply from control input
   */
  std::vector<double> acceleration_limits_;

  /**
   * @brief Gains applied to deceleration derived from control term
   */
  std::vector<double> deceleration_gains_;

  /**
   * @brief Caps the deceleration we apply from control input
   */
  std::vector<double> deceleration_limits_;

  /**
   * @brief Which control variables are being used (e.g., not every vehicle is
   * controllable in Y or Z)
   */
  std::vector<bool> control_update_vector_;

  /**
   * @brief Variable that gets updated every time we process a measurement and
   * we have a valid control
   */
  Eigen::VectorXd control_acceleration_;

  /**
   * @brief Latest control term
   */
  Eigen::VectorXd latest_control_;

  /**
   * @brief Holds the last predicted state of the filter
   */
  Eigen::VectorXd predicted_state_;

  /**
   * @brief This is the robot's state vector, which is what we are trying to
   * filter. The values in this vector are what get reported by the node.
   */
  Eigen::VectorXd state_;

  /**
   * @brief Covariance matrices can be incredibly unstable. We can add a small
   * value to it at each iteration to help maintain its positive-definite
   * property.
   */
  Eigen::MatrixXd covariance_epsilon_;

  /**
   * @brief Gets updated when useDynamicProcessNoise_ is true
   */
  Eigen::MatrixXd dynamic_process_noise_covariance_;

  /**
   * @brief This matrix stores the total error in our position estimate (the
   * state_ variable).
   */
  Eigen::MatrixXd estimate_error_covariance_;

  /**
   * @brief We need the identity for a few operations. Better to store it.
   */
  Eigen::MatrixXd identity_;

  /**
   * @brief As we move through the world, we follow a predict/update cycle. If
   * one were to imagine a scenario where all we did was make predictions
   * without correcting, the error in our position estimate would grow without
   * bound. This error is stored in the stateEstimateCovariance_ matrix.
   * However, this matrix doesn't answer the question of *how much* our error
   * should grow for each time step. That's where the processNoiseCovariance
   * matrix comes in. When we make a prediction using the transfer function, we
   * add this matrix (times delta_t) to the state estimate covariance matrix.
   */
  Eigen::MatrixXd process_noise_covariance_;

  /**
   * @brief The Kalman filter transfer function
   *
   * Kalman filters and extended Kalman filters project the current state
   * forward in time. This is the "predict" part of the predict/correct cycle. A
   * Kalman filter has a (typically constant) matrix A that defines  how to turn
   * the current state, x, into the predicted next state. For an EKF, this
   * matrix becomes a function f(x). However, this function can still be
   * expressed as a matrix to make the math a little cleaner, which is what we
   * do here. Technically, each row in the matrix is actually a function. Some
   * rows will contain many trigonometric functions, which are of course
   * non-linear. In any case, you can think of this as the 'A' matrix in the
   * Kalman filter formulation.
   */
  Eigen::MatrixXd transfer_function_;

  /**
   * @brief The Kalman filter transfer function Jacobian
   *
   * The transfer function is allowed to be non-linear in an EKF, but for
   * propagating (predicting) the covariance matrix, we need to linearize it
   * about the current mean (i.e., state). This is done via a Jacobian, which
   * calculates partial derivatives of each row of the transfer function matrix
   * with respect to each state variable.
   */
  Eigen::MatrixXd transfer_function_jacobian_;

private:
  /**
   * @brief Whether or not the filter is in debug mode
   */
  bool debug_;
};

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__FILTER_BASE_HPP_
