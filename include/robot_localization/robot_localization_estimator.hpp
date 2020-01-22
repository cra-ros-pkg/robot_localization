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

#ifndef ROBOT_LOCALIZATION__ROBOT_LOCALIZATION_ESTIMATOR_HPP_
#define ROBOT_LOCALIZATION__ROBOT_LOCALIZATION_ESTIMATOR_HPP_

#include <boost/circular_buffer.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <vector>

#include "robot_localization/filter_base.hpp"
#include "robot_localization/filter_common.hpp"
#include "robot_localization/filter_utilities.hpp"

namespace robot_localization
{

struct Twist
{
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

//! @brief Robot Localization Estimator State
//!
//! The Estimator State data structure bundles the state information of the
//! estimator.
//!
struct EstimatorState
{
  EstimatorState()
  : time_stamp(0.0),
    state(STATE_SIZE),
    covariance(STATE_SIZE, STATE_SIZE)
  {
    state.setZero();
    covariance.setZero();
  }

  //! @brief Time at which this state is/was achieved
  double time_stamp;

  //! @brief System state at time = time_stamp
  Eigen::VectorXd state;

  //! @brief System state covariance at time = time_stamp
  Eigen::MatrixXd covariance;

  friend std::ostream & operator<<(std::ostream & os, const EstimatorState & state)
  {
    return os << "state:\n - time_stamp: " << state.time_stamp <<
           "\n - state: \n" << state.state <<
           " - covariance: \n" << state.covariance;
  }
};

namespace EstimatorResults
{
enum EstimatorResult
{
  ExtrapolationIntoFuture = 0,
  Interpolation,
  ExtrapolationIntoPast,
  Exact,
  EmptyBuffer,
  Failed
};
}  // namespace EstimatorResults

namespace FilterTypes
{
enum FilterType
{
  EKF = 0,
  UKF,
  NotDefined
};
}  // namespace FilterTypes

//! @brief Robot Localization Listener class
//!
//! The Robot Localization Estimator class buffers states of and inputs to a
//! system and can interpolate and extrapolate based on a given system model.
//!
class RobotLocalizationEstimator
{
public:
  //! @brief Constructor for the RobotLocalizationListener class
  //!
  //! @param[in] args - Generic argument container (not used here, but needed
  //! so that the ROS filters can pass arbitrary arguments to templated filter
  //! types).
  //!
  explicit RobotLocalizationEstimator(
    unsigned int buffer_capacity,
    FilterTypes::FilterType filter_type,
    const Eigen::MatrixXd & process_noise_covariance,
    const std::vector<double> & filter_args = std::vector<double>());

  //! @brief Destructor for the RobotLocalizationListener class
  //!
  virtual ~RobotLocalizationEstimator();

  //! @brief Sets the current internal state of the listener.
  //!
  //! @param[in] state - The new state vector to set the internal state to
  //!
  void setState(const EstimatorState & state);

  //! @brief Returns the state at a given time
  //!
  //! Projects the current state and error matrices forward using a model of the robot's motion.
  //!
  //! @param[in] time - The time to which the prediction is being made
  //! @param[out] state - The returned state at the given time
  //!
  //! @return GetStateResult enum
  //!
  EstimatorResults::EstimatorResult getState(
    const double time,
    EstimatorState & state) const;

  //! @brief Clears the internal state buffer
  //!
  void clearBuffer();

  //! @brief Sets the buffer capacity
  //!
  //! @param[in] capacity - The new buffer capacity
  //!
  void setBufferCapacity(const int capacity);

  //! @brief Returns the buffer capacity
  //!
  //! Returns the number of EstimatorState objects that can be pushed to the
  //! buffer before old ones are dropped. (The capacity of the buffer).
  //!
  //! @return buffer capacity
  //!
  unsigned int getBufferCapacity() const;

  //! @brief Returns the current buffer size
  //!
  //! Returns the number of EstimatorState objects currently in the buffer.
  //!
  //! @return current buffer size
  //!
  unsigned int getSize() const;

private:
  friend std::ostream & operator<<(
    std::ostream & os,
    const RobotLocalizationEstimator & rle)
  {
    for (boost::circular_buffer<EstimatorState>::const_iterator it =
      rle.state_buffer_.begin(); it != rle.state_buffer_.end(); ++it)
    {
      os << *it << "\n";
    }
    return os;
  }

  //! @brief Extrapolates the given state to a requested time stamp
  //!
  //! @param[in] boundary_state - state from which to extrapolate
  //! @param[in] requested_time - time stamp to extrapolate to
  //! @param[out] state_at_req_time - predicted state at requested time
  //!
  void extrapolate(
    const EstimatorState & boundary_state,
    const double requested_time,
    EstimatorState & state_at_req_time) const;

  //! @brief Interpolates the given state to a requested time stamp
  //!
  //! @param[in] given_state_1 - last state update before requested time
  //! @param[in] given_state_2 - next state update after requested time
  //! @param[in] requested_time - time stamp to extrapolate to
  //! @param[out] state_at_req_time - predicted state at requested time
  //!
  void interpolate(
    const EstimatorState & given_state_1,
    const EstimatorState & /*given_state_2*/,
    const double requested_time, EstimatorState & state_at_req_time) const;

  //!
  //! @brief The buffer holding the system states that have come in.
  //! Interpolation and extrapolation is done starting from these states.
  //!
  boost::circular_buffer<EstimatorState> state_buffer_;

  //!
  //! @brief A pointer to the filter instance that is used for extrapolation
  //!
  std::unique_ptr<FilterBase> filter_;
};

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__ROBOT_LOCALIZATION_ESTIMATOR_HPP_
