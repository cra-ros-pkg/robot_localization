/*
 * Copyright (c) 2014, 2015, 2016, Charles River Analytics, Inc.
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

#ifndef ROBOT_LOCALIZATION_LISTENER_H
#define ROBOT_LOCALIZATION_LISTENER_H

#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>

#include <fstream>
#include <vector>
#include <set>
#include <queue>

namespace RobotLocalization
{

//! @brief Robot Localization Estimator State
//!
//! The Estimator State data structure bundles the state information of the
//! estimator.
//!
struct EstimatorState
{
    //! @brief Time at which this state is/was achieved
    double time_stamp;

    //! @brief System state at time = time_stamp
    Eigen::VectorXd state;

    //! @brief System state covariance at time = time_stamp
    Eigen::MatrixXd covariance;

    //! @brief System input at time = time_stamp
    Eigen::VectorXd input;
};

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
    //! @param[in] args - Generic argument container (not used here, but
    //! needed so that the ROS filters can pass arbitrary arguments to
    //! templated filter types).
    //!
    explicit RobotLocalizationEstimator();

    //! @brief Destructor for the RobotLocalizationListener class
    //!
    ~RobotLocalizationEstimator();

    //! @brief Sets the current internal state of the listener.
    //!
    //! @param[in] state - The new state vector to set the internal state to
    //!
    void setState(const EstimatorState& state);

    //! @brief Returns the most recent state
    //!
    //! Projects the current state and error matrices forward using a model of
    //! the vehicle's motion.
    //!
    //! @param[out] state - The returned (most recent) state
    //!
    int getState(EstimatorState &state);

    //! @brief Returns the state at a given time
    //!
    //! Projects the current state and error matrices forward using a model of
    //! the vehicle's motion.
    //!
    //! @param[in] time - The time at which the prediction is being made
    //! @param[out] state - The returned state at the given time
    //!
    int getState(const double time, EstimatorState &state);

  private:

    boost::circular_buffer<EstimatorState> state_buffer_;

};

}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_LISTENER_H
