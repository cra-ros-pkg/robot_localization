/*
 * Copyright (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * Copyright (c) 2017, Locus Robotics, Inc.
 * Copyright (c) 2019, Steve Macenski
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

#ifndef ROBOT_LOCALIZATION__FILTER_STATE_HPP_
#define ROBOT_LOCALIZATION__FILTER_STATE_HPP_

#include <memory>
#include "Eigen/Dense"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

namespace robot_localization
{

/**
 * @brief Structure used for storing and comparing filter states
 *
 * This structure is useful when higher-level classes need to remember filter
 * history. Measurement units are assumed to be in meters and radians. Times are
 * real-valued and measured in seconds.
 */
struct FilterState
{
  FilterState()
  : state_(), estimate_error_covariance_(), latest_control_(),
    last_measurement_time_(0.0), latest_control_time_(0)
  {
  }

  // The filter state vector
  Eigen::VectorXd state_;

  // The filter error covariance matrix
  Eigen::MatrixXd estimate_error_covariance_;

  // The most recent control vector
  Eigen::VectorXd latest_control_;

  // The time stamp of the most recent measurement for the filter
  rclcpp::Time last_measurement_time_;

  // The time stamp of the most recent control term
  rclcpp::Time latest_control_time_;

  // We want the queue to be sorted from latest to earliest timestamps.
  bool operator()(const FilterState & a, const FilterState & b)
  {
    return a.last_measurement_time_ < b.last_measurement_time_;
  }
};
using FilterStatePtr = std::shared_ptr<FilterState>;

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__FILTER_STATE_HPP_
