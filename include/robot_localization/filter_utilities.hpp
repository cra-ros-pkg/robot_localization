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

#ifndef ROBOT_LOCALIZATION__FILTER_UTILITIES_HPP_
#define ROBOT_LOCALIZATION__FILTER_UTILITIES_HPP_

#include <Eigen/Dense>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>

#include <iostream>
#include <ostream>
#include <string>
#include <vector>

#define FB_DEBUG(msg) \
  if (getDebug()) { \
    *debug_stream_ << msg; \
  }

// Handy methods for debug output
std::ostream & operator<<(std::ostream & os, const Eigen::MatrixXd & mat);
std::ostream & operator<<(std::ostream & os, const Eigen::VectorXd & vec);
std::ostream & operator<<(std::ostream & os, const std::vector<size_t> & vec);
std::ostream & operator<<(std::ostream & os, const std::vector<int> & vec);

namespace robot_localization
{
namespace filter_utilities
{

/**
 * @brief Utility method keeping RPY angles in the range [-pi, pi]
 * @param[in] rotation - The rotation to bind
 * @return the bounded value
 */
inline double clampRotation(double rotation)
{
  while (rotation > PI) {
    rotation -= TAU;
  }

  while (rotation < -PI) {
    rotation += TAU;
  }

  return rotation;
}

/**
 * @brief Utility method for appending tf2 prefixes cleanly
 * @param[in] tf_prefix - the tf2 prefix to append
 * @param[in, out] frame_id - the resulting frame_id value
 */
inline void appendPrefix(std::string tf_prefix, std::string & frame_id)
{
  // Strip all leading slashes for tf2 compliance
  if (!frame_id.empty() && frame_id.at(0) == '/') {
    frame_id = frame_id.substr(1);
  }

  if (!tf_prefix.empty() && tf_prefix.at(0) == '/') {
    tf_prefix = tf_prefix.substr(1);
  }

  // If we do have a tf prefix, then put a slash in between
  if (!tf_prefix.empty()) {
    frame_id = tf_prefix + "/" + frame_id;
  }
}

inline double nanosecToSec(const rcl_time_point_value_t nanoseconds)
{
  return static_cast<double>(nanoseconds) * 1e-9;
}

inline int secToNanosec(const double seconds)
{
  return static_cast<int>(seconds * 1e9);
}

inline double toSec(const rclcpp::Duration & duration)
{
  return nanosecToSec(duration.nanoseconds());
}

inline double toSec(const rclcpp::Time & time)
{
  return nanosecToSec(time.nanoseconds());
}

inline double toSec(const std_msgs::msg::Header::_stamp_type & stamp)
{
  return static_cast<double>(stamp.sec) + nanosecToSec(stamp.nanosec);
}

}  // namespace filter_utilities
}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__FILTER_UTILITIES_HPP_
