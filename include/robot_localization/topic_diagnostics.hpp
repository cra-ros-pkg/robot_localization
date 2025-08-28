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

#ifndef ROBOT_LOCALIZATION__TOPIC_DIAGNOSTICS_HPP_
#define ROBOT_LOCALIZATION__TOPIC_DIAGNOSTICS_HPP_

#include <memory>
#include <diagnostic_updater/publisher.hpp>

namespace robot_localization
{

//! @brief Struct to hold settings for topic diagnostics
struct TopicDiagnosticSettings
{
  //! @brief The minimum frequency for the topic
  double min_freq = 0.0;
  //! @brief The maximum frequency for the topic
  double max_freq = 0.0;
  //! @brief The tolerance for the frequency
  double tolerance = 0.1;
  //! @brief The window size for the frequency
  int window_size = 5;
  //! @brief The minimum acceptable time difference between messages
  double min_stamp_dt_acceptable = -1.0;
  //! @brief The maximum acceptable time difference between messages
  double max_stamp_dt_acceptable = 5.0;
};

//! @brief Class to encapsulate \class diagnostic_updater::TopicDiagnostic for more convenient API
//! The contructor for \class diagnostic_updater::TopicDiagnostic takes parameters by a \class diagnostic_updater::FrequencyStatusParam
//! which stores the min and max frequency as pointers, what is inconvenient. 
//! This class provides storage for min and max frequency and passes them to \class diagnostic_updater::TopicDiagnostic
class TopicDiagnostic final
{
public:
  TopicDiagnostic(
    std::string name, diagnostic_updater::Updater & diag, const TopicDiagnosticSettings & settings,
    const rclcpp::Clock::SharedPtr & clock = std::make_shared<rclcpp::Clock>())
  : min_freq_(settings.min_freq),
    max_freq_(settings.max_freq),
    task_(std::make_unique<diagnostic_updater::TopicDiagnostic>(
      name, diag,
      diagnostic_updater::FrequencyStatusParam(
        &min_freq_, &max_freq_, settings.tolerance, settings.window_size),
      diagnostic_updater::TimeStampStatusParam{
        settings.min_stamp_dt_acceptable, settings.max_stamp_dt_acceptable},
      clock))
  {
  }

  TopicDiagnostic(const TopicDiagnostic &) = delete;
  TopicDiagnostic(TopicDiagnostic &&) = default;
  TopicDiagnostic & operator=(const TopicDiagnostic &) = delete;
  TopicDiagnostic & operator=(TopicDiagnostic &&) = default;

  void tick(const rclcpp::Time & stamp) { task_->tick(stamp); }

private:
  double min_freq_ = 0.0;
  double max_freq_ = 0.0;
  std::unique_ptr<diagnostic_updater::TopicDiagnostic> task_;
};
}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__TOPIC_DIAGNOSTICS_HPP_