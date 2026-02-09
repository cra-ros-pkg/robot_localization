/*
 * Copyright (c) 2018, Locus Robotics
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
#include <atomic>
#include <csignal>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "robot_localization/ros_filter_types.hpp"

namespace
{
std::atomic_bool g_sigint_requested{false};

void sigintHandler(int)
{
  g_sigint_requested.store(true);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::InitOptions init_options;
  rclcpp::init(argc, argv, init_options, rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, sigintHandler);
  rclcpp::NodeOptions options;
  options.arguments({"ekf_filter_node"});
  options.clock_type(RCL_ROS_TIME);
  std::shared_ptr<robot_localization::RosEkf> filter =
    std::make_shared<robot_localization::RosEkf>(options);

  // Handle lifecycle management after the shared_ptr is created
  if (!filter->get_parameter("lifecycle_managed_node").as_bool()) {
    RCLCPP_INFO(
      filter->get_logger(),
      "Lifecycle management disabled - Using legacy initialization");
    filter->configure();
    filter->activate();
  } else {
    RCLCPP_INFO(filter->get_logger(),
      "Lifecycle management enabled - Node requires external lifecycle management "
      "via ros2 lifecycle commands.");
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(filter->get_node_base_interface());
  while (rclcpp::ok()) {
    executor.spin_some();
    if (g_sigint_requested.load()) {
      break;
    }
  }

  // Ensure lifecycle node is properly shut down to avoid warnings on exit.
  const auto state_id = filter->get_current_state().id();
  if (state_id != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
    uint8_t transition_id = 0;
    if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      transition_id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN;
    } else if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      transition_id = lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN;
    } else if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
      transition_id = lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
    }

    if (transition_id != 0) {
      try {
        filter->trigger_transition(transition_id);
      } catch (const std::exception & e) {
        RCLCPP_WARN(filter->get_logger(), "Failed to shutdown node: %s", e.what());
      }
    }
  }

  rclcpp::shutdown();
  return 0;
}
