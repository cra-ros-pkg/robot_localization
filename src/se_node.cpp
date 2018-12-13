/*
 * Copyright (c) 2018, Locus Robotics
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
#include <robot_localization/filter_base.hpp>
#include <robot_localization/ros_filter.hpp>
#include <robot_localization/ukf.hpp>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <string>
#include <memory>
#include <vector>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("se_node");

  std::string filter_type = "ekf";
  node->get_parameter("filter_type", filter_type);
  std::transform(filter_type.begin(), filter_type.end(), filter_type.begin(),
    ::tolower);

  robot_localization::FilterBase::UniquePtr filter;

  if (filter_type == "ukf") {
    double alpha = 0.001;
    double kappa = 0.0;
    double beta = 2.0;

    node->get_parameter("alpha", alpha);
    node->get_parameter("kappa", kappa);
    node->get_parameter("beta", beta);

    filter = std::make_unique<robot_localization::Ukf>(alpha, kappa, beta);
  } else {
    if (filter_type != "ekf") {
      std::cerr << "Unsupported filter type of " << filter_type <<
        " specified. Defaulting to ekf.\n";
    }

    filter = std::make_unique<robot_localization::Ekf>();
  }

  robot_localization::RosFilter ros_filter(node, filter);
  ros_filter.run();

  return 0;
}
