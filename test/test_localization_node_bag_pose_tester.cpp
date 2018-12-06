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

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <fstream>
#include <functional> // for bind()
#include <gtest/gtest.h>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <sstream>
#include <string>

using namespace std;

nav_msgs::msg::Odometry filtered_;

using namespace std::chrono_literals;

void filterCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  filtered_ = *msg;
}

TEST(BagTest, PoseCheck) {
  auto node = rclcpp::Node::make_shared("localization_node_bag_pose_tester");

  double finalX = 0;
  double finalY = 0;
  double finalZ = 0;
  double tolerance = 0;
  bool outputFinalPosition = false;
  std::string finalPositionFile;

  node->get_parameter("final_x", finalX);
  node->get_parameter("final_y", finalY);
  node->get_parameter("final_z", finalZ);
  node->get_parameter("tolerance", tolerance);
  node->get_parameter_or("output_final_position", outputFinalPosition, false);
  node->get_parameter_or("output_location", finalPositionFile,
                         std::string("test.txt"));

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);
  // Get parameters
  for (auto &parameter : parameters_client->get_parameters(
           {"final_x", "final_y", "final_z", "tolerance",
            "output_final_position", "output_location"})) {
    std::cout << "Parameter name: " << parameter.get_name() << std::endl;
    std::cout << "Parameter value (" << parameter.get_type_name()
              << "): " << parameter.value_to_string() << std::endl;
  }

  auto filteredSub = node->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", filterCallback);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    rclcpp::Rate(3).sleep();
  }

  if (outputFinalPosition) {
    try {
      std::ofstream posOut;
      posOut.open(finalPositionFile.c_str(), std::ofstream::app);
      posOut << filtered_.pose.pose.position.x << " "
             << filtered_.pose.pose.position.y << " "
             << filtered_.pose.pose.position.z << std::endl;
      posOut.close();
    } catch (...) {
      RCLCPP_ERROR(node->get_logger(), "Unable to open output file.\n");
    }
  }

  double xDiff = filtered_.pose.pose.position.x - finalX;
  double yDiff = filtered_.pose.pose.position.y - finalY;
  double zDiff = filtered_.pose.pose.position.z - finalZ;

  std::cout << "xDiff =" << xDiff << std::endl;
  std::cout << "yDiff =" << yDiff << std::endl;
  std::cout << "zDiff =" << zDiff << std::endl;

  EXPECT_LT(::sqrt(xDiff * xDiff + yDiff * yDiff + zDiff * zDiff), tolerance);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::Rate(0.5).sleep();

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
