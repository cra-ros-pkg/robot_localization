/*
 * Copyright (c) 2021, Charles River Analytics, Inc.
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


#include <gtest/gtest.h>

#include <robot_localization/navsat_conversions.hpp>
#include <chrono>
#include <memory>
#include <string>

#include "robot_localization/srv/set_datum.hpp"
#include "robot_localization/srv/from_ll.hpp"
#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;

TEST(NavSatTransformUTMJumpTest, UtmTest)
{
  auto node_ = rclcpp::Node::make_shared("test_navsat_transform");
  auto setDatumClient = node_->create_client<robot_localization::srv::SetDatum>("/datum");

  EXPECT_TRUE(setDatumClient->wait_for_service(5s));

  // Initialise the navsat_transform node to a UTM zone
  auto setDatumRequest = std::make_shared<robot_localization::srv::SetDatum::Request>();
  setDatumRequest->geo_pose.position.latitude = 1;
  setDatumRequest->geo_pose.position.longitude = 4;
  setDatumRequest->geo_pose.orientation.w = 1;

  auto setDatumResponse = setDatumClient->async_send_request(setDatumRequest);
  auto ret = rclcpp::spin_until_future_complete(node_, setDatumResponse, 5s);
  EXPECT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return ret;
}
