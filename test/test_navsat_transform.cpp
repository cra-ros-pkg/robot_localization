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

#include "robot_localization/navsat_transform.h"
#include <robot_localization/SetDatum.h>
#include <robot_localization/ToLL.h>
#include <robot_localization/FromLL.h>

#include <gtest/gtest.h>

#include <string>

TEST(NavSatTransformUTMJumpTest, UtmTest)
{
  ros::NodeHandle nh;
  ros::ServiceClient set_datum_client = nh.serviceClient<robot_localization::SetDatum>("/datum");
  ros::ServiceClient from_ll_client = nh.serviceClient<robot_localization::FromLL>("/fromLL");

  EXPECT_TRUE(set_datum_client.waitForExistence(ros::Duration(5)));

  // Initialise the navsat_transform node to a UTM zone
  robot_localization::SetDatum set_datum_srv;
  set_datum_srv.request.geo_pose.position.latitude = 1;
  set_datum_srv.request.geo_pose.position.longitude = 4;
  set_datum_srv.request.geo_pose.orientation.w = 1;
  EXPECT_TRUE(set_datum_client.call(set_datum_srv));

  // Let the node figure out its transforms
  ros::Duration(0.2).sleep();

  // Request the GPS point of said point:
  robot_localization::FromLL from_ll_srv;
  from_ll_srv.request.ll_point.latitude = 10;
  from_ll_srv.request.ll_point.longitude = 4.5;
  EXPECT_TRUE(from_ll_client.call(from_ll_srv));
  auto initial_response = from_ll_srv.response;

  // Request GPS point also in that zone
  from_ll_srv.request.ll_point.longitude = 5.5;
  EXPECT_TRUE(from_ll_client.call(from_ll_srv));
  auto same_zone_response = from_ll_srv.response;

  // 1° of longitude is about 111 kilometers in length
  EXPECT_NEAR(initial_response.map_point.x, same_zone_response.map_point.x, 120e3);
  EXPECT_NEAR(initial_response.map_point.y, same_zone_response.map_point.y, 120e3);

  // Request GPS point from neighboring zone (zone crossing is at 6 degrees)
  from_ll_srv.request.ll_point.longitude = 6.5;
  from_ll_client.call(from_ll_srv);
  auto neighbour_zone_response = from_ll_srv.response;

  EXPECT_NEAR(initial_response.map_point.x, neighbour_zone_response.map_point.x, 2*120e3);
  EXPECT_NEAR(initial_response.map_point.y, neighbour_zone_response.map_point.y, 2*120e3);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_navsat_transform");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
