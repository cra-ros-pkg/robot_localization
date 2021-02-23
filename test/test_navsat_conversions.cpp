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

#include "robot_localization/navsat_conversions.h"

#include <gtest/gtest.h>

#include <string>

TEST(NavsatConversionsTest, UtmTest)
{
    const double lat = 51.423964;
    const double lon = 5.494271;

    double UTMNorthing;
    double UTMEasting;
    std::string UTMZone;
    RobotLocalization::NavsatConversions::LLtoUTM(lat, lon, UTMNorthing, UTMEasting, UTMZone);
    EXPECT_NEAR(UTMNorthing, 5699924.709, 1e-2);
    EXPECT_NEAR(UTMEasting, 673409.989, 1e-2);
    EXPECT_EQ(UTMZone, "31U");
    double lat_new;
    double lon_new;
    RobotLocalization::NavsatConversions::UTMtoLL(UTMNorthing, UTMEasting, UTMZone, lat_new, lon_new);
    EXPECT_NEAR(lat, lat, 1e-5);
    EXPECT_NEAR(lon, lon, 1e-5);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
