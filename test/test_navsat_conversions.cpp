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

#include <cmath>
#include <string>

#include "GeographicLib/Geocentric.hpp"
#include "navsat_conversions.hpp"
#include "robot_localization/navsat_transform.hpp"

void NavsatConversionsTest(
  const double lat, const double lon,
  const double UTMNorthing, const double UTMEasting,
  const std::string UTMZone, const double gamma)
{
  double UTMNorthing_new;
  double UTMEasting_new;
  std::string UTMZone_new;
  double gamma_new;
  robot_localization::navsat_conversions::LLtoUTM(
    lat, lon, UTMNorthing_new, UTMEasting_new, UTMZone_new, gamma_new);
  EXPECT_NEAR(UTMNorthing, UTMNorthing_new, 1e-2);
  EXPECT_NEAR(UTMEasting, UTMEasting_new, 1e-2);
  EXPECT_EQ(UTMZone, UTMZone_new);
  EXPECT_NEAR(gamma, gamma_new, 1e-2);
  double lat_new;
  double lon_new;
  robot_localization::navsat_conversions::UTMtoLL(
    UTMNorthing, UTMEasting, UTMZone, lat_new, lon_new);
  EXPECT_NEAR(lat_new, lat, 1e-5);
  EXPECT_NEAR(lon_new, lon, 1e-5);
}

TEST(NavsatConversionsTest, UtmTest)
{
  NavsatConversionsTest(51.423964, 5.494271, 5699924.709, 673409.989, "31U", 1.950);
  NavsatConversionsTest(-43.530955, 172.636645, 5178919.718, 632246.802, "59G", -1.127);
}

// Helper to compute ECEF coordinates for a given geodetic position
void geodeticToECEF(double lat_deg, double lon_deg, double h_m, double & x, double & y, double & z)
{
  GeographicLib::Geocentric geo(GeographicLib::Constants::WGS84_a(),
    GeographicLib::Constants::WGS84_f());
  geo.Forward(lat_deg, lon_deg, h_m, x, y, z);
}

TEST(EarthToCartesianTest, OriginMapsToZero)
{
  // The geodetic origin should map to (0, 0, 0) in the local ENU frame
  const double lat = 40.0;
  const double lon = -74.0;
  const double alt = 100.0;

  tf2::Transform transform = robot_localization::computeEarthToCartesian(lat, lon, alt);

  // Get ECEF coordinates of the origin
  double x_ecef, y_ecef, z_ecef;
  geodeticToECEF(lat, lon, alt, x_ecef, y_ecef, z_ecef);

  // Transform the origin point to local frame
  tf2::Vector3 origin_ecef(x_ecef, y_ecef, z_ecef);
  tf2::Vector3 local = transform * origin_ecef;

  EXPECT_NEAR(local.x(), 0.0, 1e-6);
  EXPECT_NEAR(local.y(), 0.0, 1e-6);
  EXPECT_NEAR(local.z(), 0.0, 1e-6);
}

TEST(EarthToCartesianTest, RotationMatrixIsOrthonormal)
{
  tf2::Transform transform = robot_localization::computeEarthToCartesian(45.0, -122.0, 0.0);
  tf2::Matrix3x3 R = transform.getBasis();

  // R * R^T should equal identity
  tf2::Matrix3x3 RRt;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      double sum = 0.0;
      for (int k = 0; k < 3; ++k) {
        sum += R[i][k] * R[j][k];
      }
      RRt[i][j] = sum;
    }
  }

  // Check diagonal elements are 1
  EXPECT_NEAR(RRt[0][0], 1.0, 1e-10);
  EXPECT_NEAR(RRt[1][1], 1.0, 1e-10);
  EXPECT_NEAR(RRt[2][2], 1.0, 1e-10);

  // Check off-diagonal elements are 0
  EXPECT_NEAR(RRt[0][1], 0.0, 1e-10);
  EXPECT_NEAR(RRt[0][2], 0.0, 1e-10);
  EXPECT_NEAR(RRt[1][0], 0.0, 1e-10);
  EXPECT_NEAR(RRt[1][2], 0.0, 1e-10);
  EXPECT_NEAR(RRt[2][0], 0.0, 1e-10);
  EXPECT_NEAR(RRt[2][1], 0.0, 1e-10);
}

TEST(EarthToCartesianTest, RotationMatrixDeterminantIsOne)
{
  tf2::Transform transform = robot_localization::computeEarthToCartesian(45.0, -122.0, 0.0);
  tf2::Matrix3x3 R = transform.getBasis();

  // Compute determinant
  double det = R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]) -
    R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0]) +
    R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]);

  EXPECT_NEAR(det, 1.0, 1e-10);
}

TEST(EarthToCartesianTest, EquatorPrimeMeridian)
{
  // At equator/prime meridian (0, 0, 0):
  // - East points in +Y_ecef direction
  // - North points in +Z_ecef direction
  // - Up points in +X_ecef direction
  tf2::Transform transform = robot_localization::computeEarthToCartesian(0.0, 0.0, 0.0);
  tf2::Matrix3x3 R = transform.getBasis();

  // Row 0 is East axis in ECEF: should be (0, 1, 0)
  EXPECT_NEAR(R[0][0], 0.0, 1e-10);
  EXPECT_NEAR(R[0][1], 1.0, 1e-10);
  EXPECT_NEAR(R[0][2], 0.0, 1e-10);

  // Row 1 is North axis in ECEF: should be (0, 0, 1)
  EXPECT_NEAR(R[1][0], 0.0, 1e-10);
  EXPECT_NEAR(R[1][1], 0.0, 1e-10);
  EXPECT_NEAR(R[1][2], 1.0, 1e-10);

  // Row 2 is Up axis in ECEF: should be (1, 0, 0)
  EXPECT_NEAR(R[2][0], 1.0, 1e-10);
  EXPECT_NEAR(R[2][1], 0.0, 1e-10);
  EXPECT_NEAR(R[2][2], 0.0, 1e-10);
}

TEST(EarthToCartesianTest, NorthPole)
{
  // At north pole (90, 0, 0):
  // - Up points in +Z_ecef direction
  // - East/North are degenerate but should still form orthonormal basis
  tf2::Transform transform = robot_localization::computeEarthToCartesian(90.0, 0.0, 0.0);
  tf2::Matrix3x3 R = transform.getBasis();

  // Row 2 is Up axis: at north pole, should point in +Z_ecef direction
  EXPECT_NEAR(R[2][0], 0.0, 1e-10);
  EXPECT_NEAR(R[2][1], 0.0, 1e-10);
  EXPECT_NEAR(R[2][2], 1.0, 1e-10);

  // The matrix should still be orthonormal (tested separately, but verify here too)
  double det = R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]) -
    R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0]) +
    R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]);
  EXPECT_NEAR(det, 1.0, 1e-10);
}

TEST(EarthToCartesianTest, AxisDirectionsAtEquator)
{
  // At equator (0 lat) with 90 deg longitude:
  // - East points in -X_ecef direction
  // - North points in +Z_ecef direction
  // - Up points in +Y_ecef direction
  tf2::Transform transform = robot_localization::computeEarthToCartesian(0.0, 90.0, 0.0);
  tf2::Matrix3x3 R = transform.getBasis();

  // Row 0 is East: should be (-1, 0, 0)
  EXPECT_NEAR(R[0][0], -1.0, 1e-10);
  EXPECT_NEAR(R[0][1], 0.0, 1e-10);
  EXPECT_NEAR(R[0][2], 0.0, 1e-10);

  // Row 1 is North: should be (0, 0, 1)
  EXPECT_NEAR(R[1][0], 0.0, 1e-10);
  EXPECT_NEAR(R[1][1], 0.0, 1e-10);
  EXPECT_NEAR(R[1][2], 1.0, 1e-10);

  // Row 2 is Up: should be (0, 1, 0)
  EXPECT_NEAR(R[2][0], 0.0, 1e-10);
  EXPECT_NEAR(R[2][1], 1.0, 1e-10);
  EXPECT_NEAR(R[2][2], 0.0, 1e-10);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
