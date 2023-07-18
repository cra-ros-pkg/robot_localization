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

#include <gtest/gtest.h>
#include <chrono>
#include <robot_localization/filter_base.hpp>
#include <vector>
#include <memory>

#include "robot_localization/ekf.hpp"
#include "robot_localization/measurement.hpp"
#include "robot_localization/ros_filter_types.hpp"

using robot_localization::Measurement;
using robot_localization::Ekf;
using robot_localization::RosEkf;
using robot_localization::STATE_SIZE;

Measurement createMeasurement(std::string topic_name, rclcpp::Time time, Eigen::VectorXd measurement, Eigen::MatrixXd covariance, std::vector<bool> update_vector, double mahalanobis_thresh = 2.0) {
  Measurement m;
  m.topic_name_ = std::move(topic_name);
  m.measurement_ = std::move(measurement);
  m.covariance_ = std::move(covariance);
  m.update_vector_ = std::move(update_vector);
  m.time_ = std::move(time);
  m.mahalanobis_thresh_ = mahalanobis_thresh;
  return m;
}

Measurement create_position_measurement(std::string topic_name, rclcpp::Time time, Eigen::Vector3d position, double pos_var) {
  Eigen::VectorXd measurement = Eigen::VectorXd::Zero(STATE_SIZE);
  measurement(0) = position.x();
  measurement(1) = position.y();
  measurement(2) = position.z();

  Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE);
  cov *= 1e9;
  cov(0,0) = pos_var;
  cov(1,1) = pos_var;
  cov(2,2) = pos_var;

  std::vector<bool> pos_update_vec{
    true, true, true, 
    false, false, false, 
    false, false, false, 
    false, false, false,
    false, false, false
  };

  return createMeasurement(topic_name, time, measurement, cov, pos_update_vec);
}

TEST(LSP_EkfTest, EkfCorrectShouldReturnFalseForOutlier) {
  Eigen::MatrixXd initialCov(15, 15);
  initialCov.setIdentity();
  initialCov *= 0.25;

  Ekf filter;
  filter.setEstimateErrorCovariance(initialCov);

  { // Initialize the filter
    Measurement init_m = create_position_measurement("odom0", rclcpp::Time(1000), Eigen::Vector3d(0.0, 0.0, 0.0), 0.25);
    
    filter.processMeasurement(init_m);
  }
  
  { // Create a measurement that is valid
    Measurement valid_m = create_position_measurement("odom0", rclcpp::Time(1001), Eigen::Vector3d(0.0, 0.0, 0.0), 0.25);
    auto is_applied = filter.processMeasurement(valid_m);
    EXPECT_TRUE(is_applied);
  }

  { // Create an invalid measurement
    Measurement invalid_m = create_position_measurement("odom0", rclcpp::Time(1000), Eigen::Vector3d(100.0, 0.0, 0.0), 0.25);
    auto is_applied = filter.processMeasurement(invalid_m);
    EXPECT_FALSE(is_applied);
  }
}

std::vector<rclcpp::Parameter> create_parameters_override(bool use_state_lock_protection){
  std::vector<rclcpp::Parameter> params;

  Eigen::MatrixXd initialCov = Eigen::MatrixXd::Identity(15, 15);
  initialCov *= 0.25;
  std::vector<double> initialCovVec(initialCov.data(), initialCov.data() + initialCov.size());

  params.emplace_back("initial_estimate_covariance", initialCovVec);
  params.emplace_back("predict_to_current_time", true);

  std::vector<double> state_lock_var = {1.e8, 1.e9, 1.e10, 1.e9, 1.e9, 1.e9, 1.e9, 1.e9, 1.e9, 1.e9, 1.e9, 1.e9, 1.e9, 1.e9, 1.e9};
  params.emplace_back("use_state_lock_protection", use_state_lock_protection);
  params.emplace_back("state_lock_protection_threshold_s", 2.0);
  params.emplace_back("state_lock_protection_variance", state_lock_var);

  return params;
}

TEST(LSP_EkfTest, LockProtectionEnabled_RejectedMeasurementShouldApplyStateLockProtection) {
  using namespace std::chrono_literals;
  auto time = [](std::chrono::nanoseconds ns) {
    return rclcpp::Time(ns.count());
  };
  // node handle is created as per ros2
  rclcpp::NodeOptions options;
  options.arguments({"ekf_filter_node"});
  options.parameter_overrides(create_parameters_override(true));

  std::shared_ptr<robot_localization::RosEkf> filter =
    std::make_shared<robot_localization::RosEkf>(options);
  filter->initialize();

  { // Initialize the filter
    Measurement init_m = create_position_measurement("odom0", time(1s), Eigen::Vector3d(0.0, 0.0, 0.0), 100.0);
    filter->enqueueMeasurement(init_m.topic_name_, init_m.measurement_, init_m.covariance_, init_m.update_vector_, init_m.mahalanobis_thresh_, init_m.time_);
  }

  { // Create a measurement that is valid
    Measurement valid_m = create_position_measurement("odom0", time(2s), Eigen::Vector3d(1.0, 2.0, 0.0), 0.1*0.1);
    filter->enqueueMeasurement(valid_m.topic_name_, valid_m.measurement_, valid_m.covariance_, valid_m.update_vector_, valid_m.mahalanobis_thresh_, valid_m.time_);
  }

  { // Create an invalid measurement
    Measurement invalid_m = create_position_measurement("odom0", time(3s), Eigen::Vector3d(100.0, 0.0, 0.0), 0.25);
    filter->enqueueMeasurement(invalid_m.topic_name_, invalid_m.measurement_, invalid_m.covariance_, invalid_m.update_vector_, invalid_m.mahalanobis_thresh_, invalid_m.time_);
  }

  filter->robot_localization::RosEkf::integrateMeasurements(time(5s));

  auto state = filter->getFilter().getState();
  EXPECT_NEAR(state[0], 1.0, 0.1);
  EXPECT_NEAR(state[1], 2.0, 0.1);

  auto cov = filter->getFilter().getEstimateErrorCovariance();

  EXPECT_DOUBLE_EQ(cov(0,0), 1.e8);
  EXPECT_DOUBLE_EQ(cov(1,1), 1.e9);
  EXPECT_DOUBLE_EQ(cov(2,2), 1.e10);
};

TEST(LSP_EkfTest, LockProtectionEnabled_ValidMeasurementShouldNotApplyStateLockProtection) {
  using namespace std::chrono_literals;
  auto time = [](std::chrono::nanoseconds ns) {
    return rclcpp::Time(ns.count());
  };
  // node handle is created as per ros2
  rclcpp::NodeOptions options;
  options.arguments({"ekf_filter_node"});
  options.parameter_overrides(create_parameters_override(true));

  std::shared_ptr<robot_localization::RosEkf> filter =
    std::make_shared<robot_localization::RosEkf>(options);
  filter->initialize();

  { // Initialize the filter
    Measurement init_m = create_position_measurement("odom0", time(1s), Eigen::Vector3d(0.0, 0.0, 0.0), 100.0);
    filter->enqueueMeasurement(init_m.topic_name_, init_m.measurement_, init_m.covariance_, init_m.update_vector_, init_m.mahalanobis_thresh_, init_m.time_);
  }

  { // Create a measurement that is valid
    Measurement valid_m = create_position_measurement("odom0", time(2s), Eigen::Vector3d(0.0, 0.0, 0.0), 0.1*0.1);
    filter->enqueueMeasurement(valid_m.topic_name_, valid_m.measurement_, valid_m.covariance_, valid_m.update_vector_, valid_m.mahalanobis_thresh_, valid_m.time_);
  }

  { // Create an invalid measurement
    Measurement invalid_m = create_position_measurement("odom0", time(3s), Eigen::Vector3d(0.0, 0.0, 0.0), 0.25);
    filter->enqueueMeasurement(invalid_m.topic_name_, invalid_m.measurement_, invalid_m.covariance_, invalid_m.update_vector_, invalid_m.mahalanobis_thresh_, invalid_m.time_);
  }

  filter->robot_localization::RosEkf::integrateMeasurements(time(5s));

  auto state = filter->getFilter().getState();
  EXPECT_NEAR(state[0], 0.0, 0.1);
  EXPECT_NEAR(state[1], 0.0, 0.1);

  auto cov = filter->getFilter().getEstimateErrorCovariance();

  EXPECT_LT(cov(0,0), 10.0);
  EXPECT_LT(cov(1,1), 10.0);
  EXPECT_LT(cov(2,2), 10.0);
};


TEST(LSP_EkfTest, LockProtectionDisabled_ShouldNotApplyStateLockProtection) {
  using namespace std::chrono_literals;
  auto time = [](std::chrono::nanoseconds ns) {
    return rclcpp::Time(ns.count());
  };
  // node handle is created as per ros2
  rclcpp::NodeOptions options;
  options.arguments({"ekf_filter_node"});
  options.parameter_overrides(create_parameters_override(false));

  std::shared_ptr<robot_localization::RosEkf> filter =
    std::make_shared<robot_localization::RosEkf>(options);
  filter->initialize();

  { // Initialize the filter
    Measurement init_m = create_position_measurement("odom0", time(1s), Eigen::Vector3d(0.0, 0.0, 0.0), 100.0);
    filter->enqueueMeasurement(init_m.topic_name_, init_m.measurement_, init_m.covariance_, init_m.update_vector_, init_m.mahalanobis_thresh_, init_m.time_);
  }

  { // Create a measurement that is valid
    Measurement valid_m = create_position_measurement("odom0", time(2s), Eigen::Vector3d(1.0, 2.0, 0.0), 0.1*0.1);
    filter->enqueueMeasurement(valid_m.topic_name_, valid_m.measurement_, valid_m.covariance_, valid_m.update_vector_, valid_m.mahalanobis_thresh_, valid_m.time_);
  }

  { // Create an invalid measurement
    Measurement invalid_m = create_position_measurement("odom0", time(3s), Eigen::Vector3d(100.0, 0.0, 0.0), 0.25);
    filter->enqueueMeasurement(invalid_m.topic_name_, invalid_m.measurement_, invalid_m.covariance_, invalid_m.update_vector_, invalid_m.mahalanobis_thresh_, invalid_m.time_);
  }

  filter->robot_localization::RosEkf::integrateMeasurements(time(5s));

  auto state = filter->getFilter().getState();
  EXPECT_NEAR(state[0], 1.0, 0.1);
  EXPECT_NEAR(state[1], 2.0, 0.1);

  auto cov = filter->getFilter().getEstimateErrorCovariance();

  EXPECT_LT(cov(0,0), 20.0);
  EXPECT_LT(cov(1,1), 20.0);
  EXPECT_LT(cov(2,2), 20.0);
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return ret;
}
