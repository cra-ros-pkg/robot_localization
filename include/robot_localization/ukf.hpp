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
#ifndef ROBOT_LOCALIZATION__UKF_HPP_
#define ROBOT_LOCALIZATION__UKF_HPP_

#include <vector>

#include "Eigen/Dense"
#include "rclcpp/time.hpp"
#include "robot_localization/filter_base.hpp"
#include "robot_localization/measurement.hpp"

namespace robot_localization
{

/**
 * @brief  Unscented Kalman filter class
 *
 * Implementation of an unscenter Kalman filter (UKF). This class derives from
 * FilterBase and overrides the predict() and correct() methods in keeping with
 * the discrete time UKF algorithm. The algorithm was derived from the UKF
 * Wikipedia article at
 * http://en.wikipedia.org/wiki/Kalman_filter#Unscented_Kalman_filter
 * as well as this paper:
 * J. J. LaViola, Jr., “A comparison of unscented and extended Kalman filtering
 * for estimating quaternion motion,” in Proc. American Control Conf., Denver,
 * CO, June 4–6, 2003, pp. 2435–2440 Obtained here:
 * http://www.cs.ucf.edu/~jjl/pubs/laviola_acc2003.pdf
 */
class Ukf : public FilterBase
{
public:
  /**
   * @brief  Constructor for the Ukf class
   *
   * @param[in] args - Generic argument container. It is assumed that args[0]
   * constains the alpha parameter, args[1] contains the kappa parameter, and
   * args[2] contains the beta parameter.
   */
  Ukf();

  /**
   * @brief  Destructor for the Ukf class
   */
  ~Ukf();

  void setConstants(double alpha, double kappa, double beta);

  /**
   * @brief  Carries out the correct step in the predict/update cycle.
   *
   * @param[in] measurement - The measurement to fuse with our estimate
   */
  void correct(const Measurement & measurement) override;

  /**
   * @brief  Carries out the predict step in the predict/update cycle.
   *
   * Projects the state and error matrices forward using a model of the
   * vehicle's motion.
   *
   * @param[in] reference_time - The time at which the prediction is being made
   * @param[in] delta - The time step over which to predict.
   */
  void predict(
    const rclcpp::Time & reference_time,
    const rclcpp::Duration & delta) override;

protected:
  /**
   * @brief  Computes the weighted covariance and sigma points
   */
  void generateSigmaPoints();

  /**
   * @brief  Carries out the predict step for the posteriori state of a sigma point
   * @param[in,out] sigma_point - The sigma point (state vector) to project
   * @param[in] delta - The time step over which to project
   */
  void projectSigmaPoint(Eigen::VectorXd & sigma_point, const rclcpp::Duration & delta);

  /**
   * @brief  The UKF sigma points
   *
   * Used to sample possible next states during prediction.
   */
  std::vector<Eigen::VectorXd> sigma_points_;

  /**
   * @brief  This matrix is used to generate the sigmaPoints_
   */
  Eigen::MatrixXd weighted_covar_sqrt_;

  /**
   * @brief  The weights associated with each sigma point when generating a new
   * state
   */
  std::vector<double> state_weights_;

  /**
   * @brief  The weights associated with each sigma point when calculating a
   * predicted estimateErrorCovariance_
   */
  std::vector<double> covar_weights_;

  /**
   * @brief  Used in weight generation for the sigma points
   */
  double lambda_;

  /**
   * @brief  Used to determine if we need to re-compute the sigma points when
   * carrying out multiple corrections
   */
  bool uncorrected_;
};

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__UKF_HPP_
