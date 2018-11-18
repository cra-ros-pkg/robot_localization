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

#include "robot_localization/ukf.h"
#include "robot_localization/filter_common.h"

#include <XmlRpcException.h>

#include <sstream>
#include <iomanip>
#include <limits>

#include <Eigen/Cholesky>

#include <iostream>
#include <vector>

#include <assert.h>

namespace RobotLocalization
{
  Ukf::Ukf(std::vector<double> args) :
    FilterBase(),  // Must initialize filter base!
    uncorrected_(true)
  {
    assert(args.size() == 3);

    double alpha = args[0];
    double kappa = args[1];
    double beta = args[2];

    // size_t sigmaCount = (STATE_SIZE << 1) + 1;
    size_t sigmaCount = this->pointsPerState(STATE_SIZE);
    sigmaPoints_.resize(sigmaCount, Eigen::VectorXd(STATE_SIZE));

    // Prepare constants
    // lambda_ = alpha * alpha * (STATE_SIZE + kappa) - STATE_SIZE;
    this->lambda_ = this->calculateLambda(alpha, kappa, STATE_SIZE);
    this->gamma_ = this->calculateGamma(this->lambda_, STATE_SIZE);

    stateWeights_.resize(sigmaCount);
    covarWeights_.resize(sigmaCount);

    stateWeights_[0] = lambda_ / (STATE_SIZE + lambda_);
    covarWeights_[0] = stateWeights_[0] + (1 - (alpha * alpha) + beta);
    sigmaPoints_[0].setZero();
    for (size_t i = 1; i < sigmaCount; ++i)
    {
      sigmaPoints_[i].setZero();
      stateWeights_[i] =  1 / (2 * (STATE_SIZE + lambda_));
      covarWeights_[i] = stateWeights_[i];
    }
  }

  Ukf::~Ukf()
  {
  }



  void Ukf::sigmaPoints(
    std::vector<Eigen::VectorXd>& sigma,
    const Eigen::VectorXd& state,
    const Eigen::MatrixXd& covariance,
    Eigen::MatrixXd& cholesky,
    const double gamma)
  {
    cholesky = covariance.llt().matrixL();
    cholesky *= gamma;
    sigma[0] = state;
    int vars = state.size();
    for (int i = 0; i < vars; ++i) {
      sigma[i+1] = state+cholesky.col(i);
      sigma[i+vars+1] = state-cholesky.col(i);
    }
  }
  
  
  void Ukf::sumWeightedMean(
    Eigen::VectorXd& state,
    const std::vector<Eigen::VectorXd>& sigma,
    const std::vector<double>& weights)
  {
    state.setZero();
    int points = sigma.size();
    for (int i = 0; i < points; ++i)
      state.noalias() += sigma[i] * weights[i];
  }


  inline void Ukf::sumWeightedCovariance(
    Eigen::MatrixXd& covariance,
    const std::vector<Eigen::VectorXd>& sigma,
    const Eigen::VectorXd& belief,
    const std::vector<double>& weight,
    const Eigen::MatrixXd& noise)
  {
    this->sumWeightedCovariance(
      covariance,
      sigma, belief,
      sigma, belief,
      weight);
    covariance.noalias() += noise;
  }


  void Ukf::sumWeightedCovariance(
    Eigen::MatrixXd& covariance,
    const std::vector<Eigen::VectorXd>& sigma_x,
    const Eigen::VectorXd& state_a,
    const std::vector<Eigen::VectorXd>& sigma_z,
    const Eigen::VectorXd& state_b,
    const std::vector<double>& weight)
  {
    covariance.setZero();
    int rows = sigma_x[0].rows(),
        inner = sigma_x.size(),
        cols = sigma_z[0].rows();
    for (int i = 0; i < rows; ++i)
      for (int j = 0; j < cols; ++j)
        for (int k = 0; k < inner; ++k)
          covariance(i,j) += 
            weight[k] *
              (sigma_x[k](i)-state_a(i)) *
              (sigma_z[k](j)-state_b(j));
  }


  void Ukf::kalmanGain(
    Eigen::MatrixXd& gain,
    const Eigen::MatrixXd& covar,
    const Eigen::MatrixXd& obser,
    Eigen::MatrixXd& obser_inv)
  {
	obser_inv = obser.inverse();
    gain = covar * obser_inv;
  }
  
  
  void Ukf::updateState(
    Eigen::VectorXd& state,
    const Eigen::VectorXd& innov,
    const Eigen::MatrixXd& gain)
  {
    state.noalias() += gain * innov;
  }


  void Ukf::updateCovariance(
    Eigen::MatrixXd& covar,
    const Eigen::MatrixXd& gain,
    const Eigen::MatrixXd& obser)
  {
    covar.noalias() -= gain * obser * gain.transpose();
  }


  void Ukf::correct(const Measurement &measurement)
  {
    FB_DEBUG("---------------------- Ukf::correct ----------------------\n" <<
             "State is:\n" << state_ <<
             "\nMeasurement is:\n" << measurement.measurement_ <<
             "\nMeasurement covariance is:\n" << measurement.covariance_ << "\n");

    // In our implementation, it may be that after we call predict once, we call correct
    // several times in succession (multiple measurements with different time stamps). In
    // that event, the sigma points need to be updated to reflect the current state.
    // Throughout prediction and correction, we attempt to maximize efficiency in Eigen.
    if (!uncorrected_)
      this->sigmaPoints(this->sigmaPoints_, this->state_,
		this->estimateErrorCovariance_, this->weightedCovarSqrt_, this->gamma_);

    // We don't want to update everything, so we need to build matrices that only update
    // the measured parts of our state vector

    // First, determine how many state vector values we're updating
    std::vector<size_t> updateIndices;
    for (size_t i = 0; i < measurement.updateVector_.size(); ++i)
    {
      if (measurement.updateVector_[i])
      {
        // Handle nan and inf values in measurements
        if (std::isnan(measurement.measurement_(i)))
        {
          FB_DEBUG("Value at index " << i << " was nan. Excluding from update.\n");
        }
        else if (std::isinf(measurement.measurement_(i)))
        {
          FB_DEBUG("Value at index " << i << " was inf. Excluding from update.\n");
        }
        else
        {
          updateIndices.push_back(i);
        }
      }
    }

    FB_DEBUG("Update indices are:\n" << updateIndices << "\n");

    size_t updateSize = updateIndices.size();

    // Now set up the relevant matrices
    Eigen::VectorXd stateSubset(updateSize);                              // x (in most literature)
    Eigen::VectorXd measurementSubset(updateSize);                        // z
    Eigen::MatrixXd measurementCovarianceSubset(updateSize, updateSize);  // R
    Eigen::MatrixXd stateToMeasurementSubset(updateSize, STATE_SIZE);     // H
    Eigen::MatrixXd kalmanGainSubset(STATE_SIZE, updateSize);             // K
    Eigen::VectorXd innovationSubset(updateSize);                         // z - Hx
    Eigen::VectorXd predictedMeasurement(updateSize);
    Eigen::VectorXd sigmaDiff(updateSize);
    Eigen::MatrixXd predictedMeasCovar(updateSize, updateSize);
    Eigen::MatrixXd crossCovar(STATE_SIZE, updateSize);

    std::vector<Eigen::VectorXd> sigmaPointMeasurements(sigmaPoints_.size(), Eigen::VectorXd(updateSize));

    stateSubset.setZero();
    measurementSubset.setZero();
    measurementCovarianceSubset.setZero();
    stateToMeasurementSubset.setZero();
    kalmanGainSubset.setZero();
    innovationSubset.setZero();
    predictedMeasurement.setZero();
    predictedMeasCovar.setZero();
    crossCovar.setZero();

    // Now build the sub-matrices from the full-sized matrices
    for (size_t i = 0; i < updateSize; ++i)
    {
      measurementSubset(i) = measurement.measurement_(updateIndices[i]);
      stateSubset(i) = state_(updateIndices[i]);

      for (size_t j = 0; j < updateSize; ++j)
      {
        measurementCovarianceSubset(i, j) = measurement.covariance_(updateIndices[i], updateIndices[j]);
      }

      // Handle negative (read: bad) covariances in the measurement. Rather
      // than exclude the measurement or make up a covariance, just take
      // the absolute value.
      if (measurementCovarianceSubset(i, i) < 0)
      {
        FB_DEBUG("WARNING: Negative covariance for index " << i <<
                 " of measurement (value is" << measurementCovarianceSubset(i, i) <<
                 "). Using absolute value...\n");

        measurementCovarianceSubset(i, i) = ::fabs(measurementCovarianceSubset(i, i));
      }

      // If the measurement variance for a given variable is very
      // near 0 (as in e-50 or so) and the variance for that
      // variable in the covariance matrix is also near zero, then
      // the Kalman gain computation will blow up. Really, no
      // measurement can be completely without error, so add a small
      // amount in that case.
      if (measurementCovarianceSubset(i, i) < 1e-9)
      {
        measurementCovarianceSubset(i, i) = 1e-9;

        FB_DEBUG("WARNING: measurement had very small error covariance for index " <<
                 updateIndices[i] <<
                 ". Adding some noise to maintain filter stability.\n");
      }
    }

    // The state-to-measurement function, h, will now be a measurement_size x full_state_size
    // matrix, with ones in the (i, i) locations of the values to be updated
    for (size_t i = 0; i < updateSize; ++i)
    {
      stateToMeasurementSubset(i, updateIndices[i]) = 1;
    }

    FB_DEBUG("Current state subset is:\n" << stateSubset <<
             "\nMeasurement subset is:\n" << measurementSubset <<
             "\nMeasurement covariance subset is:\n" << measurementCovarianceSubset <<
             "\nState-to-measurement subset is:\n" << stateToMeasurementSubset << "\n");
    
    // h function
    int points = sigmaPoints_.size();
    for (int i = 0; i < points; ++i)
        sigmaPointMeasurements[i] = stateToMeasurementSubset * sigmaPoints_[i];
	
	this->sumWeightedMean(
		predictedMeasurement,
		sigmaPointMeasurements,
		stateWeights_);
    
    this->sumWeightedCovariance(
      predictedMeasCovar,
      sigmaPointMeasurements,
      predictedMeasurement,
      covarWeights_,
      measurementCovarianceSubset);
    
    this->sumWeightedCovariance(
      crossCovar, 
      sigmaPoints_,
      state_,
      sigmaPointMeasurements, 
      predictedMeasurement,
      covarWeights_);

    Eigen::MatrixXd invInnovCov(updateSize, updateSize);
    this->kalmanGain(kalmanGainSubset,
	  crossCovar, predictedMeasCovar, invInnovCov);
    
    innovationSubset = (measurementSubset - predictedMeasurement);

    // Wrap angles in the innovation
    for (size_t i = 0; i < updateSize; ++i)
    {
      if (updateIndices[i] == StateMemberRoll  ||
          updateIndices[i] == StateMemberPitch ||
          updateIndices[i] == StateMemberYaw)
      {
        while (innovationSubset(i) < -PI)
        {
          innovationSubset(i) += TAU;
        }

        while (innovationSubset(i) > PI)
        {
          innovationSubset(i) -= TAU;
        }
      }
    }
    

    // (5) Check Mahalanobis distance of innovation
    if (checkMahalanobisThreshold(innovationSubset, invInnovCov, measurement.mahalanobisThresh_))
    {
      this->updateState(this->state_, innovationSubset, kalmanGainSubset);
      this->updateCovariance(
		this->estimateErrorCovariance_,
		kalmanGainSubset, predictedMeasCovar);

      wrapStateAngles();

      // Mark that we need to re-compute sigma points for successive corrections
      uncorrected_ = false;

      FB_DEBUG("Predicated measurement covariance is:\n" << predictedMeasCovar <<
               "\nCross covariance is:\n" << crossCovar <<
               "\nKalman gain subset is:\n" << kalmanGainSubset <<
               "\nInnovation:\n" << innovationSubset <<
               "\nCorrected full state is:\n" << state_ <<
               "\nCorrected full estimate error covariance is:\n" << estimateErrorCovariance_ <<
               "\n\n---------------------- /Ukf::correct ----------------------\n");
    }
  }
  
  
  void Ukf::predict(const double referenceTime, const double delta)
  {
    FB_DEBUG("---------------------- Ukf::predict ----------------------\n" <<
             "delta is " << delta <<
             "\nstate is " << state_ << "\n");

    double roll = state_(StateMemberRoll);
    double pitch = state_(StateMemberPitch);
    double yaw = state_(StateMemberYaw);

    // We'll need these trig calculations a lot.
    double sp = ::sin(pitch);
    double cp = ::cos(pitch);

    double sr = ::sin(roll);
    double cr = ::cos(roll);

    double sy = ::sin(yaw);
    double cy = ::cos(yaw);

    prepareControl(referenceTime, delta);

    // Prepare the transfer function
    transferFunction_(StateMemberX, StateMemberVx) = cy * cp * delta;
    transferFunction_(StateMemberX, StateMemberVy) = (cy * sp * sr - sy * cr) * delta;
    transferFunction_(StateMemberX, StateMemberVz) = (cy * sp * cr + sy * sr) * delta;
    transferFunction_(StateMemberX, StateMemberAx) = 0.5 * transferFunction_(StateMemberX, StateMemberVx) * delta;
    transferFunction_(StateMemberX, StateMemberAy) = 0.5 * transferFunction_(StateMemberX, StateMemberVy) * delta;
    transferFunction_(StateMemberX, StateMemberAz) = 0.5 * transferFunction_(StateMemberX, StateMemberVz) * delta;
    transferFunction_(StateMemberY, StateMemberVx) = sy * cp * delta;
    transferFunction_(StateMemberY, StateMemberVy) = (sy * sp * sr + cy * cr) * delta;
    transferFunction_(StateMemberY, StateMemberVz) = (sy * sp * cr - cy * sr) * delta;
    transferFunction_(StateMemberY, StateMemberAx) = 0.5 * transferFunction_(StateMemberY, StateMemberVx) * delta;
    transferFunction_(StateMemberY, StateMemberAy) = 0.5 * transferFunction_(StateMemberY, StateMemberVy) * delta;
    transferFunction_(StateMemberY, StateMemberAz) = 0.5 * transferFunction_(StateMemberY, StateMemberVz) * delta;
    transferFunction_(StateMemberZ, StateMemberVx) = -sp * delta;
    transferFunction_(StateMemberZ, StateMemberVy) = cp * sr * delta;
    transferFunction_(StateMemberZ, StateMemberVz) = cp * cr * delta;
    transferFunction_(StateMemberZ, StateMemberAx) = 0.5 * transferFunction_(StateMemberZ, StateMemberVx) * delta;
    transferFunction_(StateMemberZ, StateMemberAy) = 0.5 * transferFunction_(StateMemberZ, StateMemberVy) * delta;
    transferFunction_(StateMemberZ, StateMemberAz) = 0.5 * transferFunction_(StateMemberZ, StateMemberVz) * delta;
    transferFunction_(StateMemberRoll, StateMemberVroll) = transferFunction_(StateMemberX, StateMemberVx);
    transferFunction_(StateMemberRoll, StateMemberVpitch) = transferFunction_(StateMemberX, StateMemberVy);
    transferFunction_(StateMemberRoll, StateMemberVyaw) = transferFunction_(StateMemberX, StateMemberVz);
    transferFunction_(StateMemberPitch, StateMemberVroll) = transferFunction_(StateMemberY, StateMemberVx);
    transferFunction_(StateMemberPitch, StateMemberVpitch) = transferFunction_(StateMemberY, StateMemberVy);
    transferFunction_(StateMemberPitch, StateMemberVyaw) = transferFunction_(StateMemberY, StateMemberVz);
    transferFunction_(StateMemberYaw, StateMemberVroll) = transferFunction_(StateMemberZ, StateMemberVx);
    transferFunction_(StateMemberYaw, StateMemberVpitch) = transferFunction_(StateMemberZ, StateMemberVy);
    transferFunction_(StateMemberYaw, StateMemberVyaw) = transferFunction_(StateMemberZ, StateMemberVz);
    transferFunction_(StateMemberVx, StateMemberAx) = delta;
    transferFunction_(StateMemberVy, StateMemberAy) = delta;
    transferFunction_(StateMemberVz, StateMemberAz) = delta;

    this->sigmaPoints(
		this->sigmaPoints_, this->state_, 
		this->estimateErrorCovariance_, 
		this->weightedCovarSqrt_, this->gamma_);
    
    // process model
    int points = this->sigmaPoints_.size();
    for (int i = 0; i < points; ++i)
		this->sigmaPoints_[i] = this->transferFunction_ * this->sigmaPoints_[i];
    
    // (5) Not strictly in the theoretical UKF formulation, but necessary here
    // to ensure that we actually incorporate the processNoiseCovariance_
    Eigen::MatrixXd *processNoiseCovariance = &processNoiseCovariance_;
    if (useDynamicProcessNoiseCovariance_) {
      computeDynamicProcessNoiseCovariance(state_, delta);
      processNoiseCovariance = &dynamicProcessNoiseCovariance_;
    }
    
    this->sumWeightedMean(this->state_, this->sigmaPoints_, this->stateWeights_);
    this->sumWeightedCovariance(this->estimateErrorCovariance_,
      this->sigmaPoints_, this->state_, this->covarWeights_,
      delta * (*processNoiseCovariance));
    
    // Keep the angles bounded
    wrapStateAngles();

    // Mark that we can keep these sigma points
    uncorrected_ = true;

    FB_DEBUG("Predicted state is:\n" << state_ <<
             "\nPredicted estimate error covariance is:\n" << estimateErrorCovariance_ <<
             "\n\n--------------------- /Ukf::predict ----------------------\n");
  }

}  // namespace RobotLocalization
