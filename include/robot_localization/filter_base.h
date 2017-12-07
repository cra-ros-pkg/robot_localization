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

#ifndef ROBOT_LOCALIZATION_FILTER_BASE_H
#define ROBOT_LOCALIZATION_FILTER_BASE_H

#include "robot_localization/filter_utilities.h"
#include "robot_localization/filter_common.h"

#include <Eigen/Dense>

#include <algorithm>
#include <limits>
#include <map>
#include <ostream>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

namespace RobotLocalization
{

//! @brief Structure used for storing and comparing measurements
//! (for priority queues)
//!
//! Measurement units are assumed to be in meters and radians.
//! Times are real-valued and measured in seconds.
//!
struct Measurement
{
  // The topic name for this measurement. Needed
  // for capturing previous state values for new
  // measurements.
  std::string topicName_;

  // The measurement and its associated covariance
  Eigen::VectorXd measurement_;
  Eigen::MatrixXd covariance_;

  // This defines which variables within this measurement
  // actually get passed into the filter. std::vector<bool>
  // is generally frowned upon, so we use ints.
  std::vector<int> updateVector_;

  // The real-valued time, in seconds, since some epoch
  // (presumably the start of execution, but any will do)
  double time_;

  // The Mahalanobis distance threshold in number of sigmas
  double mahalanobisThresh_;

  // The most recent control vector (needed for lagged data)
  Eigen::VectorXd latestControl_;

  // The time stamp of the most recent control term (needed for lagged data)
  double latestControlTime_;

  // We want earlier times to have greater priority
  bool operator()(const boost::shared_ptr<Measurement> &a, const boost::shared_ptr<Measurement> &b)
  {
    return (*this)(*(a.get()), *(b.get()));
  }

  bool operator()(const Measurement &a, const Measurement &b)
  {
    return a.time_ > b.time_;
  }

  Measurement() :
    topicName_(""),
    latestControl_(),
    latestControlTime_(0.0),
    time_(0.0),
    mahalanobisThresh_(std::numeric_limits<double>::max())
  {
  }
};
typedef boost::shared_ptr<Measurement> MeasurementPtr;

//! @brief Structure used for storing and comparing filter states
//!
//! This structure is useful when higher-level classes need to remember filter history.
//! Measurement units are assumed to be in meters and radians.
//! Times are real-valued and measured in seconds.
//!
struct FilterState
{
  // The filter state vector
  Eigen::VectorXd state_;

  // The filter error covariance matrix
  Eigen::MatrixXd estimateErrorCovariance_;

  // The most recent control vector
  Eigen::VectorXd latestControl_;

  // The time stamp of the most recent measurement for the filter
  double lastMeasurementTime_;

  // The time stamp of the most recent control term
  double latestControlTime_;

  // We want the queue to be sorted from latest to earliest timestamps.
  bool operator()(const FilterState &a, const FilterState &b)
  {
    return a.lastMeasurementTime_ < b.lastMeasurementTime_;
  }

  FilterState() :
    state_(),
    estimateErrorCovariance_(),
    latestControl_(),
    lastMeasurementTime_(0.0),
    latestControlTime_(0.0)
  {}
};
typedef boost::shared_ptr<FilterState> FilterStatePtr;

class FilterBase
{
  public:
    //! @brief Constructor for the FilterBase class
    //!
    FilterBase();

    //! @brief Destructor for the FilterBase class
    //!
    virtual ~FilterBase();

    //! @brief Resets filter to its unintialized state
    //!
    void reset();

    //! @brief Computes a dynamic process noise covariance matrix using the parameterized state
    //!
    //! This allows us to, e.g., not increase the pose covariance values when the vehicle is not moving
    //!
    //! @param[in] state - The STATE_SIZE state vector that is used to generate the dynamic process noise covariance
    //!
    void computeDynamicProcessNoiseCovariance(const Eigen::VectorXd &state, const double delta);

    //! @brief Carries out the correct step in the predict/update cycle. This method
    //! must be implemented by subclasses.
    //!
    //! @param[in] measurement - The measurement to fuse with the state estimate
    //!
    virtual void correct(const Measurement &measurement) = 0;

    //! @brief Returns the control vector currently being used
    //!
    //! @return The control vector
    //!
    const Eigen::VectorXd& getControl();

    //! @brief Returns the time at which the control term was issued
    //!
    //! @return The time the control vector was issued
    //!
    double getControlTime();

    //! @brief Gets the value of the debug_ variable.
    //!
    //! @return True if in debug mode, false otherwise
    //!
    bool getDebug();

    //! @brief Gets the estimate error covariance
    //!
    //! @return A copy of the estimate error covariance matrix
    //!
    const Eigen::MatrixXd& getEstimateErrorCovariance();

    //! @brief Gets the filter's initialized status
    //!
    //! @return True if we've received our first measurement, false otherwise
    //!
    bool getInitializedStatus();

    //! @brief Gets the most recent measurement time
    //!
    //! @return The time at which we last received a measurement
    //!
    double getLastMeasurementTime();

    //! @brief Gets the filter's predicted state, i.e., the
    //! state estimate before correct() is called.
    //!
    //! @return A constant reference to the predicted state
    //!
    const Eigen::VectorXd& getPredictedState();

    //! @brief Gets the filter's process noise covariance
    //!
    //! @return A constant reference to the process noise covariance
    //!
    const Eigen::MatrixXd& getProcessNoiseCovariance();

    //! @brief Gets the sensor timeout value (in seconds)
    //!
    //! @return The sensor timeout value
    //!
    double getSensorTimeout();

    //! @brief Gets the filter state
    //!
    //! @return A constant reference to the current state
    //!
    const Eigen::VectorXd& getState();

    //! @brief Carries out the predict step in the predict/update cycle.
    //! Projects the state and error matrices forward using a model of
    //! the vehicle's motion. This method must be implemented by subclasses.
    //!
    //! @param[in] referenceTime - The time at which the prediction is being made
    //! @param[in] delta - The time step over which to predict.
    //!
    virtual void predict(const double referenceTime, const double delta) = 0;

    //! @brief Does some final preprocessing, carries out the predict/update cycle
    //!
    //! @param[in] measurement - The measurement object to fuse into the filter
    //!
    virtual void processMeasurement(const Measurement &measurement);

    //! @brief Sets the most recent control term
    //!
    //! @param[in] control - The control term to be applied
    //! @param[in] controlTime - The time at which the control in question was received
    //!
    void setControl(const Eigen::VectorXd &control, const double controlTime);

    //! @brief Sets the control update vector and acceleration limits
    //!
    //! @param[in] updateVector - The values the control term affects
    //! @param[in] controlTimeout - Timeout value, in seconds, after which a control is considered stale
    //! @param[in] accelerationLimits - The acceleration limits for the control variables
    //! @param[in] accelerationGains - Gains applied to the control term-derived acceleration
    //! @param[in] decelerationLimits - The deceleration limits for the control variables
    //! @param[in] decelerationGains - Gains applied to the control term-derived deceleration
    //!
    void setControlParams(const std::vector<int> &updateVector, const double controlTimeout,
      const std::vector<double> &accelerationLimits, const std::vector<double> &accelerationGains,
      const std::vector<double> &decelerationLimits, const std::vector<double> &decelerationGains);

    //! @brief Sets the filter into debug mode
    //!
    //! NOTE: this will generates a lot of debug output to the provided stream.
    //! The value must be a pointer to a valid ostream object.
    //!
    //! @param[in] debug - Whether or not to place the filter in debug mode
    //! @param[in] outStream - If debug is true, then this must have a valid pointer.
    //! If the pointer is invalid, the filter will not enter debug mode. If debug is
    //! false, outStream is ignored.
    //!
    void setDebug(const bool debug, std::ostream *outStream = NULL);

    //! @brief Enables dynamic process noise covariance calculation
    //!
    //! @param[in] dynamicProcessNoiseCovariance - Whether or not to compute dynamic process noise covariance matrices
    //!
    void setUseDynamicProcessNoiseCovariance(const bool dynamicProcessNoiseCovariance);

    //! @brief Manually sets the filter's estimate error covariance
    //!
    //! @param[in] estimateErrorCovariance - The state to set as the filter's current state
    //!
    void setEstimateErrorCovariance(const Eigen::MatrixXd &estimateErrorCovariance);

    //! @brief Sets the filter's last measurement time.
    //!
    //! @param[in] lastMeasurementTime - The last measurement time of the filter
    //!
    void setLastMeasurementTime(const double lastMeasurementTime);

    //! @brief Sets the process noise covariance for the filter.
    //!
    //! This enables external initialization, which is important, as this
    //! matrix can be difficult to tune for a given implementation.
    //!
    //! @param[in] processNoiseCovariance - The STATE_SIZExSTATE_SIZE process noise covariance matrix
    //! to use for the filter
    //!
    void setProcessNoiseCovariance(const Eigen::MatrixXd &processNoiseCovariance);

    //! @brief Sets the sensor timeout
    //!
    //! @param[in] sensorTimeout - The time, in seconds, for a sensor to be
    //! considered having timed out
    //!
    void setSensorTimeout(const double sensorTimeout);

    //! @brief Manually sets the filter's state
    //!
    //! @param[in] state - The state to set as the filter's current state
    //!
    void setState(const Eigen::VectorXd &state);

    //! @brief Ensures a given time delta is valid (helps with bag file playback issues)
    //!
    //! @param[in] delta - The time delta, in seconds, to validate
    //!
    void validateDelta(double &delta);

  protected:
    //! @brief Method for settings bounds on acceleration values derived from controls
    //! @param[in] state - The current state variable (e.g., linear X velocity)
    //! @param[in] control - The current control commanded velocity corresponding to the state variable
    //! @param[in] accelerationLimit - Limit for acceleration (regardless of driving direction)
    //! @param[in] accelerationGain - Gain applied to acceleration control error
    //! @param[in] decelerationLimit - Limit for deceleration (moving towards zero, regardless of driving direction)
    //! @param[in] decelerationGain - Gain applied to deceleration control error
    //! @return a usable acceleration estimate for the control vector
    //!
    inline double computeControlAcceleration(const double state, const double control, const double accelerationLimit,
      const double accelerationGain, const double decelerationLimit, const double decelerationGain)
    {
      FB_DEBUG("---------- FilterBase::computeControlAcceleration ----------\n");

      const double error = control - state;
      const bool sameSign = (::fabs(error) <= ::fabs(control) + 0.01);
      const double setPoint = (sameSign ? control : 0.0);
      const bool decelerating = ::fabs(setPoint) < ::fabs(state);
      double limit = accelerationLimit;
      double gain = accelerationGain;

      if (decelerating)
      {
        limit = decelerationLimit;
        gain = decelerationGain;
      }

      const double finalAccel = std::min(std::max(gain * error, -limit), limit);

      FB_DEBUG("Control value: " << control << "\n" <<
               "State value: " << state << "\n" <<
               "Error: " << error << "\n" <<
               "Same sign: " << (sameSign ? "true" : "false") << "\n" <<
               "Set point: " << setPoint << "\n" <<
               "Decelerating: " << (decelerating ? "true" : "false") << "\n" <<
               "Limit: " << limit << "\n" <<
               "Gain: " << gain << "\n" <<
               "Final is " << finalAccel << "\n");

      return finalAccel;
    }

    //! @brief Keeps the state Euler angles in the range [-pi, pi]
    //!
    virtual void wrapStateAngles();

    //! @brief Tests if innovation is within N-sigmas of covariance. Returns true if passed the test.
    //! @param[in] innovation - The difference between the measurement and the state
    //! @param[in] invCovariance - The innovation error
    //! @param[in] nsigmas - Number of standard deviations that are considered acceptable
    //!
    virtual bool checkMahalanobisThreshold(const Eigen::VectorXd &innovation,
                                           const Eigen::MatrixXd &invCovariance,
                                           const double nsigmas);

    //! @brief Converts the control term to an acceleration to be applied in the prediction step
    //! @param[in] referenceTime - The time of the update (measurement used in the prediction step)
    //! @param[in] predictionDelta - The amount of time over which we are carrying out our prediction
    //!
    void prepareControl(const double referenceTime, const double predictionDelta);

    //! @brief Gains applied to acceleration derived from control term
    //!
    std::vector<double> accelerationGains_;

    //! @brief Caps the acceleration we apply from control input
    //!
    std::vector<double> accelerationLimits_;

    //! @brief Variable that gets updated every time we process a measurement and we have a valid control
    //!
    Eigen::VectorXd controlAcceleration_;

    //! @brief Gains applied to deceleration derived from control term
    //!
    std::vector<double> decelerationGains_;

    //! @brief Caps the deceleration we apply from control input
    //!
    std::vector<double> decelerationLimits_;

    //! @brief Latest control term
    //!
    Eigen::VectorXd latestControl_;

    //! @brief Which control variables are being used (e.g., not every vehicle is controllable in Y or Z)
    //!
    std::vector<int> controlUpdateVector_;

    //! @brief Timeout value, in seconds, after which a control is considered stale
    //!
    double controlTimeout_;

    //! @brief Covariance matrices can be incredibly unstable. We can
    //! add a small value to it at each iteration to help maintain its
    //! positive-definite property.
    //!
    Eigen::MatrixXd covarianceEpsilon_;

    //! @brief Used for outputting debug messages
    //!
    std::ostream *debugStream_;

    //! @brief Gets updated when useDynamicProcessNoise_ is true
    //!
    Eigen::MatrixXd dynamicProcessNoiseCovariance_;

    //! @brief This matrix stores the total error in our position
    //! estimate (the state_ variable).
    //!
    Eigen::MatrixXd estimateErrorCovariance_;

    //! @brief We need the identity for a few operations. Better to store it.
    //!
    Eigen::MatrixXd identity_;

    //! @brief Whether or not we've received any measurements
    //!
    bool initialized_;

    //! @brief Tracks the time the filter was last updated using a measurement.
    //!
    //! This value is used to monitor sensor readings with respect to the sensorTimeout_.
    //! We also use it to compute the time delta values for our prediction step.
    //!
    double lastMeasurementTime_;

    //! @brief The time of reception of the most recent control term
    //!
    double latestControlTime_;

    //! @brief Holds the last predicted state of the filter
    //!
    Eigen::VectorXd predictedState_;

    //! @brief As we move through the world, we follow a predict/update
    //! cycle. If one were to imagine a scenario where all we did was make
    //! predictions without correcting, the error in our position estimate
    //! would grow without bound. This error is stored in the
    //! stateEstimateCovariance_ matrix. However, this matrix doesn't answer
    //! the question of *how much* our error should grow for each time step.
    //! That's where the processNoiseCovariance matrix comes in. When we
    //! make a prediction using the transfer function, we add this matrix
    //! (times deltaT) to the state estimate covariance matrix.
    //!
    Eigen::MatrixXd processNoiseCovariance_;

    //! @brief The updates to the filter - both predict and correct - are driven
    //! by measurements. If we get a gap in measurements for some reason, we want
    //! the filter to continue estimating. When this gap occurs, as specified by
    //! this timeout, we will continue to call predict() at the filter's frequency.
    //!
    double sensorTimeout_;

    //! @brief This is the robot's state vector, which is what we are trying to
    //! filter. The values in this vector are what get reported by the node.
    //!
    Eigen::VectorXd state_;

    //! @brief The Kalman filter transfer function
    //!
    //! Kalman filters and extended Kalman filters project the current
    //! state forward in time. This is the "predict" part of the predict/correct
    //! cycle. A Kalman filter has a (typically constant) matrix A that defines
    //! how to turn the current state, x, into the predicted next state. For an
    //! EKF, this matrix becomes a function f(x). However, this function can still
    //! be expressed as a matrix to make the math a little cleaner, which is what
    //! we do here. Technically, each row in the matrix is actually a function.
    //! Some rows will contain many trigonometric functions, which are of course
    //! non-linear. In any case, you can think of this as the 'A' matrix in the
    //! Kalman filter formulation.
    //!
    Eigen::MatrixXd transferFunction_;

    //! @brief The Kalman filter transfer function Jacobian
    //!
    //! The transfer function is allowed to be non-linear in an EKF, but
    //! for propagating (predicting) the covariance matrix, we need to linearize
    //! it about the current mean (i.e., state). This is done via a Jacobian,
    //! which calculates partial derivatives of each row of the transfer function
    //! matrix with respect to each state variable.
    //!
    Eigen::MatrixXd transferFunctionJacobian_;

    //! @brief Whether or not we apply the control term
    //!
    bool useControl_;

    //! @brief If true, uses the robot's vehicle state and the static process noise covariance matrix to generate a
    //! dynamic process noise covariance matrix
    //!
    bool useDynamicProcessNoiseCovariance_;

  private:
    //! @brief Whether or not the filter is in debug mode
    //!
    bool debug_;
};

}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_FILTER_BASE_H
