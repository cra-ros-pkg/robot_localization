/*
 * Copyright (c) 2014, Charles River Analytics, Inc.
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

#ifndef RobotLocalization_FilterBase_h
#define RobotLocalization_FilterBase_h

#include <Eigen/Dense>

#include <ostream>
#include <vector>
#include <set>
#include <queue>

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

      // We want earlier times to have greater priority
      bool operator()(const Measurement &a, const Measurement &b)
      {
        return a.time_ > b.time_;
      }
  };

  class FilterBase
  {
    public:

      //! @brief Constructor for the FilterBase class
      //!
      FilterBase();

      //! @brief Destructor for the FilterBase class
      //!
      ~FilterBase();

      //! @brief Adds a measurement to the queue of measurements to be processed
      //!
      //! @param[in] measurement - The measurement to enqueue
      //! @param[in] measurementCovariance - The covariance of the measurement
      //! @param[in] updateVector - The boolean vector that specifies which variables to update from this measurement
      //! @param[in] time - The time of arrival (in seconds)
      //!
      virtual void enqueueMeasurement(const Eigen::VectorXd &measurement,
                                      const Eigen::MatrixXd &measurementCovariance,
                                      const std::vector<int> &updateVector,
                                      const double time);

      //! @brief Processes all measurements in the measurement queue, in temporal order
      //!
      //! @param[in] currentTime - The time at which to carry out integration (the current time)
      //!
      virtual void integrateMeasurements(double currentTime);

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

      //! @brief Gets the filter's last update time
      //!
      //! @return The time at which we last updated the filter
      //!
      double getLastUpdateTime();

      //! @brief Gets the filter's process noise covariance
      //!
      //! @return A copy of the process noise covariance
      //!
      const Eigen::MatrixXd& getProcessNoiseCovariance();

      //! @brief Gets the sensor timeout value (in seconds)
      //!
      //! @return The sensor timeout value
      //!
      double getSensorTimeout();

      //! @brief Gets the filter state
      //!
      //! @return A copy of the current state
      //!
      const Eigen::VectorXd& getState();

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

      //! @brief Sets the filter's last measurement time.
      //!
      //! @param[in] lastMeasurementTime - The last measurement time of the filter
      //!
      void setLastMeasurementTime(const double lastMeasurementTime);

      //! @brief Sets the filter's last update time.
      //!
      //! This is used mostly for initialization purposes, as the integrateMeasurements()
      //! function will update the filter's last update time as well.
      //!
      //! @param[in] lastUpdateTime - The last update time of the filter
      //!
      void setLastUpdateTime(const double lastUpdateTime);

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

    protected:

      //! @brief Carries out the correct step in the predict/update cycle. This method
      //! must be implemented by subclasses.
      //!
      //! @param[in] measurement - The measurement to fuse with the state estimate
      //!
      virtual void correct(const Measurement &measurement) = 0;

      //! @brief Carries out the predict step in the predict/update cycle.
      //! Projects the state and error matrices forward using a model of
      //! the vehicle's motion. This method must be implemented by subclasses.
      //!
      //! @param[in] delta - The time step over which to predict.
      //!
      virtual void predict(const double delta) = 0;

      //! @brief Does some final preprocessing, carries out the predict/update cycle
      //!
      //! @param[in] measurement - The measurement object to fuse into the filter
      //!
      virtual void processMeasurement(const Measurement &measurement);

      //! @brief Ensures a given time delta is valid (helps with bag file playback issues)
      //!
      //! @param[in] delta - The time delta, in seconds, to validate
      //!
      void validateDelta(double &delta);

      //! @brief Keeps the state angles to reasonable values
      //!
      virtual void wrapStateAngles();

      //! @brief Whether or not we've received any measurements
      //!
      bool initialized_;

      //! @brief Used for outputting debug messages
      //!
      std::ostream *debugStream_;

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

      //! @brief This matrix stores the total error in our position
      //! estimate (the state_ variable).
      //!
      Eigen::MatrixXd estimateErrorCovariance_;

      //! @brief Covariance matrices can be incredibly unstable. We can
      //! add a small value to it at each iteration to help maintain its
      //! positive-definite property.
      //!
      Eigen::MatrixXd covarianceEpsilon_;

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

      //! @brief We need the identity for a few operations. Better to store it.
      //!
      Eigen::MatrixXd identity_;

      //! @brief The updates to the filter - both predict and correct - are driven
      //! by measurements. If we get a gap in measurements for some reason, we want
      //! the filter to continue estimating. When this gap occurs, as specified by
      //! this timeout, we will continue to call predict() at the filter's frequency.
      //!
      double sensorTimeout_;

      //! @brief Commonly used constants
      //!
      const double pi_;

      const double tau_;

      //! @brief We process measurements based on their timestamp.
      //!
      //! In the events that messages come in asynchronously and with no
      //! guarantee on order, we can use this to ensure that they are
      //! processed in sequence.
      //!
      std::priority_queue<Measurement, std::vector<Measurement>, Measurement> measurementQueue_;

      //! @brief Used for tracking the latest update time as determined
      //! by this class.
      //!
      //! We assume that this class may receive measurements that occurred in the past,
      //! as may happen with sensors distributed on different machines on a network. This
      //! variable tracks when the filter was updated with respect to the executable in
      //! which this class was instantiated. We use this to determine if we have experienced
      //! a sensor timeout, i.e., if we haven't received any sensor data in a long time.
      //!
      double lastUpdateTime_;

      //! @brief Tracks the time the filter was last updated using a measurement.
      //!
      //! This value is used to monitor sensor readings with respect to the sensorTimeout_.
      //! We also use it to compute the time delta values for our prediction step.
      //!
      double lastMeasurementTime_;

    private:

      //! @brief Whether or not the filter is in debug mode
      //!
      bool debug_;
  };
}

// Handy methods for debug output
std::ostream& operator<<(std::ostream& os, const Eigen::MatrixXd &mat);
std::ostream& operator<<(std::ostream& os, const Eigen::VectorXd &vec);
std::ostream& operator<<(std::ostream& os, const std::vector<size_t> &vec);
std::ostream& operator<<(std::ostream& os, const std::vector<int> &vec);

#endif
