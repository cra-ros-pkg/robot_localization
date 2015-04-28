/*
 * Copyright (c) 2015, Charles River Analytics, Inc.
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

#ifndef RobotLocalization_RosFilterUtilities_h
#define RobotLocalization_RosFilterUtilities_h

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <Eigen/Dense>

#include <iostream>
#include <iomanip>

#define RF_DEBUG(msg) if(filter_.getDebug()) { debugStream_ << msg; }

// Handy methods for debug output
std::ostream& operator<<(std::ostream& os, const tf2::Vector3 &vec);
std::ostream& operator<<(std::ostream& os, const tf2::Quaternion &quat);
std::ostream& operator<<(std::ostream& os, const tf2::Transform &trans);

namespace RobotLocalization
{
  namespace RosFilterUtilities
  {
    //! @brief Utility method for converting quaternion to RPY
    //! @param[in] quat - The quaternion to convert
    //! @param[out] roll - The converted roll
    //! @param[out] pitch - The converted pitch
    //! @param[out] yaw - The converted yaw
    //!
    void quatToRPY(const tf2::Quaternion &quat, double &roll, double &pitch, double &yaw);

    //! @brief Converts our Eigen state vector into a TF transform/pose
    //! @param[in] state - The state to convert
    //! @param[out] stateTF - The converted state
    //!
    void stateToTF(const Eigen::VectorXd &state, tf2::Transform &stateTF);

    //! @brief Converts a TF transform/pose into our Eigen state vector
    //! @param[in] stateTF - The state to convert
    //! @param[out] state - The converted state
    //!
    void TFtoState(const tf2::Transform &stateTF, Eigen::VectorXd &state);
  }
}

#endif
