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

#ifndef ROBOT_LOCALIZATION_FILTER_UTILITIES_H
#define ROBOT_LOCALIZATION_FILTER_UTILITIES_H

#include <Eigen/Dense>

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#define FB_DEBUG(msg) if (getDebug()) { *debugStream_ << msg; }

// Handy methods for debug output
std::ostream& operator<<(std::ostream& os, const Eigen::MatrixXd &mat);
std::ostream& operator<<(std::ostream& os, const Eigen::VectorXd &vec);
std::ostream& operator<<(std::ostream& os, const std::vector<size_t> &vec);
std::ostream& operator<<(std::ostream& os, const std::vector<int> &vec);

namespace RobotLocalization
{
namespace FilterUtilities
{

  //! @brief Utility method keeping RPY angles in the range [-pi, pi]
  //! @param[in] rotation - The rotation to bind
  //! @return the bounded value
  //!
  double clampRotation(double rotation);

  //! @brief Utility method for appending tf2 prefixes cleanly
  //! @param[in] tfPrefix - the tf2 prefix to append
  //! @param[in, out] frameId - the resulting frame_id value
  //!
  void appendPrefix(std::string tfPrefix, std::string &frameId);

}  // namespace FilterUtilities
}  // namespace RobotLocalization

#endif  // ROBOT_LOCALIZATION_FILTER_UTILITIES_H
