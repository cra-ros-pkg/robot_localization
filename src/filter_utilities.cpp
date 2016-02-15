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

#include "robot_localization/filter_utilities.h"
#include "robot_localization/filter_common.h"

#include <string>
#include <vector>

std::ostream& operator<<(std::ostream& os, const Eigen::MatrixXd &mat)
{
  os << "[";

  int rowCount = static_cast<int>(mat.rows());

  for (int row = 0; row < rowCount; ++row)
  {
    if (row > 0)
    {
      os << " ";
    }

    for (int col = 0; col < mat.cols(); ++col)
    {
      os << std::setiosflags(std::ios::left) << std::setw(12) << std::setprecision(5) << mat(row, col);
    }

    if (row < rowCount - 1)
    {
      os << "\n";
    }
  }

  os << "]\n";

  return os;
}

std::ostream& operator<<(std::ostream& os, const Eigen::VectorXd &vec)
{
  os << "[";
  for (int dim = 0; dim < vec.rows(); ++dim)
  {
    os << std::setiosflags(std::ios::left) << std::setw(12) << std::setprecision(5) << vec(dim);
  }
  os << "]\n";

  return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<size_t> &vec)
{
  os << "[";
  for (size_t dim = 0; dim < vec.size(); ++dim)
  {
    os << std::setiosflags(std::ios::left) << std::setw(12) << std::setprecision(5) << vec[dim];
  }
  os << "]\n";

  return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<int> &vec)
{
  os << "[";
  for (size_t dim = 0; dim < vec.size(); ++dim)
  {
    os << std::setiosflags(std::ios::left) << std::setw(3) << (vec[dim] ? "t" : "f");
  }
  os << "]\n";

  return os;
}

namespace RobotLocalization
{

namespace FilterUtilities
{
  void appendPrefix(std::string tfPrefix, std::string &frameId)
  {
    // Strip all leading slashes for tf2 compliance
    if (!frameId.empty() && frameId.at(0) == '/')
    {
      frameId = frameId.substr(1);
    }

    if (!tfPrefix.empty() && tfPrefix.at(0) == '/')
    {
      tfPrefix = tfPrefix.substr(1);
    }

    // If we do have a tf prefix, then put a slash in between
    if (!tfPrefix.empty())
    {
      frameId = tfPrefix + "/" + frameId;
    }
  }

  double clampRotation(double rotation)
  {
    while (rotation > PI)
    {
      rotation -= TAU;
    }

    while (rotation < -PI)
    {
      rotation += TAU;
    }

    return rotation;
  }

}  // namespace FilterUtilities

}  // namespace RobotLocalization
