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

#include <robot_localization/filter_common.hpp>
#include <robot_localization/filter_utilities.hpp>

#include <iomanip>
#include <string>
#include <vector>

std::ostream & operator<<(std::ostream & os, const Eigen::MatrixXd & mat)
{
  os << "[";

  int row_count = static_cast<int>(mat.rows());

  for (int row = 0; row < row_count; ++row) {
    if (row > 0) {
      os << " ";
    }

    for (int col = 0; col < mat.cols(); ++col) {
      os << std::setiosflags(std::ios::left) << std::setw(12) <<
        std::setprecision(5) << mat(row, col);
    }

    if (row < row_count - 1) {
      os << "\n";
    }
  }

  os << "]\n";

  return os;
}

std::ostream & operator<<(std::ostream & os, const Eigen::VectorXd & vec)
{
  os << "[";
  for (int dim = 0; dim < vec.rows(); ++dim) {
    os << std::setiosflags(std::ios::left) << std::setw(12) <<
      std::setprecision(5) << vec(dim);
  }
  os << "]\n";

  return os;
}

std::ostream & operator<<(std::ostream & os, const std::vector<size_t> & vec)
{
  os << "[";
  for (size_t dim = 0; dim < vec.size(); ++dim) {
    os << std::setiosflags(std::ios::left) << std::setw(12) <<
      std::setprecision(5) << vec[dim];
  }
  os << "]\n";

  return os;
}

std::ostream & operator<<(std::ostream & os, const std::vector<int> & vec)
{
  os << "[";
  for (size_t dim = 0; dim < vec.size(); ++dim) {
    os << std::setiosflags(std::ios::left) << std::setw(3) <<
    (vec[dim] ? "t" : "f");
  }
  os << "]\n";

  return os;
}
