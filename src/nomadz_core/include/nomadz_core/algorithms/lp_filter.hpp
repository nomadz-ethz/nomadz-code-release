// BSD 2-Clause License

// Copyright (c) 2023, Intelligent Control Systems
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// ------------------------------------------------------------------------------
// Original Code from: https://gitlab.ethz.ch/ics/crs/-/blob/main/software/src/crs/commons/include/commons/filter.h
// 13-Jul-2024

#pragma once

#include <vector>

namespace nomadz_core::algorithms {

  /**
   * @brief Implementation of a Discrete-time and Statistical Signal Processing (DSSP) Filter
   *
   */
  class LPFilter {
  public:
    /**
     * default constructor
     */
    LPFilter() : b_(1, 0), a_(1, 0), z_(1, 0){};

    /**
     * @brief Construct a new Filter object
     *
     * @param b numerator coefficients of digital Filter transfer function
     * @param a denominator coefficients of digital Filter transfer function
     */
    LPFilter(const std::vector<float>& b, const std::vector<float>& a) : b_(b), a_(a), z_(a.size(), 0){}; // NOLINT

    /**
     * @brief implement digital Filter using difference equation in
     *        direct II transposed structure form
     * @param sig new value of signal to Filter
     */
    float process(const float& sig);

  private:
    std::vector<float> b_;
    std::vector<float> a_;
    std::vector<float> z_;
  };

} // namespace nomadz_core::algorithms
