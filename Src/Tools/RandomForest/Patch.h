/**
 * @file Patch.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include <opencv2/core/core.hpp>

/*
 *Struct that represents a patch
 */
struct Patch {

  std::string name;
  cv::Mat mat;
  unsigned int label;
  unsigned int diameter;
  cv::Point offset;
  cv::Mat rotation;
  cv::Mat translation;
};
