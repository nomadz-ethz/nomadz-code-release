/**
 * @file FIRFilter.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#pragma once

#include "Tools/DortmundWalkingEngine/DynamicRingBuffer.h"
#include "Filter.h"
#include <string>
#include <vector>

class FIRFilter : public Filter {
public:
  FIRFilter(void);
  ~FIRFilter(void) { delete buffer; };

  double nextValue(double v);

  bool readCoefficients(std::string path);
  void setCoefficients(double* coefficients, int n);

private:
  unsigned int n;
  DynamicRingBuffer<double>* buffer;
  std::vector<double> coefficients;
};
