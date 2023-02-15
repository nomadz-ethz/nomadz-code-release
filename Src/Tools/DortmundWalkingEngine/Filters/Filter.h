/**
 * @file Filter.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#pragma once

class Filter {
public:
  Filter(void){};
  ~Filter(void){};

  virtual double nextValue(double v) = 0;
};
