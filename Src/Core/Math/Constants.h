/**
 * @file Constants.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in License.Nao_Devils.txt.
 * (c) 2023 Nao Devils and NomadZ team
 */

#pragma once

namespace Constants {
  constexpr float g = 9806.65f;        ///< Gravity of the earth in mm/s�
  constexpr float g_1000 = g / 1000.f; ///< Gravity of earth in m/s�

  constexpr float e = 2.71828182845902353602874713527f; ///< Euler's number

  /** @name constants for some often used angles */
  ///@{
  /** constant for a half circle*/
  constexpr float pi = 3.1415926535897932384626433832795f;
  /** constant for a full circle*/
  constexpr float pi2 = 2.f * pi;
  /** constant for three quater circle*/
  constexpr float pi3_2 = 1.5f * pi;
  /** constant for a quarter circle*/
  constexpr float pi_2 = pi / 2.f;
  /** constant for a 1/8 circle*/
  constexpr float pi_4 = pi / 4.f;
  /** constant for a 1/16 circle*/
  constexpr float pi_8 = pi / 8.f;
  /** constant for a 3/8 circle*/
  constexpr float pi3_4 = pi * 0.75f;
  ///@}

  constexpr float motionCycleTime = 0.012f;
}; // namespace Constants

using Constants::pi;
using Constants::pi2;
using Constants::pi3_2;
using Constants::pi3_4;
using Constants::pi_2;
using Constants::pi_4;
using Constants::pi_8;
