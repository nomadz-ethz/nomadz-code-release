/**
 * @file Kalman.h
 *
 * Template class for one-dimensional kalman filtering.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#pragma once

/**
 * @class Kalman
 * Implements a one-dimensional kalman filter.
 */
template <class V = float> class Kalman {
public:
  V value;    /**< The estimated state. */
  V variance; /**< The current variance. */

  /**
   * Default constructor.
   */
  Kalman() : value(V()), variance(V()) {}

  /**
   * Constructor.
   * @param value The initial state.
   * @param variance The initial variance.
   */
  Kalman(const V& value, const V& variance) : value(value), variance(variance) {}

  /**
   * Initializes the filter.
   * @param value The initial state.
   * @param variance The initial variance.
   */
  void init(const V& value, const V& variance) {
    this->value = value;
    this->variance = variance;
  }

  /**
   * Performs a prediction step.
   * @param u A dynamic measurement.
   * @param uVariance The variance of this measurement.
   */
  void predict(const V& u, const V& uVariance) {
    value += u;
    variance += uVariance;
  }

  /**
   * Performs an update step.
   * @param z An absolute measurement.
   * @param zVariance The variance of this measurement.
   */
  void update(const V& z, const V& zVariance) {
    const V& k(variance / (variance + zVariance));
    value += k * (z - value);
    variance -= k * variance;
  }
};
