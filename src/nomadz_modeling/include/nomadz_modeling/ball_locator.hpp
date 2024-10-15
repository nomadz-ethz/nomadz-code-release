#pragma once

#include <cmath>
#include <algorithm>
#include <deque>

#include <Eigen/Core>

namespace nomadz_modeling {

  class BallLocator {
  public:
    explicit BallLocator(int window_size);

    void reset();

    void update(const Eigen::Vector2f& relative_position, bool ball_seen);

    float xFiltered() const { return x_state_; }
    float yFiltered() const { return y_state_; }

  private:
    int window_size_;
    std::deque<Eigen::Vector2f> window_;
    Eigen::Vector2f sum_;
    int count_;
    float x_state_;
    float y_state_;
    float distance_reset_ = 1.0;
    int patience_ = 0;

    void addPosition(const Eigen::Vector2f& newPosition);

    Eigen::Vector2f getMovingAverage() const;
  };

} // namespace nomadz_modeling
