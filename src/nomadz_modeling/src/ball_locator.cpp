
#include "nomadz_modeling/ball_locator.hpp"

#include <cmath>
#include <algorithm>
#include <spdlog/spdlog.h>

namespace nomadz_modeling {

  BallLocator::BallLocator(int window_size) : window_size_(window_size), sum_(Eigen::Vector2f::Zero()), count_(0) {}

  void BallLocator::reset() {
    window_.clear();
    sum_ = Eigen::Vector2f::Zero();
    count_ = 0;
  }

  void BallLocator::update(const Eigen::Vector2f& relative_position, bool ball_seen) {
    if (ball_seen) {
      ++count_;
      addPosition(relative_position);
      Eigen::Vector2f moving_average = getMovingAverage();
      if ((moving_average - Eigen::Vector2f(x_state_, y_state_)).norm() > distance_reset_) {
        ++patience_;
        if (patience_ > 5) {
          reset();
          ++count_;
          window_.push_back(relative_position);
          sum_ += relative_position;
          moving_average = getMovingAverage();
        } else {
          --count_;
          sum_ -= window_.back();
          window_.pop_back();
          moving_average = getMovingAverage();
        }
      }
      x_state_ = moving_average.x();
      y_state_ = moving_average.y();
    }
  }

  void BallLocator::addPosition(const Eigen::Vector2f& newPosition) {
    // Add new position to the window and update the sum
    window_.push_back(newPosition);
    sum_ += newPosition;

    // Remove the oldest position if the window size is exceeded
    if (static_cast<int>(window_.size()) > window_size_) {
      sum_ -= window_.front();
      window_.pop_front();
    }
  }

  Eigen::Vector2f BallLocator::getMovingAverage() const {
    // Compute the moving average
    return sum_ / static_cast<float>(window_.size());
  }

} // namespace nomadz_modeling
