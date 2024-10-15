#pragma once

/**
 * @author Felix Thielke
 */

#include <cmath>
#include <vector>
#include <array>
#include <type_traits>
#include <utility>

#include <Eigen/Core>

namespace nomadz_core {

  namespace zero_type_select {
    template <typename T> struct EigenZeroType {
      static inline T value() { return T::Zero(); }
    };

    template <typename T> struct OtherZeroType {
      static inline constexpr T value() { return T(0); }
    };

    template <typename T> static EigenZeroType<T> select(const Eigen::MatrixBase<T>*);
    template <typename T> static OtherZeroType<T> select(...);
  } // namespace zero_type_select

  template <typename T> struct ZeroType : public decltype(zero_type_select::select<T>(std::declval<T*>())) {};

  template <typename ResultType, typename ValueType> class MeanCalculator {
  public:
    using CallbackType = std::function<ResultType(const ValueType&)>;

  protected:
    ResultType sum_;
    CallbackType callback_;
    size_t count_{};

  public:
    explicit MeanCalculator(const CallbackType& callback = MeanCalculator<ResultType, ValueType>::id)
        : sum_(ZeroType<ResultType>::value()), callback_(callback) {}

    static inline ResultType id(const ValueType& v) { return static_cast<ResultType>(v); }

    MeanCalculator& add(const MeanCalculator& v) {
      sum_ += v.sum_;
      count_ += v.count_;
      return *this;
    }

    MeanCalculator& add(const ValueType& v) {
      sum_ += callback_(v);
      count_++;
      return *this;
    }

    template <typename IteratorType> MeanCalculator& add(const IteratorType& begin, const IteratorType& end) {
      for (auto it = begin; it != end; std::advance(it, 1)) {
        sum_ += callback_(static_cast<ValueType>(*it));
      }
      count_ += std::distance(begin, end);
      return *this;
    }

    MeanCalculator& add(const std::vector<ValueType>& data) { return add(data.cbegin(), data.cend()); }

    template <size_t N> MeanCalculator& add(const std::array<ValueType, N>& data) { return add(data.cbegin(), data.cend()); }

    explicit operator ResultType() const { return ResultType(sum_ / count_); }
  };

  template <typename ResultType, typename ValueType, typename IteratorType>
  static inline ResultType calcMean(const IteratorType& begin, const IteratorType& end) {
    return static_cast<ResultType>(MeanCalculator<ResultType, ValueType>().add(begin, end));
  }

  template <typename ResultType, typename ValueType, size_t N>
  static inline ResultType calcMean(const std::array<ValueType, N>& data) {
    return static_cast<ResultType>(MeanCalculator<ResultType, ValueType>().add(data));
  }

  template <typename ResultType, typename ValueType> static inline ResultType calcMean(const std::vector<ValueType>& data) {
    return static_cast<ResultType>(MeanCalculator<ResultType, ValueType>().add(data));
  }

  template <typename ValueType, typename IteratorType>
  static inline float calcMeanSquared(const IteratorType& begin, const IteratorType& end) {
    return std::sqrt(
      static_cast<float>(MeanCalculator<ValueType, ValueType>([](const ValueType& v) { return v * v; }).add(begin, end)));
  }

  template <typename ResultType, typename ValueType, size_t N>
  static inline ResultType calcMeanSquared(const std::array<ValueType, N>& data) {
    return std::sqrt(
      static_cast<float>(MeanCalculator<ValueType, ValueType>([](const ValueType& v) { return v * v; }).add(data)));
  }

  template <typename ResultType, typename ValueType>
  static inline ResultType calcMeanSquared(const std::vector<ValueType>& data) {
    return std::sqrt(
      static_cast<float>(MeanCalculator<ValueType, ValueType>([](const ValueType& v) { return v * v; }).add(data)));
  }

} // namespace nomadz_core
