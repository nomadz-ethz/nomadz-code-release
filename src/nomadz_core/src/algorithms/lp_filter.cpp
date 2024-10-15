#include "nomadz_core/algorithms/lp_filter.hpp"

namespace nomadz_core::algorithms {

  /*!
   * implement digital Filter using difference equation in
   * direct II transposed structure form
   * @param sig new value of signal to Filter
   */
  float LPFilter::process(const float& sig) {
    float sig_filt = b_[0] * sig + z_[0];
    for (std::size_t i = 1; i != a_.size(); i++) {
      z_[i - 1] = b_[i] * sig + z_[i] - a_[i] * sig_filt;
    }
    return sig_filt;
  };

} // namespace nomadz_core::algorithms
