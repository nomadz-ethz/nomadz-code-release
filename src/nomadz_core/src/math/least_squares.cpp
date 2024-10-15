#include "nomadz_core/math/least_squares.hpp"

#include <Eigen/Eigenvalues>

#include "nomadz_core/math/approx.hpp"
#include "nomadz_core/math/bh_math.hpp"

namespace nomadz_core {

  bool LineFitter::fit(Eigen::Vector2f& n0, float& d) const {
    // see R.Duda, P. Hart: Pattern classification and scene analysis. Wiley, 1973. pp 332-335
    assert(count_ >= 2);

    const auto sum = this->sum_.col(0);    // (sum(x), sum(y))
    const auto sum_2 = this->sum_.col(1);  // (sum(x^2), sum(y^2))
    const float sum_xy = this->sum_(0, 2); // sum(xy)
    const float n_inv = 1.F / static_cast<float>(count_);

    // Calculate mean, variance and covariance
    const Eigen::Vector2f mean(sum * n_inv);
    const Eigen::Vector2f var(sum_2 - n_inv * sum.cwiseProduct(sum)); // (var_x * count, var_y * count)
    const float cov_xy = sum_xy - n_inv * sum.x() * sum.y();          // cov_xy * count

    // Calculate the eigen decomposition of the covariance matrix
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f>().computeDirect(
        (Eigen::Matrix2f() << var.x(), cov_xy, cov_xy, var.y()).finished()));
    if (solver.info() != Eigen::ComputationInfo::Success) {
      return false;
    }
    const auto& eigen_values = solver.eigenvalues();

    // n0 is the eigenvector with the smaller eigenvalue
    n0 = solver.eigenvectors().col(eigen_values[0] < eigen_values[1] ? 0 : 1);
    d = n0.dot(mean);

    return true;
  }

  bool CircleFitter::fit(Eigen::Vector2f& center, float& radius) const {
    // Algorithm adapted from https://dtcenter.org/sites/default/files/community-code/met/docs/write-ups/circle_fit.pdf
    assert(count_ >= 3);

    const auto sum = this->sum_.col(0);    // (sum(x), sum(y))
    const auto sum2 = this->sum_.col(1);   // (sum(x^2), sum(y^2))
    const auto sum3 = this->sum_.col(2);   // (sum(x^3 + x * y^2), sum(y^3 + x^2 * y))
    const float sum_xy = this->sum_(0, 3); // sum(xy)
    const float n_inv = 1.F / static_cast<float>(count_);

    // Calculate mean, variance and covariance
    const Eigen::Vector2f mean(sum * n_inv);
    const Eigen::Vector2f var(sum2 - n_inv * sum.cwiseProduct(sum)); // (var_x * count, var_y * count)
    const float cov_xy = sum_xy - n_inv * sum.x() * sum.y();         // cov_xy * count

    const float divisor = 2.F * (var.x() * var.y() - sqr(cov_xy));
    if (approx::isZero(divisor)) {
      return false;
    }

    const float coeff1 = 2.F * n_inv * sum.squaredNorm();
    const float coeff2 = -2.F * sum_xy;
    const Eigen::Matrix2f m1(
      (Eigen::Matrix2f() << (coeff1 - 3.F * sum2.x() - sum2.y()), coeff2, coeff2, (coeff1 - 3.F * sum2.y() - sum2.x()))
        .finished());

    const Eigen::Matrix2f m2((Eigen::Matrix2f() << var.y(), -cov_xy, -cov_xy, var.x()).finished());

    const Eigen::Vector2f c(m2 * ((sum3 + m1 * mean) / divisor));

    center = c + mean;
    radius = std::sqrt(c.squaredNorm() + n_inv * var.sum());

    return true;
  }

} // namespace nomadz_core
