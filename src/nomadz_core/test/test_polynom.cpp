#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "nomadz_core/math/polynom.hpp"

TEST(PolynomTest, BinomialCoefficient) {
  // Test binomial coefficient for n = 5, k = 2
  EXPECT_EQ(nomadz_core::polynom::binomialCoefficient(5, 2), 10);
  // Test binomial coefficient for n = 10, k = 3
  EXPECT_EQ(nomadz_core::polynom::binomialCoefficient(10, 3), 120);
  // Test binomial coefficient for n = 7, k = 4
  EXPECT_EQ(nomadz_core::polynom::binomialCoefficient(7, 4), 35);
}

TEST(PolynomTest, Bernstein) {
  EXPECT_EQ(std::pow(0, static_cast<float>(0)), 1);

  EXPECT_EQ(nomadz_core::polynom::bernstein(0.F, 3, 0), 1);
  EXPECT_EQ(nomadz_core::polynom::bernstein(0.F, 3, 1), 0);
  EXPECT_EQ(nomadz_core::polynom::bernstein(0.F, 3, 2), 0);
  EXPECT_EQ(nomadz_core::polynom::bernstein(0.F, 3, 3), 0);

  EXPECT_EQ(nomadz_core::polynom::bernstein(0.5F, 3, 0), 0.125);
  EXPECT_EQ(nomadz_core::polynom::bernstein(0.5F, 3, 1), 0.375);
  EXPECT_EQ(nomadz_core::polynom::bernstein(0.5F, 3, 2), 0.375);
  EXPECT_EQ(nomadz_core::polynom::bernstein(0.5F, 3, 3), 0.125);

  EXPECT_EQ(nomadz_core::polynom::bernstein(1.F, 3, 0), 0);
  EXPECT_EQ(nomadz_core::polynom::bernstein(1.F, 3, 1), 0);
  EXPECT_EQ(nomadz_core::polynom::bernstein(1.F, 3, 2), 0);
  EXPECT_EQ(nomadz_core::polynom::bernstein(1.F, 3, 3), 1);
}
