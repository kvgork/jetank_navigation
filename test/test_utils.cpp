// Unit tests for the pure math helpers in jetank_navigation/utils.hpp.
//
// These tests are the first (and currently only) consumer that #includes the
// header, so they double as a compile guard: a malformed declaration in
// utils.hpp would break this build before it could reach a runtime assertion.

#include <cmath>

#include <gtest/gtest.h>

#include "jetank_navigation/utils.hpp"

using jetank_navigation::calculate_angle_increment;
using jetank_navigation::deg_to_rad;
using jetank_navigation::diagonal_fov_to_horizontal;

TEST(UtilsTest, DegToRadKnownValues)
{
  EXPECT_DOUBLE_EQ(deg_to_rad(0.0), 0.0);
  EXPECT_DOUBLE_EQ(deg_to_rad(180.0), M_PI);
  EXPECT_DOUBLE_EQ(deg_to_rad(90.0), M_PI / 2.0);
  EXPECT_DOUBLE_EQ(deg_to_rad(360.0), 2.0 * M_PI);
  // Linearity / sign handling.
  EXPECT_DOUBLE_EQ(deg_to_rad(-90.0), -M_PI / 2.0);
}

TEST(UtilsTest, CalculateAngleIncrement)
{
  // 5 beams evenly spaced over [0, pi] -> 4 gaps of pi/4.
  EXPECT_DOUBLE_EQ(calculate_angle_increment(0.0, M_PI, 5), M_PI / 4.0);
  // 3 beams over [-pi, pi] -> 2 gaps of pi.
  EXPECT_DOUBLE_EQ(calculate_angle_increment(-M_PI, M_PI, 3), M_PI);
  // Symmetric span, 2 beams -> a single gap equal to the full span.
  EXPECT_DOUBLE_EQ(calculate_angle_increment(-1.0, 1.0, 2), 2.0);
}

TEST(UtilsTest, DiagonalFovToHorizontal)
{
  // Square image (w == h): tan(d/2) * 1 -> horizontal FOV == diagonal FOV.
  EXPECT_NEAR(diagonal_fov_to_horizontal(M_PI / 2.0, 1.0, 1.0), M_PI / 2.0, 1e-12);
  EXPECT_NEAR(diagonal_fov_to_horizontal(M_PI / 2.0, 480.0, 480.0), M_PI / 2.0, 1e-12);
  // Wider-than-tall image stretches the horizontal FOV: 2*atan(tan(45deg)*2).
  EXPECT_NEAR(
    diagonal_fov_to_horizontal(M_PI / 2.0, 2.0, 1.0),
    2.0 * std::atan(2.0), 1e-12);
  // A wider image yields a larger horizontal FOV than a square one.
  EXPECT_GT(
    diagonal_fov_to_horizontal(M_PI / 2.0, 16.0, 9.0),
    diagonal_fov_to_horizontal(M_PI / 2.0, 1.0, 1.0));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
