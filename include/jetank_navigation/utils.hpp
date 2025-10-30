#pragma once

#include <cmath>

namespace jetank_navigation {

#ifndef M_PI
constexpr double M_PI = 3.14159265358979323846;
#endif

/// Calculating horizontal FOV based on diagonal FOV
/// @param diagonal_FOV diagonal field of view in radians, @param image_width width of camera image in pixels, @param image_height height of camera image in pixels
/// @return horizontal fov in radians
constexpr double diagonal_fov_to_horizontal(const double diagonal_fov, const double image_width, const double image_height) {
    return 2.0 * std::atan(std::tan(diagonal_fov / 2.0) * (image_width/image_height))
}

/// Calculating radians from degrees
/// @param degrees Angle in degrees
/// @return Angle in radians
constexpr double deg_to_rad(double degrees) {
    return degrees * (M_PI / 180.0);
}

/// Calculating angle increment between laser beams
/// @param angle_min minimal angle of range in radians, @param angle_max maximal angle of range in radians, @param beam_density amount of beams
/// @return angle between beams in radians
constexpr double calculate_angle_increment(const double angle_min, const double angle_max, const int beam_density) {
    return (angle_max - angle_min) / (beam_density - 1);
}

} // namespace jetank_navigation
