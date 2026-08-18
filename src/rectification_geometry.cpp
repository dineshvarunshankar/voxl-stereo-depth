#include "hetero_dfs/rectification_geometry.hpp"

#include <cmath>

namespace hetero_dfs {
namespace {

const double kMinimumBaselineM = 1.0e-6;
const double kRotationTolerance = 1.0e-6;

bool finite_transform(const StereoTransform& transform)
{
    for (int row = 0; row < 3; ++row) {
        if (!std::isfinite(transform.translation_to_from[row])) {
            return false;
        }
        for (int column = 0; column < 3; ++column) {
            if (!std::isfinite(transform.rotation_to_from(row, column))) {
                return false;
            }
        }
    }
    return true;
}

double determinant(const cv::Matx33d& rotation)
{
    return rotation(0, 0) *
               (rotation(1, 1) * rotation(2, 2) -
                rotation(1, 2) * rotation(2, 1)) -
           rotation(0, 1) *
               (rotation(1, 0) * rotation(2, 2) -
                rotation(1, 2) * rotation(2, 0)) +
           rotation(0, 2) *
               (rotation(1, 0) * rotation(2, 1) -
                rotation(1, 1) * rotation(2, 0));
}

bool valid_rotation(const cv::Matx33d& rotation)
{
    const cv::Matx33d product = rotation.t() * rotation;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            const double expected = row == column ? 1.0 : 0.0;
            if (std::abs(product(row, column) - expected) >
                kRotationTolerance) {
                return false;
            }
        }
    }
    return std::abs(determinant(rotation) - 1.0) <= kRotationTolerance;
}

bool normalize(const cv::Vec3d& input, cv::Vec3d* output)
{
    const double length = cv::norm(input);
    if (!std::isfinite(length) || length <= kMinimumBaselineM) {
        return false;
    }
    *output = input * (1.0 / length);
    return true;
}

}  // namespace

bool compute_rectification_geometry(
    const StereoTransform& tracking_to_hires,
    RectificationGeometry* geometry)
{
    if (geometry == NULL ||
        !finite_transform(tracking_to_hires) ||
        !valid_rotation(tracking_to_hires.rotation_to_from)) {
        return false;
    }

    const cv::Matx33d hires_to_tracking =
        tracking_to_hires.rotation_to_from.t();
    const cv::Vec3d hires_center_in_tracking =
        -(hires_to_tracking * tracking_to_hires.translation_to_from);

    cv::Vec3d rectified_x;
    if (!normalize(hires_center_in_tracking, &rectified_x)) {
        return false;
    }

    const cv::Vec3d tracking_forward(0.0, 0.0, 1.0);
    const cv::Vec3d hires_forward_in_tracking =
        hires_to_tracking * tracking_forward;
    const cv::Vec3d average_forward =
        tracking_forward + hires_forward_in_tracking;
    const cv::Vec3d forward_without_baseline =
        average_forward - rectified_x * average_forward.dot(rectified_x);

    cv::Vec3d rectified_z;
    if (!normalize(forward_without_baseline, &rectified_z)) {
        return false;
    }

    cv::Vec3d rectified_y;
    if (!normalize(rectified_z.cross(rectified_x), &rectified_y)) {
        return false;
    }
    rectified_z = rectified_x.cross(rectified_y);

    geometry->rotation_rect_from_tracking = cv::Matx33d(
        rectified_x[0], rectified_x[1], rectified_x[2],
        rectified_y[0], rectified_y[1], rectified_y[2],
        rectified_z[0], rectified_z[1], rectified_z[2]);
    geometry->rotation_rect_from_hires =
        geometry->rotation_rect_from_tracking * hires_to_tracking;
    geometry->baseline_m = cv::norm(hires_center_in_tracking);
    return true;
}

}  // namespace hetero_dfs
