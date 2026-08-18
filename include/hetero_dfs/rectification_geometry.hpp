#ifndef HETERO_DFS_RECTIFICATION_GEOMETRY_HPP
#define HETERO_DFS_RECTIFICATION_GEOMETRY_HPP

#include <opencv2/core.hpp>

namespace hetero_dfs {

struct StereoTransform {
    cv::Matx33d rotation_to_from = cv::Matx33d::eye();
    cv::Vec3d translation_to_from = cv::Vec3d(0.0, 0.0, 0.0);
};

struct RectificationGeometry {
    cv::Matx33d rotation_rect_from_tracking = cv::Matx33d::eye();
    cv::Matx33d rotation_rect_from_hires = cv::Matx33d::eye();
    double baseline_m = 0.0;
};

bool compute_rectification_geometry(
    const StereoTransform& tracking_to_hires,
    RectificationGeometry* geometry);

}  // namespace hetero_dfs

#endif  // HETERO_DFS_RECTIFICATION_GEOMETRY_HPP
