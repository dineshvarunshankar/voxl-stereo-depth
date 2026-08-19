#ifndef HETERO_DFS_CALIBRATION_HPP
#define HETERO_DFS_CALIBRATION_HPP

#include "hetero_dfs/config.hpp"
#include "hetero_dfs/rectification_geometry.hpp"

#include <opencv2/core.hpp>

#include <cstdint>
#include <string>

namespace hetero_dfs {

struct CameraCalibration {
    std::string camera_name;
    CameraModel model = CameraModel::kFisheye;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    cv::Matx33d camera_matrix = cv::Matx33d::eye();
    cv::Mat distortion;
    double reprojection_error_px = 0.0;
};

struct HeterogeneousStereoCalibration {
    StereoTransform tracking_to_hires;
    int sample_count = 0;
    double rms_reprojection_error_px = 0.0;
    double max_vertical_error_px = 0.0;
};

enum class CalibrationLoad {
    kValid,
    kCannotOpen,
    kInvalidSchema,
    kWrongModel,
    kWrongGeometry,
    kWrongCameraNames,
    kInvalidTransform,
    kInvalidStatistics
};

CalibrationLoad load_camera_calibration(
    const std::string& path,
    const std::string& camera_name,
    CameraModel expected_model,
    const FrameRequirements& expected_frame,
    CameraCalibration* calibration);

CalibrationLoad load_stereo_calibration(
    const std::string& path,
    const std::string& expected_from_camera,
    const std::string& expected_to_camera,
    HeterogeneousStereoCalibration* calibration);

CalibrationLoad load_modalai_extrinsics(
    const std::string& path,
    const std::string& parent_frame,
    const std::string& from_camera,
    const std::string& to_camera,
    StereoTransform* transform);

}  // namespace hetero_dfs

#endif  // HETERO_DFS_CALIBRATION_HPP
