#include "hetero_dfs/config.hpp"

#include <cmath>

namespace hetero_dfs {
namespace {

bool valid_source_geometry(const CameraConfig& camera)
{
    return camera.frame.format == PixelFormat::kRaw8 &&
           camera.frame.width > 0U &&
           camera.frame.height > 0U;
}

}  // namespace

ConfigValidation validate_config(const ServiceConfig& config)
{
    if (config.tracking.pipe.empty() || config.hires.pipe.empty()) {
        return ConfigValidation::kMissingPipe;
    }
    if (config.tracking.model != CameraModel::kFisheye ||
        config.hires.model != CameraModel::kPinholeBrown) {
        return ConfigValidation::kWrongCameraModel;
    }
    if (!valid_source_geometry(config.tracking) ||
        !valid_source_geometry(config.hires)) {
        return ConfigValidation::kInvalidSourceGeometry;
    }
    if (config.tracking.intrinsics_file.empty() ||
        config.hires.intrinsics_file.empty() ||
        config.extrinsics_file.empty()) {
        return ConfigValidation::kMissingCalibration;
    }
    if (config.pairing.max_delta_ns < 0 ||
        config.pairing.max_queue_depth == 0U) {
        return ConfigValidation::kInvalidPairing;
    }
    if (config.rectified_width == 0U || config.rectified_height == 0U) {
        return ConfigValidation::kInvalidRectifiedGeometry;
    }
    if (config.min_disparity < 0 ||
        config.disparity_levels < 1 ||
        config.disparity_levels > 64 ||
        config.min_disparity > 254 - config.disparity_levels) {
        return ConfigValidation::kInvalidDisparityRange;
    }
    if (!std::isfinite(config.min_depth_m) ||
        !std::isfinite(config.max_depth_m) ||
        config.min_depth_m <= 0.0F ||
        config.max_depth_m <= config.min_depth_m) {
        return ConfigValidation::kInvalidDepthRange;
    }
    return ConfigValidation::kValid;
}

}  // namespace hetero_dfs
