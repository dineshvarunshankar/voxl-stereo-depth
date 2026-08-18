#ifndef HETERO_DFS_CONFIG_HPP
#define HETERO_DFS_CONFIG_HPP

#include "hetero_dfs/frame.hpp"
#include "hetero_dfs/frame_pairer.hpp"

#include <cstdint>
#include <string>

namespace hetero_dfs {

enum class CameraModel {
    kFisheye,
    kPinholeBrown
};

struct CameraConfig {
    std::string pipe;
    CameraModel model = CameraModel::kFisheye;
    FrameRequirements frame = {PixelFormat::kRaw8, 0, 0};
    std::string intrinsics_file;
};

struct ServiceConfig {
    CameraConfig tracking;
    CameraConfig hires;
    std::string extrinsics_file;
    FramePairerConfig pairing = {0, 0};
    std::uint32_t rectified_width = 0;
    std::uint32_t rectified_height = 0;
    std::int32_t min_disparity = 0;
    std::int32_t disparity_levels = 0;
    float min_depth_m = 0.0F;
    float max_depth_m = 0.0F;
};

enum class ConfigValidation {
    kValid,
    kMissingPipe,
    kWrongCameraModel,
    kInvalidSourceGeometry,
    kMissingCalibration,
    kInvalidPairing,
    kInvalidRectifiedGeometry,
    kInvalidDisparityRange,
    kInvalidDepthRange
};

ConfigValidation validate_config(const ServiceConfig& config);

}  // namespace hetero_dfs

#endif  // HETERO_DFS_CONFIG_HPP
