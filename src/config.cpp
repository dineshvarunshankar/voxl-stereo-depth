#include "hetero_dfs/config.hpp"

#include <modal_json.h>

#include <cmath>
#include <climits>

namespace hetero_dfs {
namespace {

bool get_string(cJSON* root, const char* key, std::string* output)
{
    cJSON* const item = cJSON_GetObjectItem(root, key);
    if (item == NULL || !cJSON_IsString(item) || item->valuestring == NULL) {
        return false;
    }
    *output = item->valuestring;
    return true;
}

bool get_int64(cJSON* root, const char* key, std::int64_t* output)
{
    cJSON* const item = cJSON_GetObjectItem(root, key);
    if (item == NULL || !cJSON_IsNumber(item) ||
        !std::isfinite(item->valuedouble) ||
        item->valuedouble < static_cast<double>(LLONG_MIN) ||
        item->valuedouble > static_cast<double>(LLONG_MAX)) {
        return false;
    }
    *output = static_cast<std::int64_t>(item->valuedouble);
    return true;
}

bool get_int32(cJSON* root, const char* key, std::int32_t* output)
{
    std::int64_t value = 0;
    if (!get_int64(root, key, &value) || value < INT_MIN || value > INT_MAX) {
        return false;
    }
    *output = static_cast<std::int32_t>(value);
    return true;
}

bool get_uint32(cJSON* root, const char* key, std::uint32_t* output)
{
    std::int64_t value = 0;
    if (!get_int64(root, key, &value) || value <= 0 ||
        value > static_cast<std::int64_t>(UINT32_MAX)) {
        return false;
    }
    *output = static_cast<std::uint32_t>(value);
    return true;
}

bool get_size(cJSON* root, const char* key, std::size_t* output)
{
    std::int64_t value = 0;
    if (!get_int64(root, key, &value) || value <= 0) {
        return false;
    }
    *output = static_cast<std::size_t>(value);
    return true;
}

bool get_float(cJSON* root, const char* key, float* output)
{
    cJSON* const item = cJSON_GetObjectItem(root, key);
    if (item == NULL || !cJSON_IsNumber(item) ||
        !std::isfinite(item->valuedouble)) {
        return false;
    }
    *output = static_cast<float>(item->valuedouble);
    return true;
}

bool valid_source_geometry(const CameraConfig& camera)
{
    return camera.frame.format == PixelFormat::kRaw8 &&
           camera.frame.width > 0U &&
           camera.frame.height > 0U;
}

bool valid_stereo_matcher(const ServiceConfig& config)
{
    return config.block_size >= 3 &&
           config.block_size <= 11 &&
           (config.block_size % 2) == 1 &&
           config.uniqueness_ratio >= 0 &&
           config.uniqueness_ratio <= 100 &&
           config.speckle_window_size >= 0 &&
           config.speckle_range >= 0 &&
           config.disp12_max_diff >= -1;
}

}  // namespace

ConfigValidation load_service_config(const std::string& path,
                                     ServiceConfig* config)
{
    if (config == NULL) {
        return ConfigValidation::kInvalidSchema;
    }

    cJSON* const root = json_read_file(path.c_str());
    if (root == NULL) {
        return ConfigValidation::kCannotOpen;
    }

    ServiceConfig loaded;
    loaded.tracking.model = CameraModel::kFisheye;
    loaded.hires.model = CameraModel::kPinholeBrown;
    loaded.tracking.frame.format = PixelFormat::kRaw8;
    loaded.hires.frame.format = PixelFormat::kRaw8;

    const bool ok =
        get_string(root, "tracking_pipe", &loaded.tracking.pipe) &&
        get_string(root, "hires_pipe", &loaded.hires.pipe) &&
        get_string(root, "output_pipe", &loaded.output_pipe) &&
        get_string(root, "parent_frame", &loaded.parent_frame) &&
        get_string(root, "tracking_name", &loaded.tracking_name) &&
        get_string(root, "hires_name", &loaded.hires_name) &&
        get_string(root, "tracking_intrinsics", &loaded.tracking.intrinsics_file) &&
        get_string(root, "hires_intrinsics", &loaded.hires.intrinsics_file) &&
        get_string(root, "extrinsics", &loaded.extrinsics_file) &&
        get_uint32(root, "tracking_width", &loaded.tracking.frame.width) &&
        get_uint32(root, "tracking_height", &loaded.tracking.frame.height) &&
        get_uint32(root, "hires_width", &loaded.hires.frame.width) &&
        get_uint32(root, "hires_height", &loaded.hires.frame.height) &&
        get_int64(root, "max_delta_ns", &loaded.pairing.max_delta_ns) &&
        get_int64(root, "time_offset_ns", &loaded.pairing.time_offset_ns) &&
        get_size(root, "queue_depth", &loaded.pairing.max_queue_depth) &&
        get_uint32(root, "rectified_width", &loaded.rectified_width) &&
        get_uint32(root, "rectified_height", &loaded.rectified_height) &&
        get_int32(root, "min_disparity", &loaded.min_disparity) &&
        get_int32(root, "disparity_levels", &loaded.disparity_levels) &&
        get_float(root, "min_depth_m", &loaded.min_depth_m) &&
        get_float(root, "max_depth_m", &loaded.max_depth_m) &&
        get_int32(root, "block_size", &loaded.block_size) &&
        get_int32(root, "uniqueness_ratio", &loaded.uniqueness_ratio) &&
        get_int32(root, "speckle_window_size", &loaded.speckle_window_size) &&
        get_int32(root, "speckle_range", &loaded.speckle_range) &&
        get_int32(root, "disp12_max_diff", &loaded.disp12_max_diff);
    cJSON_Delete(root);

    if (!ok) {
        return ConfigValidation::kInvalidSchema;
    }
    const ConfigValidation validation = validate_config(loaded);
    if (validation != ConfigValidation::kValid) {
        return validation;
    }
    *config = loaded;
    return ConfigValidation::kValid;
}

ConfigValidation validate_config(const ServiceConfig& config)
{
    if (config.tracking.pipe.empty() || config.hires.pipe.empty() ||
        config.output_pipe.empty()) {
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
    if (config.parent_frame.empty() ||
        config.tracking_name.empty() ||
        config.hires_name.empty()) {
        return ConfigValidation::kMissingCameraName;
    }
    if (config.pairing.max_delta_ns < 0 ||
        config.pairing.max_queue_depth < 2U) {
        return ConfigValidation::kInvalidPairing;
    }
    if (config.rectified_width == 0U || config.rectified_height == 0U) {
        return ConfigValidation::kInvalidRectifiedGeometry;
    }
    if (config.min_disparity < 0 ||
        config.disparity_levels < 1 ||
        config.disparity_levels > 64 ||
        (config.disparity_levels % 16) != 0 ||
        config.min_disparity > 254 - config.disparity_levels) {
        return ConfigValidation::kInvalidDisparityRange;
    }
    if (!std::isfinite(config.min_depth_m) ||
        !std::isfinite(config.max_depth_m) ||
        config.min_depth_m <= 0.0F ||
        config.max_depth_m <= config.min_depth_m) {
        return ConfigValidation::kInvalidDepthRange;
    }
    if (!valid_stereo_matcher(config)) {
        return ConfigValidation::kInvalidStereoMatcher;
    }
    return ConfigValidation::kValid;
}

}  // namespace hetero_dfs
