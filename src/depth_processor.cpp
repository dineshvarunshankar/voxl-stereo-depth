#include "hetero_dfs/depth_processor.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace hetero_dfs {
namespace {

double scaled_fx(const CameraCalibration& calibration, std::uint32_t width)
{
    return calibration.camera_matrix(0, 0) *
        static_cast<double>(width) / static_cast<double>(calibration.width);
}

double scaled_fy(const CameraCalibration& calibration, std::uint32_t height)
{
    return calibration.camera_matrix(1, 1) *
        static_cast<double>(height) / static_cast<double>(calibration.height);
}

cv::Matx33d rectified_camera_matrix(const ServiceConfig& config,
                                    const CameraCalibration& tracking,
                                    const CameraCalibration& hires,
                                    double* focal_px)
{
    const double focal = std::min(
        std::min(scaled_fx(tracking, config.rectified_width),
                 scaled_fy(tracking, config.rectified_height)),
        std::min(scaled_fx(hires, config.rectified_width),
                 scaled_fy(hires, config.rectified_height)));
    if (focal_px != NULL) {
        *focal_px = focal;
    }
    return cv::Matx33d(focal, 0.0,
                       (static_cast<double>(config.rectified_width) - 1.0) * 0.5,
                       0.0, focal,
                       (static_cast<double>(config.rectified_height) - 1.0) * 0.5,
                       0.0, 0.0, 1.0);
}

}  // namespace

float fixed_point_disparity_to_depth_m(std::int16_t disparity_fixed16,
                                       double focal_px,
                                       double baseline_m,
                                       float min_depth_m,
                                       float max_depth_m)
{
    if (disparity_fixed16 <= 0 || focal_px <= 0.0 || baseline_m <= 0.0) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    const double disparity_px =
        static_cast<double>(disparity_fixed16) / 16.0;
    const double depth = focal_px * baseline_m / disparity_px;
    if (!std::isfinite(depth) || depth < min_depth_m || depth > max_depth_m) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    return static_cast<float>(depth);
}

bool convert_disparity_to_depth_m(const cv::Mat& disparity_fixed16,
                                  double focal_px,
                                  double baseline_m,
                                  float min_depth_m,
                                  float max_depth_m,
                                  std::vector<float>* depth_m)
{
    if (depth_m == NULL || disparity_fixed16.empty() ||
        disparity_fixed16.type() != CV_16SC1 ||
        disparity_fixed16.rows <= 0 || disparity_fixed16.cols <= 0) {
        return false;
    }
    const std::size_t pixel_count =
        static_cast<std::size_t>(disparity_fixed16.rows) *
        static_cast<std::size_t>(disparity_fixed16.cols);
    depth_m->assign(pixel_count, std::numeric_limits<float>::quiet_NaN());
    for (int row = 0; row < disparity_fixed16.rows; ++row) {
        const std::int16_t* const disparity_row =
            disparity_fixed16.ptr<std::int16_t>(row);
        for (int column = 0; column < disparity_fixed16.cols; ++column) {
            (*depth_m)[static_cast<std::size_t>(row) *
                       static_cast<std::size_t>(disparity_fixed16.cols) +
                       static_cast<std::size_t>(column)] =
                fixed_point_disparity_to_depth_m(
                    disparity_row[column], focal_px, baseline_m,
                    min_depth_m, max_depth_m);
        }
    }
    return true;
}

StereoDepthProcessor::StereoDepthProcessor(
    const ServiceConfig& config,
    const CameraCalibration& tracking,
    const CameraCalibration& hires,
    const StereoTransform& tracking_to_hires)
    : config_(config)
{
    if (validate_config(config_) != ConfigValidation::kValid ||
        tracking.model != CameraModel::kFisheye ||
        hires.model != CameraModel::kPinholeBrown ||
        !compute_rectification_geometry(tracking_to_hires, &geometry_)) {
        return;
    }

    const cv::Size output_size(static_cast<int>(config_.rectified_width),
                               static_cast<int>(config_.rectified_height));
    const cv::Matx33d new_camera_matrix = rectified_camera_matrix(
        config_, tracking, hires, &rectified_focal_px_);
    if (!std::isfinite(rectified_focal_px_) || rectified_focal_px_ <= 0.0) {
        return;
    }

    try {
        cv::fisheye::initUndistortRectifyMap(
            tracking.camera_matrix, tracking.distortion,
            geometry_.rotation_rect_from_tracking, new_camera_matrix,
            output_size, CV_16SC2, tracking_map_x_, tracking_map_y_);

        cv::initUndistortRectifyMap(
            hires.camera_matrix, hires.distortion,
            geometry_.rotation_rect_from_hires, new_camera_matrix,
            output_size, CV_16SC2, hires_map_x_, hires_map_y_);

        const int channels = 1;
        const int p1 = 8 * channels * config_.block_size * config_.block_size;
        const int p2 = 32 * channels * config_.block_size * config_.block_size;
        matcher_ = cv::StereoSGBM::create(
            config_.min_disparity,
            config_.disparity_levels,
            config_.block_size,
            p1,
            p2,
            config_.disp12_max_diff,
            0,
            config_.uniqueness_ratio,
            config_.speckle_window_size,
            config_.speckle_range,
            cv::StereoSGBM::MODE_SGBM_3WAY);
    } catch (const cv::Exception&) {
        matcher_.release();
        return;
    }

    depth_m_.assign(static_cast<std::size_t>(config_.rectified_width) *
                        static_cast<std::size_t>(config_.rectified_height),
                    std::numeric_limits<float>::quiet_NaN());
    initialized_ = !tracking_map_x_.empty() && !hires_map_x_.empty() &&
                   !matcher_.empty();
}

bool StereoDepthProcessor::initialized() const
{
    return initialized_;
}

bool StereoDepthProcessor::process(const std::uint8_t* tracking_data,
                                   std::size_t tracking_stride,
                                   const std::uint8_t* hires_data,
                                   std::size_t hires_stride)
{
    if (!initialized_ || tracking_data == NULL || hires_data == NULL ||
        tracking_stride < config_.tracking.frame.width ||
        hires_stride < config_.hires.frame.width) {
        return false;
    }

    try {
        const cv::Mat tracking_source(
            static_cast<int>(config_.tracking.frame.height),
            static_cast<int>(config_.tracking.frame.width), CV_8UC1,
            const_cast<std::uint8_t*>(tracking_data), tracking_stride);
        const cv::Mat hires_source(
            static_cast<int>(config_.hires.frame.height),
            static_cast<int>(config_.hires.frame.width), CV_8UC1,
            const_cast<std::uint8_t*>(hires_data), hires_stride);

        cv::remap(tracking_source, tracking_rectified_, tracking_map_x_,
                  tracking_map_y_, cv::INTER_LINEAR,
                  cv::BORDER_CONSTANT, cv::Scalar(0));
        cv::remap(hires_source, hires_rectified_, hires_map_x_, hires_map_y_,
                  cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
        matcher_->compute(tracking_rectified_, hires_rectified_,
                          disparity_fixed16_);
    } catch (const cv::Exception&) {
        return false;
    }
    return convert_disparity_to_depth_m(
        disparity_fixed16_, rectified_focal_px_, geometry_.baseline_m,
        config_.min_depth_m, config_.max_depth_m, &depth_m_);
}

const std::vector<float>& StereoDepthProcessor::depth_m() const
{
    return depth_m_;
}

std::uint32_t StereoDepthProcessor::width() const { return config_.rectified_width; }
std::uint32_t StereoDepthProcessor::height() const { return config_.rectified_height; }
double StereoDepthProcessor::rectified_focal_px() const { return rectified_focal_px_; }
double StereoDepthProcessor::baseline_m() const { return geometry_.baseline_m; }

}  // namespace hetero_dfs
