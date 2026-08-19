#ifndef HETERO_DFS_DEPTH_PROCESSOR_HPP
#define HETERO_DFS_DEPTH_PROCESSOR_HPP

#include "hetero_dfs/calibration.hpp"
#include "hetero_dfs/config.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace hetero_dfs {

float fixed_point_disparity_to_depth_m(std::int16_t disparity_fixed16,
                                       double focal_px,
                                       double baseline_m,
                                       float min_depth_m,
                                       float max_depth_m);

bool convert_disparity_to_depth_m(const cv::Mat& disparity_fixed16,
                                  double focal_px,
                                  double baseline_m,
                                  float min_depth_m,
                                  float max_depth_m,
                                  std::vector<float>* depth_m);

class StereoDepthProcessor {
public:
    StereoDepthProcessor(const ServiceConfig& config,
                         const CameraCalibration& tracking,
                         const CameraCalibration& hires,
                         const StereoTransform& tracking_to_hires);

    bool initialized() const;
    bool process(const std::uint8_t* tracking_data,
                 std::size_t tracking_stride,
                 const std::uint8_t* hires_data,
                 std::size_t hires_stride);

    const std::vector<float>& depth_m() const;
    std::uint32_t width() const;
    std::uint32_t height() const;
    double rectified_focal_px() const;
    double baseline_m() const;

private:
    ServiceConfig config_;
    RectificationGeometry geometry_;
    cv::Mat tracking_map_x_;
    cv::Mat tracking_map_y_;
    cv::Mat hires_map_x_;
    cv::Mat hires_map_y_;
    cv::Mat tracking_rectified_;
    cv::Mat hires_rectified_;
    cv::Mat disparity_fixed16_;
    cv::Ptr<cv::StereoSGBM> matcher_;
    std::vector<float> depth_m_;
    double rectified_focal_px_ = 0.0;
    bool initialized_ = false;
};

}  // namespace hetero_dfs

#endif  // HETERO_DFS_DEPTH_PROCESSOR_HPP
