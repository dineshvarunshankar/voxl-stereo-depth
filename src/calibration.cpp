#include "hetero_dfs/calibration.hpp"

#include <cmath>

namespace hetero_dfs {
namespace {

bool finite_matrix(const cv::Mat& matrix)
{
    cv::Mat values;
    matrix.convertTo(values, CV_64F);
    for (int row = 0; row < values.rows; ++row) {
        for (int column = 0; column < values.cols; ++column) {
            if (!std::isfinite(values.at<double>(row, column))) {
                return false;
            }
        }
    }
    return true;
}

bool read_matrix_3x3(const cv::FileNode& node, cv::Matx33d* output)
{
    cv::Mat matrix;
    node >> matrix;
    if (matrix.rows != 3 || matrix.cols != 3 || !finite_matrix(matrix)) {
        return false;
    }
    cv::Mat values;
    matrix.convertTo(values, CV_64F);
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            (*output)(row, column) = values.at<double>(row, column);
        }
    }
    return true;
}

bool read_vector_3(const cv::FileNode& node, cv::Vec3d* output)
{
    cv::Mat vector;
    node >> vector;
    if (vector.total() != 3U || !finite_matrix(vector)) {
        return false;
    }
    cv::Mat values;
    vector.reshape(1, 3).convertTo(values, CV_64F);
    for (int index = 0; index < 3; ++index) {
        (*output)[index] = values.at<double>(index, 0);
    }
    return true;
}

bool camera_model_from_string(const std::string& name, CameraModel* model)
{
    if (name == "fisheye") {
        *model = CameraModel::kFisheye;
        return true;
    }
    if (name == "plumb_bob") {
        *model = CameraModel::kPinholeBrown;
        return true;
    }
    return false;
}

bool valid_camera_matrix(const cv::Matx33d& matrix)
{
    if (matrix(0, 0) <= 0.0 || matrix(1, 1) <= 0.0) {
        return false;
    }
    return std::abs(matrix(2, 0)) <= 1.0e-12 &&
           std::abs(matrix(2, 1)) <= 1.0e-12 &&
           std::abs(matrix(2, 2) - 1.0) <= 1.0e-12;
}

}  // namespace

CalibrationLoad load_camera_calibration(
    const std::string& path,
    const std::string& camera_name,
    CameraModel expected_model,
    const FrameRequirements& expected_frame,
    CameraCalibration* calibration)
{
    if (calibration == NULL) {
        return CalibrationLoad::kInvalidSchema;
    }

    try {
        const cv::FileStorage file(path, cv::FileStorage::READ);
        if (!file.isOpened()) {
            return CalibrationLoad::kCannotOpen;
        }

        int width = 0;
        int height = 0;
        std::string model_name;
        double reprojection_error = 0.0;
        file["width"] >> width;
        file["height"] >> height;
        file["distortion_model"] >> model_name;
        file["reprojection_error"] >> reprojection_error;

        CameraModel model;
        if (!camera_model_from_string(model_name, &model)) {
            return CalibrationLoad::kInvalidSchema;
        }
        if (model != expected_model) {
            return CalibrationLoad::kWrongModel;
        }
        if (expected_frame.format != PixelFormat::kRaw8 ||
            width <= 0 || height <= 0 ||
            static_cast<std::uint32_t>(width) != expected_frame.width ||
            static_cast<std::uint32_t>(height) != expected_frame.height) {
            return CalibrationLoad::kWrongGeometry;
        }

        cv::Matx33d camera_matrix;
        if (!read_matrix_3x3(file["M"], &camera_matrix) ||
            !valid_camera_matrix(camera_matrix)) {
            return CalibrationLoad::kInvalidSchema;
        }

        cv::Mat distortion;
        file["D"] >> distortion;
        const std::size_t expected_coefficients =
            model == CameraModel::kFisheye ? 4U : 5U;
        if (distortion.total() != expected_coefficients ||
            !finite_matrix(distortion) ||
            !std::isfinite(reprojection_error) ||
            reprojection_error < 0.0) {
            return CalibrationLoad::kInvalidSchema;
        }

        calibration->camera_name = camera_name;
        calibration->model = model;
        calibration->width = static_cast<std::uint32_t>(width);
        calibration->height = static_cast<std::uint32_t>(height);
        calibration->camera_matrix = camera_matrix;
        distortion.convertTo(calibration->distortion, CV_64F);
        calibration->reprojection_error_px = reprojection_error;
        return CalibrationLoad::kValid;
    } catch (const cv::Exception&) {
        return CalibrationLoad::kInvalidSchema;
    }
}

CalibrationLoad load_stereo_calibration(
    const std::string& path,
    const std::string& expected_from_camera,
    const std::string& expected_to_camera,
    HeterogeneousStereoCalibration* calibration)
{
    if (calibration == NULL) {
        return CalibrationLoad::kInvalidSchema;
    }

    try {
        const cv::FileStorage file(path, cv::FileStorage::READ);
        if (!file.isOpened()) {
            return CalibrationLoad::kCannotOpen;
        }

        int version = 0;
        std::string from_camera;
        std::string to_camera;
        file["schema_version"] >> version;
        file["from_camera"] >> from_camera;
        file["to_camera"] >> to_camera;
        if (version != 1) {
            return CalibrationLoad::kInvalidSchema;
        }
        if (from_camera != expected_from_camera ||
            to_camera != expected_to_camera) {
            return CalibrationLoad::kWrongCameraNames;
        }

        StereoTransform transform;
        if (!read_matrix_3x3(file["R_to_from"],
                             &transform.rotation_to_from) ||
            !read_vector_3(file["T_to_from_m"],
                           &transform.translation_to_from)) {
            return CalibrationLoad::kInvalidSchema;
        }
        RectificationGeometry geometry;
        if (!compute_rectification_geometry(transform, &geometry)) {
            return CalibrationLoad::kInvalidTransform;
        }

        int sample_count = 0;
        double rms_error = 0.0;
        double max_vertical_error = 0.0;
        file["sample_count"] >> sample_count;
        file["rms_reprojection_error_px"] >> rms_error;
        file["max_vertical_error_px"] >> max_vertical_error;
        if (sample_count <= 0 ||
            !std::isfinite(rms_error) || rms_error < 0.0 ||
            !std::isfinite(max_vertical_error) ||
            max_vertical_error < 0.0) {
            return CalibrationLoad::kInvalidStatistics;
        }

        calibration->tracking_to_hires = transform;
        calibration->sample_count = sample_count;
        calibration->rms_reprojection_error_px = rms_error;
        calibration->max_vertical_error_px = max_vertical_error;
        return CalibrationLoad::kValid;
    } catch (const cv::Exception&) {
        return CalibrationLoad::kInvalidSchema;
    }
}

}  // namespace hetero_dfs
