#include "hetero_dfs/calibration.hpp"

#include <modal_json.h>

#include <cmath>
#include <cstring>

namespace hetero_dfs {
namespace {

const double kDegreesToRadians = 3.14159265358979323846 / 180.0;

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

bool json_string_equals(const cJSON* node, const char* expected)
{
    return node != NULL && cJSON_IsString(node) &&
           node->valuestring != NULL &&
           std::strcmp(node->valuestring, expected) == 0;
}

bool read_json_vector_3(const cJSON* node, cv::Vec3d* output)
{
    if (node == NULL || output == NULL || !cJSON_IsArray(node) ||
        cJSON_GetArraySize(node) != 3) {
        return false;
    }
    for (int index = 0; index < 3; ++index) {
        const cJSON* const item = cJSON_GetArrayItem(node, index);
        if (item == NULL || !cJSON_IsNumber(item) ||
            !std::isfinite(item->valuedouble)) {
            return false;
        }
        (*output)[index] = item->valuedouble;
    }
    return true;
}

cv::Matx33d rotation_x(double radians)
{
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    return cv::Matx33d(1.0, 0.0, 0.0,
                       0.0, c, -s,
                       0.0, s, c);
}

cv::Matx33d rotation_y(double radians)
{
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    return cv::Matx33d(c, 0.0, s,
                       0.0, 1.0, 0.0,
                       -s, 0.0, c);
}

cv::Matx33d rotation_z(double radians)
{
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    return cv::Matx33d(c, -s, 0.0,
                       s, c, 0.0,
                       0.0, 0.0, 1.0);
}

cv::Matx33d modalai_rpy_parent_to_child_to_child_to_parent(
    const cv::Vec3d& rpy_degrees)
{
    const double roll = rpy_degrees[0] * kDegreesToRadians;
    const double pitch = rpy_degrees[1] * kDegreesToRadians;
    const double yaw = rpy_degrees[2] * kDegreesToRadians;
    return rotation_x(roll) * rotation_y(pitch) * rotation_z(yaw);
}

bool find_modalai_child_pose(const cJSON* root,
                             const std::string& parent_frame,
                             const std::string& child_frame,
                             cv::Matx33d* rotation_child_to_parent,
                             cv::Vec3d* translation_child_wrt_parent)
{
    if (root == NULL || rotation_child_to_parent == NULL ||
        translation_child_wrt_parent == NULL) {
        return false;
    }
    const cJSON* const extrinsics = cJSON_GetObjectItem(root, "extrinsics");
    if (extrinsics == NULL || !cJSON_IsArray(extrinsics)) {
        return false;
    }

    const int count = cJSON_GetArraySize(extrinsics);
    for (int index = 0; index < count; ++index) {
        const cJSON* const entry = cJSON_GetArrayItem(extrinsics, index);
        if (entry == NULL || !cJSON_IsObject(entry)) {
            continue;
        }
        const cJSON* const parent = cJSON_GetObjectItem(entry, "parent");
        const cJSON* const child = cJSON_GetObjectItem(entry, "child");
        if (!json_string_equals(parent, parent_frame.c_str()) ||
            !json_string_equals(child, child_frame.c_str())) {
            continue;
        }

        cv::Vec3d translation;
        cv::Vec3d rpy_degrees;
        if (!read_json_vector_3(cJSON_GetObjectItem(entry,
                                                    "T_child_wrt_parent"),
                                &translation) ||
            !read_json_vector_3(cJSON_GetObjectItem(entry,
                                                    "RPY_parent_to_child"),
                                &rpy_degrees)) {
            return false;
        }
        *rotation_child_to_parent =
            modalai_rpy_parent_to_child_to_child_to_parent(rpy_degrees);
        *translation_child_wrt_parent = translation;
        return true;
    }
    return false;
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

CalibrationLoad load_modalai_extrinsics(
    const std::string& path,
    const std::string& parent_frame,
    const std::string& from_camera,
    const std::string& to_camera,
    StereoTransform* transform)
{
    if (transform == NULL || parent_frame.empty() || from_camera.empty() ||
        to_camera.empty()) {
        return CalibrationLoad::kInvalidSchema;
    }

    cJSON* const root = json_read_file(path.c_str());
    if (root == NULL) {
        return CalibrationLoad::kCannotOpen;
    }

    cv::Matx33d rotation_from_to_parent;
    cv::Matx33d rotation_to_to_parent;
    cv::Vec3d translation_from_wrt_parent;
    cv::Vec3d translation_to_wrt_parent;
    const bool found_from = find_modalai_child_pose(
        root, parent_frame, from_camera, &rotation_from_to_parent,
        &translation_from_wrt_parent);
    const bool found_to = find_modalai_child_pose(
        root, parent_frame, to_camera, &rotation_to_to_parent,
        &translation_to_wrt_parent);
    cJSON_Delete(root);

    if (!found_from || !found_to) {
        return CalibrationLoad::kWrongCameraNames;
    }

    transform->rotation_to_from =
        rotation_to_to_parent.t() * rotation_from_to_parent;
    transform->translation_to_from =
        rotation_to_to_parent.t() *
        (translation_from_wrt_parent - translation_to_wrt_parent);
    RectificationGeometry geometry;
    return compute_rectification_geometry(*transform, &geometry) ?
        CalibrationLoad::kValid : CalibrationLoad::kInvalidTransform;
}

}  // namespace hetero_dfs
