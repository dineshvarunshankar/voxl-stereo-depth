#include "hetero_dfs/calibration.hpp"
#include "hetero_dfs/config.hpp"
#include "hetero_dfs/depth_processor.hpp"
#include "hetero_dfs/frame.hpp"
#include "hetero_dfs/modal_metadata.hpp"

#include <modal_pipe.h>

#include <algorithm>
#include <condition_variable>
#include <cinttypes>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

namespace {

const int kTrackingChannel = 0;
const int kHiresChannel = 1;
const int kDepthChannel = 2;
const char* const kProcessName = "voxl-hetero-dfs-server";
const char* const kDefaultConfigPath = "/etc/modalai/voxl-hetero-dfs-server.conf";

struct TimedFrame {
    hetero_dfs::FrameDescriptor frame;
    std::int64_t center_ns;
    std::vector<std::uint8_t> data;
};

struct WorkItem {
    TimedFrame tracking;
    TimedFrame hires;
};

struct RunningStats {
    std::uint64_t pairs = 0;
    std::uint64_t rejected_hires = 0;
    std::uint64_t waiting_hires = 0;
    std::uint64_t stale_tracking = 0;
    std::uint64_t overflow_tracking = 0;
    std::uint64_t overflow_hires = 0;
    std::uint64_t published_depth = 0;
    std::uint64_t processing_failures = 0;
    std::uint64_t output_write_failures = 0;
    std::int64_t min_signed_delta_ns = 0;
    std::int64_t max_signed_delta_ns = 0;
    std::int64_t max_absolute_delta_ns = 0;
    std::int64_t min_residual_delta_ns = 0;
    std::int64_t max_residual_delta_ns = 0;
    std::int64_t max_absolute_residual_ns = 0;
    long double signed_delta_sum_ns = 0.0L;
    long double absolute_delta_sum_ns = 0.0L;
    long double residual_delta_sum_ns = 0.0L;
    long double absolute_residual_sum_ns = 0.0L;

    void add_pair(std::int64_t signed_delta_ns,
                  std::int64_t absolute_delta_ns,
                  std::int64_t residual_delta_ns,
                  std::int64_t absolute_residual_ns)
    {
        if (pairs == 0U || signed_delta_ns < min_signed_delta_ns) {
            min_signed_delta_ns = signed_delta_ns;
        }
        if (pairs == 0U || signed_delta_ns > max_signed_delta_ns) {
            max_signed_delta_ns = signed_delta_ns;
        }
        if (absolute_delta_ns > max_absolute_delta_ns) {
            max_absolute_delta_ns = absolute_delta_ns;
        }
        if (pairs == 0U || residual_delta_ns < min_residual_delta_ns) {
            min_residual_delta_ns = residual_delta_ns;
        }
        if (pairs == 0U || residual_delta_ns > max_residual_delta_ns) {
            max_residual_delta_ns = residual_delta_ns;
        }
        if (absolute_residual_ns > max_absolute_residual_ns) {
            max_absolute_residual_ns = absolute_residual_ns;
        }
        signed_delta_sum_ns += signed_delta_ns;
        absolute_delta_sum_ns += absolute_delta_ns;
        residual_delta_sum_ns += residual_delta_ns;
        absolute_residual_sum_ns += absolute_residual_ns;
        ++pairs;
    }
};

struct Context {
    Context(const hetero_dfs::ServiceConfig& service_config,
            hetero_dfs::StereoDepthProcessor* depth_processor)
        : config(service_config),
          processor(depth_processor)
    {
    }

    hetero_dfs::ServiceConfig config;
    hetero_dfs::StereoDepthProcessor* processor;
    std::deque<TimedFrame> tracking_queue;
    std::deque<TimedFrame> hires_queue;
    std::deque<WorkItem> work_queue;
    RunningStats stats;
    std::uint64_t invalid[2] = {};
    std::int32_t output_frame_id = 0;
    bool stop_worker = false;
    std::mutex mutex;
    std::condition_variable worker_condition;
};

void usage()
{
    std::fprintf(stderr, "Usage: %s [CONFIG_FILE]\n", kProcessName);
}

bool exposure_center(const hetero_dfs::FrameDescriptor& frame,
                     std::int64_t* center_ns)
{
    if (center_ns == NULL || frame.timestamp_ns < 0 || frame.exposure_ns < 0) {
        return false;
    }
    const std::int64_t half_exposure = frame.exposure_ns / 2;
    if (frame.timestamp_ns >
        std::numeric_limits<std::int64_t>::max() - half_exposure) {
        return false;
    }
    *center_ns = frame.timestamp_ns + half_exposure;
    return true;
}

std::int64_t absolute_delta(std::int64_t value)
{
    return value >= 0 ? value : -value;
}

void prune_old_tracking(Context* context, std::int64_t target_center_ns)
{
    const std::int64_t oldest_useful =
        target_center_ns - context->config.pairing.max_delta_ns;
    while (!context->tracking_queue.empty() &&
           context->tracking_queue.front().center_ns < oldest_useful) {
        context->tracking_queue.pop_front();
        ++context->stats.stale_tracking;
    }
}

std::string pipe_basename(const std::string& path)
{
    std::size_t end = path.find_last_not_of('/');
    if (end == std::string::npos) return path;
    const std::size_t slash = path.find_last_of('/', end);
    if (slash == std::string::npos) return path.substr(0, end + 1U);
    return path.substr(slash + 1U, end - slash);
}

bool copy_string(char* destination, std::size_t destination_size,
                 const std::string& source)
{
    if (destination == NULL || destination_size == 0U ||
        source.size() >= destination_size) {
        return false;
    }
    std::snprintf(destination, destination_size, "%s", source.c_str());
    return true;
}

bool create_depth_pipe(const hetero_dfs::ServiceConfig& config)
{
    pipe_info_t info = PIPE_INFO_INITIALIZER;
    if (!copy_string(info.name, sizeof(info.name),
                     pipe_basename(config.output_pipe)) ||
        !copy_string(info.type, sizeof(info.type),
                     "camera_image_metadata_t") ||
        !copy_string(info.server_name, sizeof(info.server_name),
                     kProcessName)) {
        return false;
    }
    info.size_bytes = pipe_suggest_cam_pipe_size(
        IMAGE_FORMAT_FLOAT32,
        static_cast<int>(config.rectified_width),
        static_cast<int>(config.rectified_height));
    return pipe_server_create(kDepthChannel, info, 0) == 0;
}

void log_summary(const Context& context)
{
    const RunningStats& stats = context.stats;
    const long double mean_signed = stats.pairs == 0U ? 0.0L :
        stats.signed_delta_sum_ns / static_cast<long double>(stats.pairs);
    const long double mean_absolute = stats.pairs == 0U ? 0.0L :
        stats.absolute_delta_sum_ns / static_cast<long double>(stats.pairs);
    const long double mean_residual = stats.pairs == 0U ? 0.0L :
        stats.residual_delta_sum_ns / static_cast<long double>(stats.pairs);
    const long double mean_absolute_residual = stats.pairs == 0U ? 0.0L :
        stats.absolute_residual_sum_ns / static_cast<long double>(stats.pairs);

    std::fprintf(stderr,
        "summary pairs=%" PRIu64 " rejected_hires=%" PRIu64
        " waiting_hires=%" PRIu64
        " invalid_tracking=%" PRIu64 " invalid_hires=%" PRIu64
        " stale_tracking=%" PRIu64
        " published_depth=%" PRIu64 " processing_failures=%" PRIu64 " output_write_failures=%" PRIu64
        " overflow_tracking=%" PRIu64 " overflow_hires=%" PRIu64
        " mean_signed_ns=%.0Lf mean_absolute_ns=%.0Lf"
        " min_signed_ns=%" PRId64 " max_signed_ns=%" PRId64
        " max_absolute_ns=%" PRId64
        " time_offset_ns=%" PRId64
        " mean_residual_ns=%.0Lf mean_absolute_residual_ns=%.0Lf"
        " min_residual_ns=%" PRId64 " max_residual_ns=%" PRId64
        " max_absolute_residual_ns=%" PRId64 "\n",
        stats.pairs, stats.rejected_hires, context.hires_queue.size(),
        context.invalid[0], context.invalid[1], stats.stale_tracking,
        stats.published_depth, stats.processing_failures,
        stats.output_write_failures, stats.overflow_tracking, stats.overflow_hires,
        mean_signed, mean_absolute, stats.min_signed_delta_ns,
        stats.max_signed_delta_ns, stats.max_absolute_delta_ns,
        context.config.pairing.time_offset_ns, mean_residual, mean_absolute_residual,
        stats.min_residual_delta_ns, stats.max_residual_delta_ns,
        stats.max_absolute_residual_ns);
}

void increment_processing_failures(Context* context)
{
    std::lock_guard<std::mutex> lock(context->mutex);
    ++context->stats.processing_failures;
}

void increment_output_write_failures(Context* context)
{
    std::lock_guard<std::mutex> lock(context->mutex);
    ++context->stats.output_write_failures;
}

void increment_published_depth(Context* context)
{
    std::lock_guard<std::mutex> lock(context->mutex);
    ++context->stats.published_depth;
}

bool publish_depth(Context* context, const WorkItem& work)
{
    if (context->processor == NULL || !context->processor->process(
            work.tracking.data.data(), work.tracking.frame.stride,
            work.hires.data.data(), work.hires.frame.stride)) {
        increment_processing_failures(context);
        return false;
    }

    camera_image_metadata_t metadata = {};
    metadata.timestamp_ns =
        std::min(work.tracking.center_ns, work.hires.center_ns) +
        absolute_delta(work.tracking.center_ns - work.hires.center_ns) / 2;
    metadata.frame_id = context->output_frame_id++;
    metadata.width = static_cast<int16_t>(context->processor->width());
    metadata.height = static_cast<int16_t>(context->processor->height());
    metadata.stride =
        static_cast<int32_t>(context->processor->width() * sizeof(float));
    metadata.size_bytes = static_cast<int32_t>(
        context->processor->depth_m().size() * sizeof(float));
    metadata.exposure_ns = 0;
    metadata.format = IMAGE_FORMAT_FLOAT32;

    if (pipe_server_write_camera_frame(
            kDepthChannel, metadata,
            context->processor->depth_m().data()) != 0) {
        increment_output_write_failures(context);
        return false;
    }
    increment_published_depth(context);
    return true;
}

void worker_loop(Context* context)
{
    while (true) {
        WorkItem work;
        {
            std::unique_lock<std::mutex> lock(context->mutex);
            context->worker_condition.wait(lock, [context]() {
                return context->stop_worker || !context->work_queue.empty();
            });
            if (context->stop_worker && context->work_queue.empty()) {
                return;
            }
            work = context->work_queue.front();
            context->work_queue.pop_front();
        }
        publish_depth(context, work);
    }
}

void process_hires_queue(Context* context)
{
    while (!context->hires_queue.empty() &&
           !context->tracking_queue.empty()) {
        const TimedFrame& hires = context->hires_queue.front();
        const std::int64_t target_center_ns =
            hires.center_ns + context->config.pairing.time_offset_ns;

        if (context->tracking_queue.back().center_ns <
            target_center_ns + context->config.pairing.max_delta_ns) {
            ++context->stats.waiting_hires;
            return;
        }

        std::size_t best_index = 0U;
        std::int64_t best_absolute_delta =
            std::numeric_limits<std::int64_t>::max();
        for (std::size_t index = 0U;
             index < context->tracking_queue.size(); ++index) {
            const std::int64_t signed_delta =
                context->tracking_queue[index].center_ns - target_center_ns;
            const std::int64_t current_absolute_delta =
                absolute_delta(signed_delta);
            if (current_absolute_delta < best_absolute_delta) {
                best_index = index;
                best_absolute_delta = current_absolute_delta;
            }
        }

        if (best_absolute_delta <= context->config.pairing.max_delta_ns) {
            const TimedFrame tracking = context->tracking_queue[best_index];
            const TimedFrame accepted_hires = hires;
            const std::int64_t signed_delta =
                tracking.center_ns - accepted_hires.center_ns;
            const std::int64_t residual_delta =
                tracking.center_ns - target_center_ns;
            const std::int64_t absolute_residual = absolute_delta(residual_delta);
            context->stats.add_pair(signed_delta, absolute_delta(signed_delta),
                                    residual_delta, absolute_residual);
            context->work_queue.clear();
            context->work_queue.push_back(WorkItem{tracking, accepted_hires});
            context->worker_condition.notify_one();
            context->stats.stale_tracking += best_index;
            context->tracking_queue.erase(
                context->tracking_queue.begin(),
                context->tracking_queue.begin() + best_index + 1);
            context->hires_queue.pop_front();
        } else {
            ++context->stats.rejected_hires;
            prune_old_tracking(context, target_center_ns);
            context->hires_queue.pop_front();
        }
    }
}

void push_tracking(Context* context, const TimedFrame& frame)
{
    context->tracking_queue.push_back(frame);
    while (context->tracking_queue.size() > context->config.pairing.max_queue_depth) {
        context->tracking_queue.pop_front();
        ++context->stats.overflow_tracking;
    }
    process_hires_queue(context);
}

void push_hires(Context* context, const TimedFrame& frame)
{
    context->hires_queue.push_back(frame);
    while (context->hires_queue.size() > context->config.pairing.max_queue_depth) {
        context->hires_queue.pop_front();
        ++context->stats.overflow_hires;
        ++context->stats.rejected_hires;
    }
    process_hires_queue(context);
}

void callback(int channel, camera_image_metadata_t metadata,
              char* data, void* opaque)
{
    (void)data;
    Context* const context = static_cast<Context*>(opaque);
    const int camera_index = channel == kTrackingChannel ? 0 : 1;
    const hetero_dfs::FrameRequirements& requirements = camera_index == 0 ?
        context->config.tracking.frame : context->config.hires.frame;
    const hetero_dfs::FrameDescriptor frame =
        hetero_dfs::frame_from_modal_metadata(metadata);

    std::lock_guard<std::mutex> lock(context->mutex);
    if (hetero_dfs::validate_frame(frame, requirements) !=
        hetero_dfs::FrameValidation::kValid) {
        ++context->invalid[camera_index];
        return;
    }
    std::int64_t frame_center_ns = 0;
    if (!exposure_center(frame, &frame_center_ns)) {
        ++context->invalid[camera_index];
        return;
    }

    if (data == NULL) {
        ++context->invalid[camera_index];
        return;
    }

    const std::size_t required_size = frame.stride * frame.height;
    TimedFrame timed_frame;
    timed_frame.frame = frame;
    timed_frame.center_ns = frame_center_ns;
    timed_frame.data.assign(reinterpret_cast<std::uint8_t*>(data),
                            reinterpret_cast<std::uint8_t*>(data) +
                                required_size);

    if (camera_index == 0) {
        push_tracking(context, timed_frame);
    } else {
        push_hires(context, timed_frame);
    }
}

const char* calibration_load_name(hetero_dfs::CalibrationLoad result)
{
    switch (result) {
    case hetero_dfs::CalibrationLoad::kValid: return "valid";
    case hetero_dfs::CalibrationLoad::kCannotOpen: return "cannot_open";
    case hetero_dfs::CalibrationLoad::kInvalidSchema: return "invalid_schema";
    case hetero_dfs::CalibrationLoad::kWrongModel: return "wrong_model";
    case hetero_dfs::CalibrationLoad::kWrongGeometry: return "wrong_geometry";
    case hetero_dfs::CalibrationLoad::kWrongCameraNames: return "wrong_camera_names";
    case hetero_dfs::CalibrationLoad::kInvalidTransform: return "invalid_transform";
    case hetero_dfs::CalibrationLoad::kInvalidStatistics: return "invalid_statistics";
    }
    return "unknown";
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc > 2) {
        usage();
        return EXIT_FAILURE;
    }
    const char* const config_path = argc == 2 ? argv[1] : kDefaultConfigPath;

    hetero_dfs::ServiceConfig config;
    if (hetero_dfs::load_service_config(config_path, &config) !=
        hetero_dfs::ConfigValidation::kValid) {
        std::fprintf(stderr, "failed to load config: %s\n", config_path);
        return EXIT_FAILURE;
    }

    hetero_dfs::CameraCalibration tracking_calibration;
    hetero_dfs::CameraCalibration hires_calibration;
    hetero_dfs::CalibrationLoad load_result = hetero_dfs::load_camera_calibration(
        config.tracking.intrinsics_file, config.tracking_name,
        hetero_dfs::CameraModel::kFisheye, config.tracking.frame,
        &tracking_calibration);
    if (load_result != hetero_dfs::CalibrationLoad::kValid) {
        std::fprintf(stderr, "failed to load tracking intrinsics: %s (%s)\n",
                     config.tracking.intrinsics_file.c_str(),
                     calibration_load_name(load_result));
        return EXIT_FAILURE;
    }
    load_result = hetero_dfs::load_camera_calibration(
        config.hires.intrinsics_file, config.hires_name,
        hetero_dfs::CameraModel::kPinholeBrown, config.hires.frame,
        &hires_calibration);
    if (load_result != hetero_dfs::CalibrationLoad::kValid) {
        std::fprintf(stderr, "failed to load hires intrinsics: %s (%s)\n",
                     config.hires.intrinsics_file.c_str(),
                     calibration_load_name(load_result));
        return EXIT_FAILURE;
    }
    hetero_dfs::StereoTransform tracking_to_hires;
    load_result = hetero_dfs::load_modalai_extrinsics(
        config.extrinsics_file, config.parent_frame, config.tracking_name,
        config.hires_name, &tracking_to_hires);
    if (load_result != hetero_dfs::CalibrationLoad::kValid) {
        std::fprintf(stderr, "failed to load extrinsics: %s (%s)\n",
                     config.extrinsics_file.c_str(),
                     calibration_load_name(load_result));
        return EXIT_FAILURE;
    }

    hetero_dfs::StereoDepthProcessor processor(
        config, tracking_calibration, hires_calibration, tracking_to_hires);
    if (!processor.initialized()) {
        std::fprintf(stderr, "failed to initialize stereo depth processor\n");
        return EXIT_FAILURE;
    }

    if (!create_depth_pipe(config)) {
        std::fprintf(stderr, "failed to create output pipe: %s\n",
                     config.output_pipe.c_str());
        return EXIT_FAILURE;
    }

    std::fprintf(stderr,
                 "configured output=%s rectified=%ux%u baseline_m=%.6f fx_px=%.3f max_delta_ns=%" PRId64 " time_offset_ns=%" PRId64 "\n",
                 config.output_pipe.c_str(), processor.width(), processor.height(),
                 processor.baseline_m(), processor.rectified_focal_px(),
                 config.pairing.max_delta_ns, config.pairing.time_offset_ns);

    Context context(config, &processor);
    std::thread worker(worker_loop, &context);

    if (enable_signal_handler() != 0) {
        {
            std::lock_guard<std::mutex> lock(context.mutex);
            context.stop_worker = true;
        }
        context.worker_condition.notify_one();
        worker.join();
        return EXIT_FAILURE;
    }
    pipe_client_set_camera_helper_cb(kTrackingChannel, callback, &context);
    pipe_client_set_camera_helper_cb(kHiresChannel, callback, &context);

    main_running = 1;
    const int flags = CLIENT_FLAG_EN_CAMERA_HELPER;
    if (pipe_client_open(kTrackingChannel, config.tracking.pipe.c_str(),
                         kProcessName, flags, 0) < 0 ||
        pipe_client_open(kHiresChannel, config.hires.pipe.c_str(),
                         kProcessName, flags, 0) < 0) {
        main_running = 0;
    }

    while (main_running) usleep(100000);
    pipe_client_close_all();
    {
        std::lock_guard<std::mutex> lock(context.mutex);
        context.stop_worker = true;
    }
    context.worker_condition.notify_one();
    worker.join();
    pipe_server_close(kDepthChannel);
    log_summary(context);
    return context.stats.published_depth > 0U ? EXIT_SUCCESS : EXIT_FAILURE;
}
