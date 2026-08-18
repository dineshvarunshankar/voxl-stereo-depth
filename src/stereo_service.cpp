#include "hetero_dfs/frame.hpp"
#include "hetero_dfs/modal_metadata.hpp"

#include <modal_pipe.h>

#include <cerrno>
#include <cinttypes>
#include <climits>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <limits>
#include <mutex>
#include <unistd.h>

namespace {

const int kTrackingChannel = 0;
const int kHiresChannel = 1;
const char* const kProcessName = "voxl-hetero-dfs-server";

struct TimedFrame {
    hetero_dfs::FrameDescriptor frame;
    std::int64_t center_ns;
};

struct RunningStats {
    std::uint64_t pairs = 0;
    std::uint64_t rejected_hires = 0;
    std::uint64_t waiting_hires = 0;
    std::uint64_t stale_tracking = 0;
    std::uint64_t overflow_tracking = 0;
    std::uint64_t overflow_hires = 0;
    std::int64_t min_signed_delta_ns = 0;
    std::int64_t max_signed_delta_ns = 0;
    std::int64_t max_absolute_delta_ns = 0;
    long double signed_delta_sum_ns = 0.0L;
    long double absolute_delta_sum_ns = 0.0L;

    void add_pair(std::int64_t signed_delta_ns,
                  std::int64_t absolute_delta_ns)
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
        signed_delta_sum_ns += signed_delta_ns;
        absolute_delta_sum_ns += absolute_delta_ns;
        ++pairs;
    }
};

struct Context {
    Context(std::int64_t delta_gate_ns,
            std::size_t queue_limit,
            const hetero_dfs::FrameRequirements& tracking,
            const hetero_dfs::FrameRequirements& hires)
        : max_delta_ns(delta_gate_ns),
          max_queue_depth(queue_limit),
          tracking_requirements(tracking),
          hires_requirements(hires)
    {
    }

    std::int64_t max_delta_ns;
    std::size_t max_queue_depth;
    hetero_dfs::FrameRequirements tracking_requirements;
    hetero_dfs::FrameRequirements hires_requirements;
    std::deque<TimedFrame> tracking_queue;
    std::deque<TimedFrame> hires_queue;
    RunningStats stats;
    std::uint64_t invalid[2] = {};
    std::mutex mutex;
};

void usage()
{
    std::fprintf(stderr,
        "Usage: %s TRACKING_PIPE HIRES_PIPE TRACKING_WIDTH TRACKING_HEIGHT "
        "HIRES_WIDTH HIRES_HEIGHT MAX_DELTA_NS QUEUE_DEPTH\n",
        kProcessName);
}

bool parse(const char* text, std::uint64_t* output)
{
    if (text == NULL || text[0] == '-' || text[0] == '\0') return false;
    errno = 0;
    char* end = NULL;
    const unsigned long long value = std::strtoull(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0') return false;
    *output = static_cast<std::uint64_t>(value);
    return true;
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

void prune_old_tracking(Context* context, std::int64_t hires_center_ns)
{
    const std::int64_t oldest_useful = hires_center_ns - context->max_delta_ns;
    while (!context->tracking_queue.empty() &&
           context->tracking_queue.front().center_ns < oldest_useful) {
        context->tracking_queue.pop_front();
        ++context->stats.stale_tracking;
    }
}

void log_pair(const TimedFrame& tracking, const TimedFrame& hires,
              std::int64_t signed_delta_ns,
              std::int64_t absolute_delta_ns)
{
    std::fprintf(stderr,
        "pair tracking_frame=%" PRId32 " hires_frame=%" PRId32
        " signed_delta_ns=%" PRId64 " absolute_delta_ns=%" PRId64 "\n",
        tracking.frame.frame_id, hires.frame.frame_id,
        signed_delta_ns, absolute_delta_ns);
}

void log_summary(const Context& context)
{
    const RunningStats& stats = context.stats;
    const long double mean_signed = stats.pairs == 0U ? 0.0L :
        stats.signed_delta_sum_ns / static_cast<long double>(stats.pairs);
    const long double mean_absolute = stats.pairs == 0U ? 0.0L :
        stats.absolute_delta_sum_ns / static_cast<long double>(stats.pairs);

    std::fprintf(stderr,
        "summary pairs=%" PRIu64 " rejected_hires=%" PRIu64
        " waiting_hires=%" PRIu64
        " invalid_tracking=%" PRIu64 " invalid_hires=%" PRIu64
        " stale_tracking=%" PRIu64
        " overflow_tracking=%" PRIu64 " overflow_hires=%" PRIu64
        " mean_signed_ns=%.0Lf mean_absolute_ns=%.0Lf"
        " min_signed_ns=%" PRId64 " max_signed_ns=%" PRId64
        " max_absolute_ns=%" PRId64 "\n",
        stats.pairs, stats.rejected_hires, context.hires_queue.size(),
        context.invalid[0], context.invalid[1], stats.stale_tracking,
        stats.overflow_tracking, stats.overflow_hires,
        mean_signed, mean_absolute, stats.min_signed_delta_ns,
        stats.max_signed_delta_ns, stats.max_absolute_delta_ns);
}

void process_hires_queue(Context* context)
{
    while (!context->hires_queue.empty() &&
           !context->tracking_queue.empty()) {
        const TimedFrame& hires = context->hires_queue.front();

        if (context->tracking_queue.back().center_ns <
            hires.center_ns + context->max_delta_ns) {
            ++context->stats.waiting_hires;
            return;
        }

        std::size_t best_index = 0U;
        std::int64_t best_absolute_delta =
            std::numeric_limits<std::int64_t>::max();
        for (std::size_t index = 0U;
             index < context->tracking_queue.size(); ++index) {
            const std::int64_t signed_delta =
                context->tracking_queue[index].center_ns - hires.center_ns;
            const std::int64_t current_absolute_delta =
                absolute_delta(signed_delta);
            if (current_absolute_delta < best_absolute_delta) {
                best_index = index;
                best_absolute_delta = current_absolute_delta;
            }
        }

        if (best_absolute_delta <= context->max_delta_ns) {
            const TimedFrame tracking = context->tracking_queue[best_index];
            const TimedFrame accepted_hires = hires;
            const std::int64_t signed_delta =
                tracking.center_ns - accepted_hires.center_ns;
            context->stats.add_pair(signed_delta, best_absolute_delta);
            log_pair(tracking, accepted_hires, signed_delta,
                     best_absolute_delta);
            context->stats.stale_tracking += best_index;
            context->tracking_queue.erase(
                context->tracking_queue.begin(),
                context->tracking_queue.begin() + best_index + 1);
            context->hires_queue.pop_front();
        } else {
            ++context->stats.rejected_hires;
            prune_old_tracking(context, hires.center_ns);
            context->hires_queue.pop_front();
        }
    }
}

void push_tracking(Context* context, const TimedFrame& frame)
{
    context->tracking_queue.push_back(frame);
    while (context->tracking_queue.size() > context->max_queue_depth) {
        context->tracking_queue.pop_front();
        ++context->stats.overflow_tracking;
    }
    process_hires_queue(context);
}

void push_hires(Context* context, const TimedFrame& frame)
{
    context->hires_queue.push_back(frame);
    while (context->hires_queue.size() > context->max_queue_depth) {
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
        context->tracking_requirements : context->hires_requirements;
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

    if (camera_index == 0) {
        push_tracking(context, TimedFrame{frame, frame_center_ns});
    } else {
        push_hires(context, TimedFrame{frame, frame_center_ns});
    }
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 9) {
        usage();
        return EXIT_FAILURE;
    }

    std::uint64_t values[6] = {};
    for (int index = 0; index < 6; ++index) {
        if (!parse(argv[index + 3], &values[index])) {
            usage();
            return EXIT_FAILURE;
        }
    }
    if (values[0] == 0U || values[0] > UINT32_MAX ||
        values[1] == 0U || values[1] > UINT32_MAX ||
        values[2] == 0U || values[2] > UINT32_MAX ||
        values[3] == 0U || values[3] > UINT32_MAX ||
        values[4] > static_cast<std::uint64_t>(INT64_MAX) ||
        values[5] == 0U || values[5] > SIZE_MAX) {
        usage();
        return EXIT_FAILURE;
    }

    const hetero_dfs::FrameRequirements tracking{
        hetero_dfs::PixelFormat::kRaw8,
        static_cast<std::uint32_t>(values[0]),
        static_cast<std::uint32_t>(values[1])};
    const hetero_dfs::FrameRequirements hires{
        hetero_dfs::PixelFormat::kRaw8,
        static_cast<std::uint32_t>(values[2]),
        static_cast<std::uint32_t>(values[3])};
    Context context(
        static_cast<std::int64_t>(values[4]),
        static_cast<std::size_t>(values[5]),
        tracking, hires);

    if (enable_signal_handler() != 0) return EXIT_FAILURE;
    pipe_client_set_camera_helper_cb(kTrackingChannel, callback, &context);
    pipe_client_set_camera_helper_cb(kHiresChannel, callback, &context);

    main_running = 1;
    const int flags = CLIENT_FLAG_EN_CAMERA_HELPER;
    if (pipe_client_open(kTrackingChannel, argv[1],
                         kProcessName, flags, 0) < 0 ||
        pipe_client_open(kHiresChannel, argv[2],
                         kProcessName, flags, 0) < 0) {
        main_running = 0;
    }

    while (main_running) usleep(100000);
    pipe_client_close_all();
    log_summary(context);
    return context.stats.pairs > 0U ? EXIT_SUCCESS : EXIT_FAILURE;
}
