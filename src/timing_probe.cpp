#include "hetero_dfs/frame.hpp"
#include "hetero_dfs/frame_pairer.hpp"
#include "hetero_dfs/modal_metadata.hpp"

#include <modal_pipe.h>

#include <cinttypes>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <unistd.h>

namespace {

const int kTrackingChannel = 0;
const int kHiresChannel = 1;
const char* const kProcessName = "voxl-stereo-timing";

struct Context {
    Context(const hetero_dfs::FramePairerConfig& pairer_config,
            const hetero_dfs::FrameRequirements& tracking,
            const hetero_dfs::FrameRequirements& hires,
            std::uint64_t limit)
        : pairer(pairer_config),
          tracking_requirements(tracking),
          hires_requirements(hires),
          sample_limit(limit)
    {
    }

    hetero_dfs::FramePairer pairer;
    hetero_dfs::FrameRequirements tracking_requirements;
    hetero_dfs::FrameRequirements hires_requirements;
    std::uint64_t sample_limit;
    std::uint64_t samples = 0;
    std::uint64_t invalid[2] = {};
    std::mutex mutex;
};

void usage()
{
    std::fprintf(stderr,
        "Usage: %s TRACKING_PIPE HIRES_PIPE TRACKING_WIDTH TRACKING_HEIGHT "
        "HIRES_WIDTH HIRES_HEIGHT MAX_DELTA_NS QUEUE_DEPTH SAMPLES\n",
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

std::int64_t center(const hetero_dfs::FrameDescriptor& frame)
{
    return frame.timestamp_ns + frame.exposure_ns / 2;
}

void callback(int channel, camera_image_metadata_t metadata,
              char* data, void* opaque)
{
    (void)data;
    Context* const context = static_cast<Context*>(opaque);
    const int camera_index = channel == kTrackingChannel ? 0 : 1;
    const hetero_dfs::Camera camera = camera_index == 0 ?
        hetero_dfs::Camera::kTracking : hetero_dfs::Camera::kHires;
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
    if (!context->pairer.push(camera, frame)) return;

    hetero_dfs::FramePair pair;
    while (context->pairer.pop(&pair)) {
        std::printf("%" PRId32 ",%" PRId32 ",%" PRId64 ",%" PRId64
                    ",%" PRId64 "\n",
                    pair.tracking.frame_id, pair.hires.frame_id,
                    center(pair.tracking), center(pair.hires), pair.delta_ns);
        std::fflush(stdout);
        if (++context->samples >= context->sample_limit) {
            main_running = 0;
            return;
        }
    }
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 10) {
        usage();
        return EXIT_FAILURE;
    }

    std::uint64_t values[7] = {};
    for (int index = 0; index < 7; ++index) {
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
        values[5] == 0U || values[5] > SIZE_MAX || values[6] == 0U) {
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
        {static_cast<std::int64_t>(values[4]),
         static_cast<std::size_t>(values[5])},
        tracking, hires, values[6]);

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

    std::printf("tracking_frame,hires_frame,tracking_center_ns,"
                "hires_center_ns,absolute_delta_ns\n");
    while (main_running) usleep(100000);
    pipe_client_close_all();

    const hetero_dfs::FramePairerCounters& counters =
        context.pairer.counters();
    std::fprintf(stderr,
        "samples=%" PRIu64 " invalid_tracking=%" PRIu64
        " invalid_hires=%" PRIu64 " stale_tracking=%" PRIu64
        " stale_hires=%" PRIu64 " overflow_tracking=%" PRIu64
        " overflow_hires=%" PRIu64 "\n",
        context.samples, context.invalid[0], context.invalid[1],
        counters.stale_tracking, counters.stale_hires,
        counters.overflow_tracking, counters.overflow_hires);
    return context.samples >= context.sample_limit ?
        EXIT_SUCCESS : EXIT_FAILURE;
}
