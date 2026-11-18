#include "hetero_dfs/frame_pairer.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace hetero_dfs {
namespace {

std::int64_t absolute_delta(std::int64_t first, std::int64_t second)
{
    return first >= second ? first - second : second - first;
}

}  // namespace

FramePairer::FramePairer(const FramePairerConfig& config)
    : config_(config)
{
    if (config_.max_delta_ns < 0) {
        throw std::invalid_argument("max_delta_ns must be nonnegative");
    }
    if (config_.max_queue_depth < 2U) {
        throw std::invalid_argument("max_queue_depth must be at least 2");
    }
}

bool FramePairer::exposure_center(const FrameDescriptor& frame,
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

void FramePairer::insert(Camera camera, const TimedFrame& timed_frame)
{
    Queue* queue = camera == Camera::kTracking ? &tracking_ : &hires_;
    const Queue::iterator position = std::upper_bound(
        queue->begin(), queue->end(), timed_frame.center_ns,
        [](std::int64_t center_ns, const TimedFrame& queued) {
            return center_ns < queued.center_ns;
        });
    queue->insert(position, timed_frame);

    if (queue->size() > config_.max_queue_depth) {
        queue->pop_front();
        if (camera == Camera::kTracking) {
            ++counters_.overflow_tracking;
        } else {
            ++counters_.overflow_hires;
        }
    }
}

bool FramePairer::push(Camera camera, const FrameDescriptor& frame)
{
    std::int64_t center_ns = 0;
    if (!exposure_center(frame, &center_ns)) {
        ++counters_.invalid_timestamp;
        return false;
    }
    insert(camera, TimedFrame{frame, center_ns});
    return true;
}

bool FramePairer::pop(FramePair* pair)
{
    if (pair == NULL) {
        return false;
    }

    while (!tracking_.empty() && !hires_.empty()) {
        const TimedFrame& hires = hires_.front();
        const std::int64_t target_center =
            hires.center_ns + config_.time_offset_ns;

        if (tracking_.back().center_ns <
            target_center + config_.max_delta_ns) {
            return false;
        }

        std::size_t best_tracking = 0U;
        std::int64_t best_delta = std::numeric_limits<std::int64_t>::max();

        for (std::size_t tracking_index = 0U;
             tracking_index < tracking_.size(); ++tracking_index) {
            const std::int64_t delta = absolute_delta(
                tracking_[tracking_index].center_ns, target_center);
            if (delta < best_delta) {
                best_delta = delta;
                best_tracking = tracking_index;
            }
        }

        if (best_delta <= config_.max_delta_ns) {
            counters_.stale_tracking += best_tracking;
            tracking_.erase(tracking_.begin(),
                            tracking_.begin() + best_tracking);

            const TimedFrame tracking = tracking_.front();
            const TimedFrame accepted_hires = hires;
            tracking_.pop_front();
            hires_.pop_front();

            pair->tracking = tracking.frame;
            pair->hires = accepted_hires.frame;
            pair->delta_ns = absolute_delta(tracking.center_ns,
                                            accepted_hires.center_ns);
            const std::int64_t earlier =
                std::min(tracking.center_ns, accepted_hires.center_ns);
            const std::int64_t later =
                std::max(tracking.center_ns, accepted_hires.center_ns);
            pair->timestamp_ns = earlier + (later - earlier) / 2;
            ++counters_.paired;
            return true;
        }

        const std::int64_t oldest_useful =
            target_center - config_.max_delta_ns;
        while (!tracking_.empty() &&
               tracking_.front().center_ns < oldest_useful) {
            tracking_.pop_front();
            ++counters_.stale_tracking;
        }
        hires_.pop_front();
        ++counters_.stale_hires;
    }
    return false;
}

std::size_t FramePairer::tracking_queue_size() const
{
    return tracking_.size();
}

std::size_t FramePairer::hires_queue_size() const
{
    return hires_.size();
}

const FramePairerCounters& FramePairer::counters() const
{
    return counters_;
}

}  // namespace hetero_dfs
