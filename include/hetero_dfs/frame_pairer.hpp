#ifndef HETERO_DFS_FRAME_PAIRER_HPP
#define HETERO_DFS_FRAME_PAIRER_HPP

#include "hetero_dfs/frame.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>

namespace hetero_dfs {

enum class Camera {
    kTracking,
    kHires
};

struct FramePairerConfig {
    std::int64_t max_delta_ns;
    std::size_t max_queue_depth;
};

struct FramePair {
    FrameDescriptor tracking;
    FrameDescriptor hires;
    std::int64_t delta_ns = 0;
    std::int64_t timestamp_ns = 0;
};

struct FramePairerCounters {
    std::uint64_t paired = 0;
    std::uint64_t stale_tracking = 0;
    std::uint64_t stale_hires = 0;
    std::uint64_t overflow_tracking = 0;
    std::uint64_t overflow_hires = 0;
    std::uint64_t invalid_timestamp = 0;
};

class FramePairer {
public:
    explicit FramePairer(const FramePairerConfig& config);

    bool push(Camera camera, const FrameDescriptor& frame);
    bool pop(FramePair* pair);

    std::size_t tracking_queue_size() const;
    std::size_t hires_queue_size() const;
    const FramePairerCounters& counters() const;

private:
    struct TimedFrame {
        FrameDescriptor frame;
        std::int64_t center_ns;
    };

    typedef std::deque<TimedFrame> Queue;

    void insert(Camera camera, const TimedFrame& frame);
    static bool exposure_center(const FrameDescriptor& frame,
                                std::int64_t* center_ns);

    FramePairerConfig config_;
    Queue tracking_;
    Queue hires_;
    FramePairerCounters counters_;
};

}  // namespace hetero_dfs

#endif  // HETERO_DFS_FRAME_PAIRER_HPP
