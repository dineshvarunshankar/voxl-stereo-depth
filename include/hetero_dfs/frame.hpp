#ifndef HETERO_DFS_FRAME_HPP
#define HETERO_DFS_FRAME_HPP

#include <cstddef>
#include <cstdint>

namespace hetero_dfs {

enum class PixelFormat : std::int16_t {
    kRaw8 = 0,
    kNv12 = 1
};

struct FrameRequirements {
    PixelFormat format;
    std::uint32_t width;
    std::uint32_t height;
};

struct FrameDescriptor {
    PixelFormat format = PixelFormat::kRaw8;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::size_t stride = 0;
    std::size_t payload_size = 0;
    std::int64_t timestamp_ns = 0;
    std::int64_t exposure_ns = 0;
    std::int32_t frame_id = 0;
};

enum class FrameValidation {
    kValid,
    kWrongFormat,
    kWrongDimensions,
    kInvalidStride,
    kSizeOverflow,
    kPayloadTooSmall,
    kInvalidExposure
};

FrameValidation validate_frame(const FrameDescriptor& frame,
                               const FrameRequirements& requirements);

}  // namespace hetero_dfs

#endif  // HETERO_DFS_FRAME_HPP
