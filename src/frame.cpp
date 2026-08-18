#include "hetero_dfs/frame.hpp"

#include <limits>

namespace hetero_dfs {

FrameValidation validate_frame(const FrameDescriptor& frame,
                               const FrameRequirements& requirements)
{
    if (frame.format != requirements.format) {
        return FrameValidation::kWrongFormat;
    }
    if (frame.width != requirements.width ||
        frame.height != requirements.height) {
        return FrameValidation::kWrongDimensions;
    }
    if (frame.stride < frame.width) {
        return FrameValidation::kInvalidStride;
    }
    if (frame.height != 0U &&
        frame.stride > std::numeric_limits<std::size_t>::max() / frame.height) {
        return FrameValidation::kSizeOverflow;
    }
    const std::size_t required_size = frame.stride * frame.height;
    if (frame.payload_size < required_size) {
        return FrameValidation::kPayloadTooSmall;
    }
    if (frame.exposure_ns < 0) {
        return FrameValidation::kInvalidExposure;
    }
    return FrameValidation::kValid;
}

}  // namespace hetero_dfs
