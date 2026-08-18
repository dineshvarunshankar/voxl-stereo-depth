#ifndef HETERO_DFS_MODAL_METADATA_HPP
#define HETERO_DFS_MODAL_METADATA_HPP

#include "hetero_dfs/frame.hpp"

#include <cstddef>
#include <cstdint>

namespace hetero_dfs {
namespace detail {

template <typename Integer>
std::size_t nonnegative_size(Integer value)
{
    return value > 0 ? static_cast<std::size_t>(value) : 0U;
}

template <typename Integer>
std::uint32_t nonnegative_dimension(Integer value)
{
    return value > 0 ? static_cast<std::uint32_t>(value) : 0U;
}

}  // namespace detail

template <typename CameraMetadata>
FrameDescriptor frame_from_modal_metadata(const CameraMetadata& metadata)
{
    FrameDescriptor frame;
    frame.format = static_cast<PixelFormat>(
        static_cast<std::int16_t>(metadata.format));
    frame.width = detail::nonnegative_dimension(metadata.width);
    frame.height = detail::nonnegative_dimension(metadata.height);
    frame.stride = detail::nonnegative_size(metadata.stride);
    frame.payload_size = detail::nonnegative_size(metadata.size_bytes);
    frame.timestamp_ns = static_cast<std::int64_t>(metadata.timestamp_ns);
    frame.exposure_ns = static_cast<std::int64_t>(metadata.exposure_ns);
    frame.frame_id = static_cast<std::int32_t>(metadata.frame_id);
    return frame;
}

}  // namespace hetero_dfs

#endif  // HETERO_DFS_MODAL_METADATA_HPP
