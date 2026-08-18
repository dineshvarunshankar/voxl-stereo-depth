#!/bin/bash
set -euo pipefail

TOOLCHAIN_QRB5165="/opt/cross_toolchain/qrb5165_ubun1_18.04_aarch64.toolchain.cmake"

check_cross_image()
{
    local image_file="/etc/modalai/image.name"
    if [[ ! -f "${image_file}" ]]; then
        echo "Run the qrb5165 build inside voxl-cross 4.4 or newer." >&2
        exit 1
    fi
    local image_string
    image_string=$(<"${image_file}")
    if [[ ! "${image_string}" =~ ^voxl-cross\(([0-9]+\.[0-9]+)\)$ ]]; then
        echo "Invalid voxl-cross image marker: ${image_string}" >&2
        exit 1
    fi
    if [[ "$(printf '%s\n' "4.4" "${BASH_REMATCH[1]}" | sort -V | head -n1)" != "4.4" ]]; then
        echo "voxl-cross 4.4 or newer is required." >&2
        exit 1
    fi
}

case "${1:-}" in
    native)
        cmake -S . -B build \
            -DCMAKE_BUILD_TYPE=Debug \
            -DHETERO_DFS_BUILD_VOXL=OFF
        cmake --build build --parallel "${JOBS:-$(nproc)}"
        ;;
    qrb5165)
        check_cross_image
        cmake -S . -B build-qrb5165 \
            -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_QRB5165}" \
            -DCMAKE_BUILD_TYPE=Release \
            -DHETERO_DFS_BUILD_VOXL=ON
        cmake --build build-qrb5165 --parallel "${JOBS:-$(nproc)}"
        ;;
    *)
        echo "Usage: $0 {native|qrb5165}" >&2
        exit 1
        ;;
esac
