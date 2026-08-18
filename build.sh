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
    qrb5165)
        check_cross_image
        cmake -S . -B build-qrb5165 \
            -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_QRB5165}" \
            -DCMAKE_BUILD_TYPE=Release
        cmake --build build-qrb5165 --parallel "${JOBS:-$(nproc)}"
        if [[ ! -x build-qrb5165/src/voxl-stereo-timing ]]; then
            echo "Missing build-qrb5165/src/voxl-stereo-timing" >&2
            exit 1
        fi
        if [[ ! -x build-qrb5165/src/voxl-hetero-dfs-server ]]; then
            echo "Missing build-qrb5165/src/voxl-hetero-dfs-server" >&2
            exit 1
        fi
        ;;
    *)
        echo "Usage: $0 qrb5165" >&2
        exit 1
        ;;
esac
