#!/bin/bash
set -euo pipefail

DEPS="
libmodal-json
libmodal-pipe
voxl-opencv
"

print_usage()
{
    echo "Usage: $0 {platform} {section}" >&2
    echo "Example: $0 qrb5165 dev" >&2
}

if [[ "$#" -ne 2 ]]; then
    print_usage
    exit 1
fi

platform=$(echo "$1" | tr '[:upper:]' '[:lower:]')
section=$(echo "$2" | tr '[:upper:]' '[:lower:]')

if [[ ! -f /etc/modalai/image.name ]]; then
    echo "Run this script inside the voxl-cross container." >&2
    exit 1
fi

case "${platform}" in
    qrb5165)
        ;;
    *)
        echo "Unsupported platform: ${platform}" >&2
        print_usage
        exit 1
        ;;
esac

source_file="/etc/apt/sources.list.d/modalai.list"
source_line="deb [trusted=yes] http://voxl-packages.modalai.com/ ./dists/${platform}/${section}/binary-arm64/"

echo "Using ModalAI repo: ${platform}/${section}"
echo "${source_line}" | sudo tee "${source_file}" >/dev/null

sudo apt-get update \
    -o Dir::Etc::sourcelist="sources.list.d/modalai.list" \
    -o Dir::Etc::sourceparts="-" \
    -o APT::Get::List-Cleanup="0"

sudo apt-get install -y ${DEPS}
