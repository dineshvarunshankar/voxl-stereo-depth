#!/bin/bash
set -euo pipefail

print_usage()
{
    echo "Usage: $0 {adb|ssh [ip]}" >&2
    echo "Example: $0 adb" >&2
    echo "Example: $0 ssh 192.168.1.123" >&2
}

if [[ "$#" -lt 1 ]]; then
    print_usage
    exit 1
fi

mode="$1"
case "${mode}" in
    help|--help|-h)
        print_usage
        exit 0
        ;;
esac

deb_count=$(find . -maxdepth 1 -type f -name '*.deb' | wc -l)
if [[ "${deb_count}" -ne 1 ]]; then
    echo "Expected exactly one .deb file in this directory." >&2
    echo "Run ./make_package.sh first." >&2
    exit 1
fi

deb_file=$(find . -maxdepth 1 -type f -name '*.deb' | head -n 1)
deb_base=$(basename "${deb_file}")

case "${mode}" in
    adb)
        if ! command -v adb >/dev/null 2>&1; then
            echo "adb is not installed." >&2
            exit 1
        fi
        adb wait-for-device
        adb push "${deb_file}" "/data/${deb_base}"
        adb shell "dpkg -i --force-downgrade /data/${deb_base}"
        ;;
    ssh)
        if [[ "$#" -gt 2 ]]; then
            print_usage
            exit 1
        fi
        if ! command -v ssh >/dev/null 2>&1; then
            echo "ssh is not installed." >&2
            exit 1
        fi
        if ! command -v scp >/dev/null 2>&1; then
            echo "scp is not installed." >&2
            exit 1
        fi
        target_ip="${2:-${VOXL_IP:-}}"
        if [[ -z "${target_ip}" ]]; then
            echo "Set VOXL_IP or pass an IP address." >&2
            exit 1
        fi
        scp "${deb_file}" "root@${target_ip}:/data/${deb_base}"
        ssh "root@${target_ip}" "dpkg -i --force-downgrade /data/${deb_base}"
        ;;
    *)
        print_usage
        exit 1
        ;;
esac
