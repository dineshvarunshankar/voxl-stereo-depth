#!/bin/bash
set -euo pipefail

use_timestamp=false

print_usage()
{
    echo "Usage: $0 [timestamp]" >&2
}

for arg in "$@"; do
    case "$(echo "${arg}" | tr '[:upper:]' '[:lower:]')" in
        timestamp|--timestamp|-t)
            use_timestamp=true
            ;;
        help|--help|-h)
            print_usage
            exit 0
            ;;
        "") ;;
        *)
            print_usage
            exit 1
            ;;
    esac
done

if [[ ! -d build-qrb5165 ]]; then
    echo "Missing build-qrb5165/. Run ./build.sh qrb5165 first." >&2
    exit 1
fi

if [[ ! -x build-qrb5165/src/voxl-stereo-timing ]]; then
    echo "Missing build-qrb5165/src/voxl-stereo-timing." >&2
    exit 1
fi

if [[ ! -x build-qrb5165/src/voxl-hetero-dfs-server ]]; then
    echo "Missing build-qrb5165/src/voxl-hetero-dfs-server." >&2
    exit 1
fi

control_file="pkg/control/control"
if [[ ! -f "${control_file}" ]]; then
    echo "Missing ${control_file}." >&2
    exit 1
fi

package=$(awk '/^Package:/ {print $2}' "${control_file}")
version=$(awk '/^Version:/ {print $2}' "${control_file}")

if [[ -z "${package}" || -z "${version}" ]]; then
    echo "Package control file must contain Package and Version." >&2
    exit 1
fi

data_dir="pkg/data"
deb_dir="pkg/DEB"

rm -rf "${data_dir}" "${deb_dir}"
rm -f ./*.deb

DESTDIR="${PWD}/${data_dir}" \
    cmake --install build-qrb5165 --prefix /usr --strip --config Release

mkdir -p "${deb_dir}/DEBIAN"
cp -R pkg/control/. "${deb_dir}/DEBIAN/"
cp -R "${data_dir}"/. "${deb_dir}"/

if [[ "${use_timestamp}" == true ]]; then
    timestamp=$(date +"%Y%m%d%H%M")
    version="${version}-${timestamp}"
    sed -i -E "s/^Version:.*/Version: ${version}/" "${deb_dir}/DEBIAN/control"
fi

deb_name="${package}_${version}_arm64.deb"
dpkg-deb --build "${deb_dir}" "${deb_name}"

echo "Created ${deb_name}"
