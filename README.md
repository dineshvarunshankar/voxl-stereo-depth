# VOXL Heterogeneous DFS

This repo contains C/C++ deployment code for heterogeneous stereo depth on
VOXL 2. The first target is Starling 2 with `tracking_front` and
`hires_small_grey`.

The runtime does not use VINS, ZipDepth, or a neural model.

## Install Build Dependencies

Run this command inside the VOXL cross-build container:

```bash
./install_build_deps.sh qrb5165 dev
```

Use the repo section that matches the target VOXL image.

## VOXL 2 Build

Use a VOXL cross-build container with `voxl-cross` 4.4 or newer.

Run this command:

```bash
./build.sh qrb5165
```

This command builds `build-qrb5165/src/voxl-stereo-timing` and
`build-qrb5165/src/voxl-hetero-dfs-server`.

Make sure that the binaries exist:

```bash
test -x build-qrb5165/src/voxl-stereo-timing
test -x build-qrb5165/src/voxl-hetero-dfs-server
```

## Package

Run this command after the VOXL 2 build:

```bash
./make_package.sh
```

The command creates `voxl-hetero-dfs_<version>_arm64.deb`.

## Deploy

Deploy over ADB:

```bash
./deploy_to_voxl.sh adb
```

Deploy over SSH:

```bash
./deploy_to_voxl.sh ssh 192.168.1.123
```

## Installed Binaries

- `voxl-stereo-timing`
- `voxl-hetero-dfs-server`

## Quick Run

```bash
voxl-stereo-timing /run/mpa/tracking_front /run/mpa/hires_small_grey \
    1280 800 1024 768 12000000 60 900 > /data/stereo_timing.csv
```

```bash
timeout 60s voxl-hetero-dfs-server /run/mpa/tracking_front /run/mpa/hires_small_grey \
    1280 800 1024 768 5000000 60 2> /data/hetero_dfs.log
```

The service skeleton pairs frames. It does not publish a depth image yet.

## Script Summary

- `install_build_deps.sh` installs cross-build packages in `voxl-cross`.
- `build.sh qrb5165` compiles VOXL 2 targets in `voxl-cross`.
- `make_package.sh` creates the `.deb` package.
- `deploy_to_voxl.sh` installs the `.deb` package on VOXL 2.
- `voxl-hetero-dfs-server` is the timestamp-pairing service skeleton.

See `RUNBOOK.md` for field diagnostics.
