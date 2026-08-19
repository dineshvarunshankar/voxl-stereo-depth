# VOXL Heterogeneous DFS

This repo contains C/C++ deployment code for lightweight heterogeneous stereo
depth on VOXL 2. The first target is Starling 2 with `tracking_front` and
`hires_small_grey`, publishing one FLOAT32 metric depth pipe.

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

## Installed Files

- `voxl-stereo-timing`
- `voxl-hetero-dfs-server`
- `/etc/modalai/voxl-hetero-dfs-server.conf`
- `/etc/systemd/system/voxl-hetero-dfs-server.service`
- `/usr/share/modalai/voxl-hetero-dfs-server/*.yml`

## Runtime

The package installs `voxl-hetero-dfs-server` and its systemd unit. The service
reads `/etc/modalai/voxl-hetero-dfs-server.conf` and starts without CLI
arguments.

Enable and start it after checking the calibration paths in the config match the
target:

```bash
systemctl enable voxl-hetero-dfs-server
systemctl start voxl-hetero-dfs-server
```

```bash
systemctl status voxl-hetero-dfs-server
```

Output pipe:

```text
/run/mpa/hetero_depth
```

Format:

```text
IMAGE_FORMAT_FLOAT32, meters per pixel, NaN for invalid depth
```

## Script Summary

- `install_build_deps.sh` installs cross-build packages in `voxl-cross`.
- `build.sh qrb5165` compiles VOXL 2 targets in `voxl-cross`.
- `make_package.sh` creates the `.deb` package.
- `deploy_to_voxl.sh` installs the `.deb` package on VOXL 2.
- `voxl-hetero-dfs-server` publishes metric stereo depth from one configured camera pair.

See `RUNBOOK.md` for field diagnostics.
