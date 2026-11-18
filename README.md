# VOXL Heterogeneous DFS

Stereo depth for VOXL 2 using the Starling 2 `tracking_front` and
`hires_small_grey` cameras. The C++ service rectifies images, computes SGBM
disparity, and publishes metric depth.

## Build

In a `voxl-cross` 4.4 or newer container:

```bash
./install_build_deps.sh qrb5165 dev
./build.sh qrb5165
./make_package.sh
```

Replace `dev` with the package repository section for the target VOXL image.
The package is `voxl-hetero-dfs_<version>_arm64.deb`.

## Deploy

Over ADB:

```bash
./deploy_to_voxl.sh adb
```

Or SSH:

```bash
./deploy_to_voxl.sh ssh 192.168.1.123
```

## Run

On VOXL 2, check the camera dimensions and calibration paths in
`/etc/modalai/voxl-hetero-dfs-server.conf`.

```bash
systemctl enable voxl-hetero-dfs-server
systemctl start voxl-hetero-dfs-server
systemctl status voxl-hetero-dfs-server
```

The output pipe is `/run/mpa/hetero_depth`: `IMAGE_FORMAT_FLOAT32`, depth in
meters, with `NaN` for invalid pixels.

See [RUNBOOK.md](RUNBOOK.md) for timing measurements and service diagnostics.
