# VOXL Heterogeneous DFS

This repo contains C/C++ deployment code for heterogeneous stereo depth on
VOXL 2. The first target is Starling 2 with `tracking_front` and
`hires_small_grey`.

The runtime does not use VINS, ZipDepth, or a neural model.

## Desktop Build

Install OpenCV development headers first.

Run this command:

```bash
./build.sh native
```

The command configures `build/` and compiles the host targets.

## VOXL 2 Build

Use a VOXL cross-build container with `voxl-cross` 4.4 or newer.

Run this command:

```bash
./build.sh qrb5165
```

This command builds `voxl-stereo-timing`. It needs the ModalAI SDK headers and
libraries in the cross-build environment.

## Timing Probe

Use `voxl-stereo-timing` on VOXL 2 before dense stereo flight tests.

The probe reads two image pipes. It writes exposure-center timestamp pairs to
CSV.

The CSV columns are:

```text
tracking_frame,hires_frame,tracking_center_ns,hires_center_ns,absolute_delta_ns
```

## Bring-Up

1. Configure both cameras to the same supported FPS.
2. Run the timing probe on VOXL 2.
3. Capture a shared stereo calibration dataset.
4. Generate and inspect heterogeneous rectification maps.
5. Add the dense disparity and depth service.
6. Test the 0.5 m to 5 m range outdoors at 2 m/s.

## Calibration

Use mechanical extrinsics only as an initial estimate. The stereo path needs a
real shared-board calibration before flight tests.
