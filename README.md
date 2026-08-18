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

## Timing Probe

Use `voxl-stereo-timing` on VOXL 2 before dense stereo flight tests.

Example command:

```bash
voxl-stereo-timing /run/mpa/tracking_front /run/mpa/hires_small_grey \
    1280 800 1024 768 10000000 30 300 > /data/stereo_timing.csv
```

Set the widths and heights to the active camera configuration.

The probe reads two image pipes. It writes exposure-center timestamp pairs to
CSV.

The CSV columns are:

```text
tracking_frame,hires_frame,tracking_center_ns,hires_center_ns,signed_delta_ns,absolute_delta_ns
```

Summarize the timing file:

```bash
awk -F, 'NR>1 {
  n++; signed_sum+=$5; abs_sum+=$6
  if(abs_min==""||$6<abs_min)abs_min=$6
  if($6>abs_max)abs_max=$6
}
END {
  print "pairs", n, "mean_signed_ns", signed_sum/n,
        "mean_abs_ns", abs_sum/n, "min_abs_ns", abs_min,
        "max_abs_ns", abs_max
}' /data/stereo_timing.csv
```

Inspect the frame-step pattern:

```bash
awk -F, 'NR>2 {print $1-p1, $2-p2} {p1=$1; p2=$2}' \
    /data/stereo_timing.csv | sort | uniq -c
```

Inspect signed jitter:

```bash
awk -F, 'NR>1 {d[NR-1]=$5; n++}
END {
  for(i=1;i<=n;i++) s+=d[i]
  mean=s/n
  for(i=1;i<=n;i++) {
    e=d[i]-mean; if(e<0)e=-e; if(e>max_jitter)max_jitter=e
  }
  print "mean_signed_ns", mean, "max_signed_jitter_ns", max_jitter
}' /data/stereo_timing.csv
```

## Stereo Service Bring-Up

Run the service skeleton after the timing probe is acceptable:

```bash
voxl-hetero-dfs-server /run/mpa/tracking_front /run/mpa/hires_small_grey \
    1280 800 1024 768 5000000 60 2> /data/hetero_dfs.log
```

This first service stage validates frames and pairs them by exposure-center
timestamp. It does not publish a depth image yet.

Use `5000000` ns as the initial absolute-delta gate for 15 FPS bring-up.

Read the diagnostics:

```bash
tail -n 20 /data/hetero_dfs.log
```

## Bring-Up

1. Build `voxl-stereo-timing` in the VOXL cross-build container.
2. Package the binary.
3. Install the package on VOXL 2.
4. Start both camera pipes.
5. Run `voxl-stereo-timing`.
6. Save the CSV file.
7. Inspect `signed_delta_ns`, `absolute_delta_ns`, and the stderr counters.
8. Run `voxl-hetero-dfs-server` and inspect the pairing diagnostics.

## Flight Test Gate

1. Configure both cameras to the same supported FPS.
2. Run the timing probe on VOXL 2.
3. Capture a shared stereo calibration dataset.
4. Generate and inspect heterogeneous rectification maps.
5. Add the dense disparity and depth service.
6. Test the 0.5 m to 5 m range outdoors at 2 m/s.

## Calibration

Use mechanical extrinsics only as an initial estimate. The stereo path needs a
real shared-board calibration before flight tests.

## Script Summary

- `install_build_deps.sh` installs cross-build packages in `voxl-cross`.
- `build.sh qrb5165` compiles VOXL 2 targets in `voxl-cross`.
- `make_package.sh` creates the `.deb` package.
- `deploy_to_voxl.sh` installs the `.deb` package on VOXL 2.
- `voxl-hetero-dfs-server` is the timestamp-pairing service skeleton.
