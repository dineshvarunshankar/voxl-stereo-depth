# Field Runbook

Use this file for VOXL 2 field diagnostics.

## Timing Probe

Run this command with the active camera sizes:

```bash
voxl-stereo-timing /run/mpa/tracking_front /run/mpa/hires_small_grey \
    1280 800 1024 768 12000000 60 900 > /data/stereo_timing.csv
```

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

## Service Skeleton

Run this command after the timing probe is acceptable:

```bash
timeout 60s voxl-hetero-dfs-server \
    /run/mpa/tracking_front /run/mpa/hires_small_grey \
    1280 800 1024 768 5000000 60 \
    2> /data/hetero_dfs.log
```

The service skeleton validates frames and pairs them by exposure-center
timestamp. It does not publish a depth image yet.

Read the recent diagnostics:

```bash
tail -n 20 /data/hetero_dfs.log
```

Read the summary line:

```bash
grep summary /data/hetero_dfs.log | tail -n 5
```

The summary line reports accepted `pairs`, `rejected_hires`, queued
`waiting_hires`, and timing statistics for the accepted pairs.

## Bring-Up Gate

1. Configure both cameras to the target FPS.
2. Run the timing probe.
3. Run the service skeleton.
4. Capture the shared stereo calibration dataset.
5. Generate and inspect heterogeneous rectification maps.
6. Add the dense disparity and depth service.
7. Test the 0.5 m to 5 m range outdoors at 2 m/s.
