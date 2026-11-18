# Field Runbook

## Timing Probe

On VOXL 2, use the active camera dimensions:

```bash
voxl-stereo-timing /run/mpa/tracking_front /run/mpa/hires_small_grey \
    1280 800 1024 768 12000000 60 900 > /data/stereo_timing.csv
```

Arguments after the pipe paths: tracking width and height, hires width and height,
pairing tolerance (ns), queue depth, and sample count. Queue depth must be at least `2`.

CSV columns:

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

## Depth Service

Configuration:

```text
/etc/modalai/voxl-hetero-dfs-server.conf
```

Check the camera calibration paths before starting:

```bash
systemctl enable voxl-hetero-dfs-server
systemctl start voxl-hetero-dfs-server
```

Restart after config changes:

```bash
systemctl restart voxl-hetero-dfs-server
```

Inspect service logs:

```bash
journalctl -u voxl-hetero-dfs-server -n 80 --no-pager
```

Confirm the output pipe exists:

```bash
voxl-inspect-pipe hetero_depth
```

The output is `IMAGE_FORMAT_FLOAT32`, meters per pixel. Invalid pixels are NaN.
