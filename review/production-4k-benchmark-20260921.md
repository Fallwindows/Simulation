# Production 4K sensor benchmark r2

Date: 2026-09-21

Host: Windows 11, Isaac Sim 6.1, RTX 4070 Ti 12 GB, ROS 2 Jazzy through the existing Pixi workspace

## Configuration and invocation

`config/scenarios/production_walking_4k.yaml` selects native 3840×2160 RGB at 30 Hz while reusing the baseline LiDAR configuration and complete 20.5-second walking trajectory. The 1280×720 baseline remains the preview/test profile.

Production capture is explicit and fail closed:

```powershell
.\scripts\capture_simulation.ps1 `
  -Scenario config/scenarios/production_walking_4k.yaml `
  -Realtime -Headless
```

The capture launcher limits simulation to 0.125× wall time while preserving 30 Hz simulation timestamps. Isaac publishes RGB with reliable KEEP_ALL QoS. One NumPy-free raw CDR subscription feeds independent bounded sqlite3 and FFmpeg workers, so the bag and video index receive the same source samples without duplicating each 24.9 MB message across Zenoh. Completion independently checks recorder cadence, bag cadence, target tail, and exact common-window stamp equality before creating `CAPTURE_COMPLETE`.

The benchmark used a local ignored scenario derived from production; only trajectory duration changed from 20.5 to 4.0 simulated seconds:

```powershell
.\scripts\capture_simulation.ps1 `
  -Scenario runs/benchmark-4k/production_walking_4k_short_r2.yaml `
  -Realtime -Headless
```

## Diagnosis and fail-closed evidence

The original one-second run wrote 27 recorder frames with one 66.7 ms gap and only 24 bag RGB messages. Before the final remedy, a four-second real-time run showed Isaac observing a contiguous 117-frame sequence while two external subscribers received different subsets: recorder 110, bag 111, overlap 104, union 117. Separating FFmpeg into a worker left its queue high-water mark at one and did not remove the gaps, proving encoder throughput was not the loss point. Raw CDR subscriptions removed deserialize/reserialize copies, but duplicated 4K Zenoh fan-out still lost samples. All these attempts now produce incomplete metadata and no completion sentinel.

The installed FFmpeg exposes `h264_nvenc`; it was not selected because the measured bottleneck was ingress/fan-out and the accepted libx264 `fast`, CRF 18 quality contract did not need to change.

## Passing four-second result

- Capture: `runs/20260921-033357133/capture/`
- Wall time: 106.781 seconds, including startup, Isaac runtime, queue drains, cadence validation, hashing, and manifest finalization.
- Recorder and bag each contain the same 118 RGB stamps from 0.1 through 4.0 seconds. All 117 intervals are contiguous at 30 Hz: zero missing, irregular, duplicate, out-of-order, or trailing intervals; maximum gap 33.333335 ms and maximum interval error 1.67 ns.
- FFprobe reports H.264, 3840×2160, 30/1, 118 frames, and 3.933333 seconds. The recorder used libx264 `fast`, CRF 18; its encoder queue high-water mark was 1 of 30 with zero overflow.
- Bag counts are 238 clock, 118 RGB, 39 LiDAR, 240 TF, and 1 static TF. The sqlite3 writer queue high-water mark was 2 of 30 with zero overflow or writer error.
- SQLite integrity is `ok`, journal mode is `delete`, 636 messages are present, and no WAL/SHM sidecars remain.
- Capture bytes total 3,029,114,086. The sqlite3 DB is 3,012,268,032 bytes and the MP4 is 15,185,411 bytes.
- GPU telemetry peaked at 51% utilization, 6,104 MiB of 12,282 MiB, and 131.66 W. Mean sampled values were 9.72%, 2,743.54 MiB, and 25.18 W.
- Canonical manifest validation passed with capture SHA-256 `033f175269896df09c35ad9647b513168ac7bce8bad6f1eeaa8a9784629dd7427`.

Artifact SHA-256 values:

- Manifest file: `f6f7956148898e4f96dae53078356ea39c74593911cdf28909744c8822dfc66c`
- RGB MP4: `4509b00527b10591d531b24a37acd990f5e66f9e2da2e4878c9795f3116abb59`
- Bag DB: `79d7bea1f13891e55d47f903af539467b26126d5bac5bed0a741698c882876a4`
- GPU telemetry: `fc01cc869009fe5761373edac6db0affb1af22e0d286b8a3c248903ef3507316`

## Visual inspection

Native decoded frames 0, 59, and 117 were inspected at start, midpoint, and end. All show coherent forward motion through the aisle, stable exposure and geometry, fine package edges, and no black, corrupt, torn, or partially encoded regions. Existing mirrored text on some authored package faces remains scene content. PNG SHA-256 values are `b4c5ad34…`, `9119d851…`, and `f520e1c1…`.

## Residual limits

This proves a contiguous four-second native capture on the target host. The complete 20.5-second production trajectory was not benchmarked end to end. Raw RGB alone is about 15.3 GB for that duration before sqlite3, LiDAR, TF, and metadata overhead; reserve substantially more than 16 GB. The production run remains guarded by the same cadence, queue-overflow, SQLite, common-window, manifest, and completion-sentinel checks, so a host that cannot sustain the workload fails instead of publishing a complete capture.
