# Production 4K sensor benchmark

Date: 2026-09-21

Source base: `b18eedde2c2608f5ee250039f7b90398b950e1e3` plus the candidate configuration in this change

Host: Windows 11, Isaac Sim 6.1, RTX 4070 Ti 12 GB, ROS 2 Jazzy through the existing Pixi workspace

## Configuration and invocation

`config/scenarios/production_walking_4k.yaml` selects native 3840×2160 RGB at 30 Hz while reusing the baseline LiDAR configuration and the complete 20.5-second walking trajectory. The 1280×720 `walking_baseline.yaml` remains the preview/test profile.

The native benchmark used `scripts/capture_simulation.ps1` without bypasses or fallback:

```powershell
.\scripts\capture_simulation.ps1 `
  -Scenario runs/benchmark-4k/production_walking_4k_short.yaml `
  -Headless
```

The ignored benchmark scenario reused the production environment, sensor, and mapping paths and changed only walking trajectory duration from 20.5 seconds to 1.0 second. The production sensor file SHA-256 is `ddaa1198b917434025936a6171cdd454e2800e9a7b809079807e82b5f9069da8`; the short trajectory SHA-256 is `977923caf6416a7d092d15b4a3547e257a7c3a7536bad1897de77483e6544d6a`.

## Result

- Wall time: 62.348 seconds, including router, recorder/bag startup, Isaac startup and shutdown, and manifest finalization.
- Isaac simulated 60 frames through 1.0 seconds. Runtime camera cadence reports requested/effective 30.0 Hz and 27 observed frames.
- The recorder wrote 27 valid 3840×2160 RGB8 frames from stamp 0.1 through 1.0 seconds. FFprobe reports H.264, 3840×2160, 30/1 encoded frame rate, 27 frames, and 0.900-second encoded duration.
- Recorder timestamp cadence had 25 intervals of 33.333 ms and one 66.667 ms interval between stamps 0.5 and 0.566667; reported timestamp-derived rate is 28.889 fps. There were no invalid, duplicate, out-of-order, or post-target frames.
- The bag reached the absolute 1.0-second simulation horizon at clock 1.033333. Counts were 57 clock, 24 RGB, 9 LiDAR, 60 TF, and 1 static TF message; all five required topics and native types are present.
- LiDAR retained 10 Hz, 0.2–60 m range, the accepted rotary asset, and 134,643–146,052 points per observed cloud.
- The canonical capture manifest validates with capture SHA-256 `f6b4156f041b8e5c7a844c431976b357b8eb3d9994a4689b03f39a796fed6657`. SQLite integrity is `ok`, journal mode is `delete`, 151 messages are present, and no WAL/SHM sidecars remain.
- Capture bytes total 618,384,764. The sqlite3 bag is 613,064,704 bytes and the H.264 RGB video is 3,676,976 bytes.
- GPU telemetry peaked at 100% utilization, 5,336 MB of 12,282 MB reported memory, and 179.99 W. Isaac reported 14,997 MB free system memory at startup.

A simple measured-rate projection is about 12.68 GB for 20.5 seconds. A complete 30 Hz raw RGB stream alone is 15.30 GB before sqlite3, LiDAR, TF, and metadata overhead, so a production run should reserve more than 16 GB and confirm sustained subscriber cadence before relying on the lower measured projection. The one-second wall time is dominated by fixed startup and shutdown and is not a reliable full-run wall-time estimate.

## Visual inspection

Decoded frames 0, 13, and 26 at stamps 0.1, 0.566667, and 1.0 seconds were inspected at native 3840×2160. All three show the expected aisle with stable exposure and geometry, forward walking motion, fine package edges, and no black, corrupt, or partially encoded regions. Some authored package faces show reversed/mirrored label text in all three frames; that is existing scene/asset content rather than a sensor-resolution defect.

Evidence is local and ignored by Git:

- Capture: `runs/20260921-025252875/capture/`
- Capture manifest SHA-256: `3b8db0250c797cdd07cb95cee4dde3084ad251cb11518d83d46ad7b44d8a9e15`
- RGB MP4 SHA-256: `8f4fa3e00a92400bb5e6b4bba1eab8ca7fbc3a41750e9588017f3eabac412506`
- Bag DB SHA-256: `2358cfa1643c8f1d4ae0f766e20903685d0d0dac8639af703095e80346a980b9`
- GPU telemetry: `runs/benchmark-4k/gpu-monitor.csv` (`9822494361f444c2f8a8164cfed2279ece8339bb05aecd08a034a5b002991c3b`)
- Inspected PNG SHA-256 values: `82026bb8…`, `32cf6a44…`, and `57b31112…`

This benchmark proves native 4K generation, recording, bagging, and artifact finalization on the target host. Its short duration exposes one recorder-frame interval gap and a lower bag RGB count during startup; a full production capture still needs its own sustained-cadence and storage check.
