# Technical view motion trace

The selective LiDAR renderer writes `technical_camera_trace.jsonl` beside each
preview or delivery manifest. The manifest binds the trace path, SHA-256, exact
810-row count, canonical filename, and timing basis. Delivery validation parses
every row and recomputes the complete trace from the hash-bound trajectory,
sensor transforms, technical plan, and deterministic estimated-inventory focus.
It then cross-checks shot, frame, role, times, phase, eye, target, finite
differences, and every per-view derivation receipt.

## Camera policy

- Shots 6 and 7 do not apply a presentation camera. Their trace directly audits
  the recorded `camera_optical` pose at each causal sensor-replay cutoff. Shot 6
  is persistently labeled as an earlier recorded replay at t=2.0–5.9 s; its RGB
  and LiDAR remain co-timed inside that replay. Its interpolated poses depend on
  the actual bracketing trajectory knots at 1.2–6.8 s; shot 7 likewise discloses
  its 5.6–9.2 s knot support.
- A monotone piecewise septic schedule shares velocity, acceleration, and jerk
  at every shot boundary. A global Bezier guide sampled only from the real
  piecewise estimated trajectory removes pose-knot derivative impulses.
- Shot 8 joins the map presentation orbit with a C3 blend. The orbit remains one
  continuous path through shot 12. The global smooth fit samples estimated poses
  at 2.0–18.5 s, whose actual bracketing knot support is 1.2–18.6 s; every map
  shot discloses that full dependency.
- The map path approaches the fixed primary LiDAR-supported ROI during the two
  object views, then returns to the estimated aisle trajectory. Its focus is the
  first deterministic selected row in the hash-bound estimated inventory and
  object detail must contain a LiDAR-supported ROI with that track identity.
  View boundaries do not reset angle, radius, target, or phase.
- Exactly the final 24 frames of shot 12 use an identical eye and target; the
  preceding frame must differ. Seventh-order time shaping reaches the hold with
  zero velocity, acceleration, and jerk.

The camera changes presentation only. Point selection, causal scan choice,
recorded calibration, estimated poses, raw return indices, and receipts retain
their existing source and provenance contracts.

## Trace fields

Each JSON line includes the global and view-local frame index, view ID, boundary
flag, `camera_guide_pose_timestamp_s`, `rendered_data_cutoff_s`, motion phase,
eye, target, and finite-difference velocity, acceleration, and jerk for both eye
and target. Camera guide pose time is separate from the scan cutoff. Each view
declares its sampled `camera_guide_timestamp_window_s` and its full
`camera_pose_dependency_window_s`. Construction recomputes the real
`EstimatedTrajectory` interpolation support and rejects a declaration that does
not match it. Dependency windows are therefore wider than sampled times wherever
an endpoint lies between source knots.
Positions use estimated map-frame metres and derivatives use the configured
30 fps cadence. Position vectors retain nine decimal places; derivative vectors
use a canonical seven-place rounding so the source-derived trace remains
byte-stable across supported numerical runtimes.

The four-frame RGB LRU is keyed by output dimensions and recorded frame index.
It keeps the causal previous/latest scan pair resident, avoiding FFmpeg decoder
restarts when the renderer revisits the previous co-timed frame. The cache is
bounded to four decoded frames.

The trace is designed for CPU review and does not require Isaac Sim or a GPU.
`tests/test_technical_motion.py` checks sparse nonaligned interpolation support,
adversarial mutations inside and outside disclosed knot windows, velocity,
acceleration, and jerk at every boundary on a deliberately kinked
`EstimatedTrajectory`, the final hold, timing separation, replay disclosure,
decoder restart/decode bounds, plan roles, and continuous activation-ray opacity.
Presentation tests adversarially remove and rehash trace rows and forge per-view
motion receipts.

## Diagnostic scope

Historical trace evidence from capture `20260921-053814804` is diagnostic only.
That capture predates the G02 scene and M1 trajectory and remains ineligible for
delivery. Final visual acceptance requires regenerated trace and boundary strips
from the fresh, paired 1920×1080 G02 capture.
