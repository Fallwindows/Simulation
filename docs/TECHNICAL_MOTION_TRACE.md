# Technical view motion trace

The selective LiDAR renderer writes `technical_camera_trace.jsonl` beside each
preview or delivery manifest. The manifest binds the trace path, SHA-256, frame
count, and timing basis. The trace contains one row for each of the 810 source
frames used by technical shots 6–12.

## Camera policy

- Shots 6 and 7 report the recorded camera optical pose from the bound estimated
  trajectory. Source time uses half-open sampling so adjacent shots do not
  duplicate a boundary pose or introduce a zero-velocity frame.
- Shot 8 begins at the next recorded optical pose. A quintic bridge preserves
  position, velocity, and acceleration while moving onto one continuous map
  camera path shared by shots 8–12.
- The map path approaches the fixed primary LiDAR-supported ROI during the two
  object views, then returns to the estimated aisle trajectory. View boundaries
  do not reset its angle, radius, target, or phase.
- The final 24 frames of shot 12 use an identical eye and target. Quintic time
  shaping reduces velocity and acceleration before that hold.

The camera changes presentation only. Point selection, causal scan choice,
recorded calibration, estimated poses, raw return indices, and receipts retain
their existing source and provenance contracts.

## Trace fields

Each JSON line includes the global and view-local frame index, view ID, boundary
flag, source timestamp, motion phase, eye, target, and finite-difference eye
velocity and acceleration. Positions use estimated map-frame metres. Velocity
and acceleration use the configured 30 fps source cadence.

The trace is designed for CPU review and does not require Isaac Sim or a GPU.
`tests/test_technical_motion.py` checks every boundary, the final hold, plan
roles, and continuous activation-ray opacity.

## Diagnostic scope

Historical trace evidence from capture `20260921-053814804` is diagnostic only.
That capture predates the G02 scene and M1 trajectory and remains ineligible for
delivery. Final visual acceptance requires regenerated trace and boundary strips
from the fresh, paired 1920×1080 G02 capture.
