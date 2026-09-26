# RST-003 / R2 locomotion control candidate

Date: 2026-09-26

Status: **CPU code candidate only; an Isaac 6.1 feedback adapter and future smoke
harness now exist, but no Isaac/PhysX run has occurred and the R2 gate has not
passed**

## Routing and immutable base

- Task: `RST-003`, R2 short-path biped locomotion.
- Assigned implementation worktree:
  `C:\Users\suyog\.codex\worktrees\restock-r2-locomotion\Simulation`.
- Assigned base commit:
  `1fe5ac5c42466b3ea581b73585e919e0a8e32117`.
- Assigned base tree:
  `88c52e464ab2542ba5e3eb854b00064719f659c7`.
- Follow-on task `RST-004` owns only the Isaac feedback adapter, future smoke
  harness, focused CPU tests, and this scope report. It was initially developed
  from commit `72857a89f78449f1e6e4d0c27d2841378fae3d58`, held through the
  rejected revision 2 review, and then ported without changing `locomotion.py`
  onto the orchestrator-supplied RST-003 revision 3 base commit
  `db94dd139cd0e1e60a01557c25103a3c792ce9d6`, tree
  `29752b846a0ad72999a9c31fca251d4c32d78878`. It does not edit the
  locomotion policy, integration worktree, reviewer snapshot, or journal.
  RST-003 revision 3 remains under fresh separate review and has no acceptance
  status implied by this port.
- The task is routed to GPT-5.6 Sol with high reasoning effort. Runtime model and
  effort identity are not independently exposed to the worker and remain
  `unknown` rather than inferred from configuration.
- Implementation and acceptance review are separate assignments. This worker
  cannot approve its own candidate; every changed candidate or combined
  integration base requires a new independent review.
- The orchestrator owns integration and the authoritative implementation
  journal. This task does not edit either.

The owner-authorized restocking specification makes R2 an intermediate gate.
The full goal remains one continuous physical store-to-backroom pick, carry,
shelf placement, release, withdrawal, and stable-placement run. A deterministic
waypoint route is allowed. Teleporting the robot between phases, a substitute
wheeled base, reinforcement learning, and VLA control are outside this task.

## Acceptance and relevant-input identity

The formal RST-003 acceptance authority is the readable, hash-verified owner
specification at:

`C:\Users\suyog\.codex\visualizations\2026\09\26\01a0dc8d-fcfa-7782-bd3d-8b3cb5f8fa3e\ROBOT_BACKROOM_RESTOCKING_IMPLEMENTATION_SPEC.md`

Its raw-file SHA-256 is
`7ea5ca5fa7558aa0a58bf94999adf545a7f6b53232fd155e2cc051cb15bcf0ad`.
The owner download at
`C:\Users\suyog\Downloads\ROBOT_BACKROOM_RESTOCKING_IMPLEMENTATION_SPEC.md`
is byte-identical with the same hash. The checked-in video implementation
specification is an operating reference, not this task's acceptance spec.

The one formal relevant input manifest is
`robot_spike/production/production_manifest.json`. Its SHA-256 over the raw Git
blob payload resolved from the candidate tree is
`8c4f698803a8e877eb714a3c1d0964d284480eb95f6fe055efec9dd2ddf2b542`.
Supporting raw Git blob identities are:

- `robot_spike/production/robot_config.json`:
  `9bad70e436732051436f50d731e2b157d9ff2402aaad29cfc14fb4b47ed459b1`;
- `robot_spike/production/asimov_orcahand_restocking.urdf`:
  `b605c9a54f4a8494d333fd7cc5a20de3b4c76ffc83e33bd8027e6bafb7a1f59c`.

## Actual base architecture and command boundary

The reviewed R1 production model is the owner's Asimov biped with the right
OrcaHand attached. It has a free root, 40 revolute DOFs in the full robot, and
12 leg DOFs: bilateral hip pitch/roll/yaw, knee, ankle pitch, and ankle roll.
The left and right pitch/knee axes are mirrored in the supplied URDF; notably,
the left knee range is `[0, 1.5] rad` and the right knee range is
`[-1.5, 0] rad`.

Normal locomotion sends bounded leg targets only through the existing
`ArticulationController.command_joint_positions()` seam. Direct root pose,
root velocity, joint pose, and joint velocity writes remain confined to the
explicit deterministic reset method in `runtime.py`. The controller does not
write a root target after reset and does not represent the biped as a wheel or
planar kinematic base.

## Controller design

`robot_spike/production/locomotion.py` provides a dependency-free policy layer:

- planar waypoints become alternating left/right footsteps with a default
  maximum 0.060 m step, 8 degree yaw increment, and 0.050 m/s nominal speed;
- every swing begins from a measured foot pose and follows a quintic horizontal
  and yaw blend with a quartic 0.025 m vertical clearance bump;
- double support must be observed before liftoff;
- touchdown requires an observed airborne sample, late-swing contact, and a
  measured foot pose near the planned target;
- missing double support or touchdown ends in a bounded fault after a timeout;
- a normal stop finishes an airborne step before settling in double support;
- docking requires both contacts, low measured root velocity, target position
  and yaw tolerance, and a continuous settle dwell;
- a small feedback policy consumes measured root roll/pitch, body-frame angular
  and linear velocity, foot world pose/contact, all leg `q`/`qd`, COM and support
  center, and signed support margin;
- final targets are clamped inside the actual URDF limits and then checked by
  the production model's authoritative target validator.

`robot_spike/production/isaac_feedback.py` now provides that version-specific
integration boundary. It contains no Isaac imports and accepts injected Isaac
objects so normalization and geometry remain CPU-testable. Required readings
fail closed: malformed tensor shapes, nonfinite values, invalid or stale contact
sensor readings, missing raw contact points, a nonpositive total link mass, or
a degenerate support polygon that cannot pass the contact-conditioned geometry
gates raises `FeedbackUnavailableError`. The adapter does not substitute reset
values or nominal contact for an unavailable measurement.

The adapter reads one free-root articulation and reports:

- measured root world pose plus measured root linear/angular velocity rotated
  into the root body frame;
- measured DOF positions and velocities bound by the articulation's ordered
  `dof_names`;
- measured ankle-roll link poses converted to a sole reference derived from the
  production URDF's four radius-5 mm collision spheres;
- valid bilateral contact readings and raw world contact positions;
- whole-robot COM from measured link transforms, per-link local COM offsets,
  and per-link masses;
- a convex hull of the currently measured raw contact positions, or a strictly
  inset contact-conditioned support polygon described below, plus its area
  centroid and the COM projection's signed inward half-space margin.

Using the whole nominal sole footprint merely because a sensor reports contact
would overstate support at an edge. The adapter uses the measured raw hull when
it has at least three non-collinear points. PhysX contact reduction can supply
fewer points than the four authored collision spheres on a rigid flat foot. In
that case a contacting foot contributes only a 50%-scale inset of its URDF
four-sphere footprint, and only when its sensor has positive force, every raw
point is within 5.5 mm in XY of an authored sphere bottom, the raw points share
one plane within 0.5 mm, and all four sphere bottoms transformed by the measured
ankle pose lie on that measured plane within 0.5 mm. These conditions establish
a flat, ground-matched collision footprint for this flat-floor harness while
the inset avoids claiming the full authored boundary. A failed condition remains
unavailable rather than silently relaxing the support polygon.

## Installed Isaac Sim 6.1 API evidence

Local source inspection used the installed
`6.1.0-rc.26+release.49347.2d230af4.gl` runtime. No module containing
`SimulationApp` was imported for this inspection.

| Required reading | Verified installed source contract |
|---|---|
| Root pose | `C:\isaacsim\exts\isaacsim.core.experimental.prims\isaacsim\core\experimental\prims\impl\articulation.py`, `get_world_poses()` lines 2219–2269: root position `(N,3)` and orientation `(N,4)` in world, returned as `wxyz`. |
| Root velocity | Same file, `get_velocities()` lines 2454–2493: tensor root linear and angular velocity, each `(N,3)`. The installed underlying `omni.physics.tensors` `api.py` lines 1555–1570 identifies root velocity as global-frame; the adapter inverse-rotates both vectors into the controller's root body frame. |
| Joint state | Same file, `get_dof_positions()` lines 1951–1994 and `get_dof_velocities()` lines 2054–2095: ordered `(N,D)` tensor reads; `dof_names` lines 221–240 supplies binding order. |
| Link identity and pose | Same file, `link_names`/`link_paths` lines 410–455 supplies ordered link paths. `RigidPrim.get_world_poses()` in `impl\rigid_prim.py` lines 291–336 returns each link's world position and `wxyz` orientation. The future harness constructs this view before physics starts and rejects a path lacking `RigidBodyAPI`. |
| Contact report | `C:\isaacsim\exts\isaacsim.sensors.experimental.physics\isaacsim\sensors\experimental\physics\impl\contact_sensor.py`, `get_sensor_reading()` lines 157–185 returns explicit validity, contact state, force, and time; `get_raw_data()` lines 187–200 returns body IDs, position, normal, impulse, time, and dt. `contact.py` lines 169–199 verifies a rigid-body ancestor and applies `PhysxContactReportAPI`. NVIDIA's generated Isaac Sim 6.1 `ContactRawData` API identifies x/y/z as world coordinates: `https://docs.isaacsim.omniverse.nvidia.com/6.1.0/py/api/structisaacsim_1_1sensors_1_1experimental_1_1physics_1_1_contact_raw_data.html`. |
| Link masses and COM | Experimental articulation `get_link_masses()` lines 3830–3890 returns `(N,L)` and `get_link_coms()` lines 3892–3937 returns `(N,L,3/4)`. The underlying installed `omni.physics.tensors` `api.py` lines 2309–2328 explicitly says the principal-axis/COM pose is relative to and expressed in each rigid-body prim frame; the adapter composes it with the link world pose before mass weighting. |
| Simulation time/step | `C:\isaacsim\exts\isaacsim.core.simulation_manager\isaacsim\core\simulation_manager\impl\simulation_manager.py`, `get_simulation_time()` lines 895–911 and `step()` lines 968–1021. The harness fixes and verifies physics dt before play. |
| Application shutdown and launcher exit | `C:\isaacsim\exts\isaacsim.simulation_app\isaacsim\simulation_app\simulation_app.py`, SHA-256 `e5db812e752cc415f969c464dfe7248bb4524eaa240ba556386d3a8007f36ce5`: `DEFAULT_LAUNCHER_CONFIG` sets `fast_shutdown` true at lines 92–116, while `close()` at lines 886–1005 documents and uses an `os._exit()` fast path. The harness sets `fast_shutdown` false so `close()` returns, then explicitly exits with the durable result. Installed `C:\isaacsim\python.bat`, SHA-256 `ead7a729c2c9d37a04f11c40e754e1f814127986e14d32953fcefc09d95d9871`, maps a nonzero Kit child result to launcher result 1. Both files are included in the installed-API identity vector. |
| Support margin | Isaac supplies contact points, link state, mass, and COM rather than a ready biped support margin. The adapter's dependency-free convex-hull and signed half-space calculation is covered analytically on CPU. Reduced raw contact sets use the explicitly gated inset URDF derivation above; otherwise they fail closed. |

The safety policy also checks measured root clearance above the highest
contacting foot against a configurable `[0.45, 0.80] m` envelope. A collapsed
or implausibly elevated root latches a fault before another articulation target
is sent. The envelope brackets the R1 model's documented 0.635 m neutral reset
clearance by 0.185 m below and 0.165 m above; it is a conservative simulation
safety assumption pending measured Isaac settling data, not a hardware limit.

## Revision response to independent review

- `RST-003-F01`: fixed. Before each liftoff, the controller now compares the
  captured measured swing-foot pose with the planned target. A displacement
  above 0.060 m or yaw change above 8 degrees faults before entering swing.
  CPU regression cases cover a 0.13 m off-nominal displacement and a 20 degree
  off-nominal yaw.
- `RST-003-F02`: fixed. Body-frame lateral foot displacement now produces a
  bounded hip/ankle roll landing target while preserving URDF limits. Opposite
  lateral routes produce distinct, opposite-sign commands in the CPU test.
- `RST-003-F03`: fixed. Balance safety now derives root clearance from measured
  root height and contacting-foot world height and enforces the documented
  envelope. Collapsed, over-height, and nonfinite inputs are covered; unsafe
  feedback and latched faults issue no further gait target.
- `RST-003-R2-F04`: addressed in revision 3. Both hip-yaw axes are `-Z`, so
  desired physical foot yaw is mapped to the opposite numeric joint sign for
  both legs and turn directions. Touchdown also requires measured yaw within
  the configured 5 degree tolerance.
- `RST-003-R2-F05`: addressed in revision 3. Invalid/nonfinite feedback is
  converted into a latched, command-free controller fault. A later valid sample
  cannot resume commands without a new controller/reset lifecycle.
- `RST-003-R2-I01`: addressed in revision 3 with the exact acceptance-spec path
  and SHA-256 plus one formal raw-Git-blob input-manifest identity above.
- `RST-004-F01`: addressed in this revision. Preflight opens and hashes the
  actual owner acceptance-spec bytes, rejects a missing or mismatched file, and
  includes `production_manifest.json`, both configuration-selected URDFs, the
  runtime configuration, and all harness source inputs in the recorded vector.
- `RST-004-F02`: addressed in this revision. The complete initial owner-spec,
  candidate/input, and installed-API identity vector is re-read at the start
  and end of every repeat and immediately before final success. Every check is
  persisted; missing or changed input aborts with a nonzero result.
- `RST-004-F03`: addressed in the follow-up revision. The harness creates the
  requested exclusive output directory before identity preflight and catches
  every missing, changed, or unavailable identity input. It writes a
  `preflight_identity` error report with `runtime_invoked: false` and returns
  nonzero without calling the Isaac runtime boundary.
- `RST-004-F04`: addressed after the first serialized integration attempt.
  Isaac Sim 6.1 defaulted `SimulationApp.fast_shutdown` to true, so `close()`
  terminated the interpreter with status 0 while unwinding a runtime failure;
  the durable file remained `running` at `repeat_0_start` and no sample file was
  created. The harness now selects graceful shutdown explicitly. After the
  runner returns, `_execute_smoke` rewrites the returned terminal result; a
  propagated runtime exception writes an error report and returns 2.
- `RST-004-F05`: the graceful-shutdown retry on integration commit
  `a89a2b9d6cab022e220b2d32c6f91089396ff2c3` reached the first post-settle
  read and durably reported an insufficient support polygon. That receipt did
  not contain raw coordinates, so it proves fewer than three distinct finite
  combined XY points but does not prove the exact per-foot counts. The revised
  adapter records validity, contact state, force, sensor time/age, raw count and
  world coordinates for each foot, plus every support gate and selected support
  point. The repeat status is rewritten before a feedback exception unwinds.
- `RST-004-F06`: the same retry produced a durable runtime error but
  `C:\isaacsim\python.bat` still returned 0. The installed launcher returns 0
  when its `kit.exe` child returns 0 and maps every nonzero child result to 1.
  After graceful Isaac shutdown and durable reporting, the script now flushes
  its streams and calls `os._exit` with the computed 0/1/2 result. Thus a runtime
  error reaches the launcher as nonzero, while the durable report retains the
  more specific result and traceback. A CPU-only probe through the installed
  `python.bat` using a child `os._exit(2)` returned launcher exit code 1; it did
  not import or start Isaac.
- `RST-004-N01`: corrected to the exact official Isaac Sim 6.1 generated API
  URL above.

These are programmer changes awaiting fresh independent review; this note does
not mark any finding accepted or the R2 gate passed.

## CPU checks and what they establish

`tests/test_robot_locomotion.py` covers:

- mirrored knee/hip/ankle pitch signs and URDF-limit enforcement;
- exact swing endpoints, midpoint clearance, endpoint velocity behavior, and
  local trajectory continuity;
- low-speed bounded target-to-step planning and strict side alternation;
- double-support/liftoff, observed-unload/touchdown, docking settle, stop settle,
  and timeout transitions;
- measured-state balance input and support-margin safety response;
- static inspection that direct root/joint state setters appear only in the R1
  reset path and never in locomotion.

These tests establish deterministic policy behavior on CPU. They do not prove
stable walking, sufficient drive gains/effort, correct imported foot-frame
selection, collision behavior, foot traction, contact-sensor fidelity, COM
accuracy, or recovery from a physical disturbance.

`tests/test_robot_isaac_feedback.py` adds focused CPU coverage for Isaac tensor
shape normalization, quaternion/frame conversion, sole placement, DOF binding,
mass-weighted world COM, current-contact support polygons, signed support
margin, invalid/stale/missing sensor failure, malformed data failure, and static
confirmation that the future smoke harness contains no direct root/joint state
setter call. Invalid, nonfinite, or nonpositive run arguments are also checked
through the CPU-safe preflight boundary. Regression cases mutate or remove the
acceptance spec, manifest hash, and installed-API hash and verify that preflight,
repeat, and final checks fail closed after persisting the mismatch. Launcher
sentinels confirm preflight failures do not call the runtime boundary. Static
inspection requires `fast_shutdown: false`; injected post-shutdown runners
verify durable pass/fail results with exit codes 0/1 and durable runtime errors
with exit code 2.

## Future one-job evidence harness

`robot_spike/production/run_locomotion_smoke.py` is an explicit future heavy-job
entry point. Importing it does not import Isaac. Before constructing
`SimulationApp`, it requires a clean worktree and exact expected candidate and
tree SHAs. It opens and hashes the actual owner acceptance spec, the production
manifest, runtime configuration, production and approved-source URDFs, every
relevant source/test/report input, and every local Isaac API source file on
which the adapter depends. The exclusive output directory is created before
this identity read; a preflight error is durably recorded there and returns
nonzero before the Isaac runtime boundary is called. The preserved initial
vector is re-read and compared at both boundaries of every repeat and before
final success. Checks, including failures, are written to the durable status
report; a missing or changed byte prevents a passing result. A run uses a fixed
physics rate, verifies metre stage units, and launches `SimulationApp` with
`fast_shutdown: false`. Successful or exceptional shutdown therefore returns
to the reporting boundary. Pass/fail state is written before shutdown and
rewritten after `close()`; a runtime exception is likewise written as a terminal
error with `shutdown_returned: false` before close, then wrapped in the final
durable error after close. The run performs one explicit
deterministic reset per repeat, and uses only drive position targets after each
reset. It writes every controller step to one JSONL measured sample stream per
repeat plus an incrementally durable status report containing root motion,
target error, clearance, tilt, velocity, contact forces/transitions/raw-point
counts, contacting-sole slip, joint state, one-step-lag target error, COM,
signed support margin, terminal state, fault reason, and repeat deltas. Missing
sensing also persists per-foot contact values, times, raw coordinates, support
gates, and candidate support points before producing a nonzero exit. A timeout
or controller fault produces a failed result and nonzero exit. After the status
write and graceful `SimulationApp.close()`, an explicit process exit conveys
that result to `python.bat`; its public exit is 0 for pass and 1 for either
failed or errored runs.

The later serialized invocation must name the exact independently reviewed
snapshot, for example:

```powershell
C:\isaacsim\python.bat robot_spike\production\run_locomotion_smoke.py `
  --output <new-exclusive-output-directory> `
  --expected-candidate-sha <reviewed-commit> `
  --expected-candidate-tree-sha <reviewed-tree> `
  --repeats 2
```

Two predecessor integration attempts exist, both failed before gait commands.
Neither is passing evidence for this candidate. The script itself is not
evidence that sensing, inferred support, walking, docking, or reset
repeatability works.

## Scheduled Isaac validation requirements

No Isaac, PhysX, or GPU process is launched by this task. R2 remains pending
until the orchestrator schedules one heavy job and an independent reviewer
checks the exact integrated candidate. That validation must at minimum:

1. import and settle the exact 40-DOF free-root production articulation;
2. prove the installed adapter actually binds to measured root state, bilateral
   sole/contact state and raw contact positions, leg `q`/`qd`, and per-link
   mass/COM reads with the recorded imported link order;
3. verify the foot reference frame and contact target height used by the plan;
4. tune articulation drives without adding root-state writes;
5. walk a short collision-free path at the bounded speed and step length;
6. show alternating physical swing, contact-gated touchdown, stop, and dock;
7. record falls, slips, timeouts, tracking error, actual root motion, and exact
   source/config identities rather than treating a command trace as motion;
8. repeat reset and the short-path run to check reproducibility.

The source-level API names and return shapes are now resolved for the installed
6.1 build. The failed `a89a2b9` run verified the imported link order and reached
valid enough contact data to attempt support geometry, but its old receipt did
not preserve exact per-foot values. Physical behavior remains unresolved: the
actual per-foot raw contact counts and coordinates, whether the strict inset
support gates pass under settling and swing, correctness of the selected sole
reference under load, mass/COM fidelity, drive authority, friction/slip,
self-collision policy, fall recovery, gait stability, docking, and repeatability
all require the next serialized Isaac run. No CPU test can promote those limits
to a physical pass.
