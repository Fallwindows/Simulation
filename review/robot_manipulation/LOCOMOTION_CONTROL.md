# RST-003 / R2 locomotion control candidate

Date: 2026-09-26

Status: **CPU revision candidate after failed Isaac 6.1 physical attempts; the
R2 gate has not passed and this revision has not run in Isaac/PhysX**

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
  locomotion policy, integration worktree, or reviewer snapshot.
  The current staged-startup revision is based exactly on approved integration
  commit `a3f2899f6a968b36d43f15ca9a17f8efe4ee1e01`, tree
  `b003f8f5c0bf19721f06fcd06f23427289f5dd55`.
- The task is routed to GPT-5.6 Sol with high reasoning effort. Runtime model and
  effort identity are not independently exposed to the worker and remain
  `unknown` rather than inferred from configuration.
- Implementation and acceptance review are separate assignments. This worker
  cannot approve its own candidate; every changed candidate or combined
  integration base requires a new independent review.
- The orchestrator owns integration. This revision updates only the requested
  R2 runtime entry in the authoritative implementation journal.

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
four-sphere footprint only when at least three sphere support points qualify.
The adapter transforms the authored sphere centers, then derives each sphere's
world-lowest point by subtracting its radius along world Z. Transforming a
link-local bottom offset would rotate the bottom direction with a tilted foot
and is physically wrong for a sphere. The sensor must have positive force, raw
points must share one plane within 0.5 mm, and every raw point must map within
5.5 mm in XY to an authored sphere center. Only sphere world-lowest points within
0.5 mm of that measured plane become candidates. Three or more candidates on
one foot are inset by 50%; one or two remain explicit points and cannot make a
single-foot polygon. A failed condition remains unavailable rather than silently
relaxing the support polygon.

Installed `ContactSensor.get_raw_data()` identifies the parent bodies but does
not identify the individual collision shape. The production ankle-roll links
have exactly these four collision spheres and no other collision geometry, so
the fallback records and checks the raw-point-to-sphere spatial match rather
than claiming a shape ID that Isaac did not report.

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
- `RST-004-F07`: exact candidate `58f12985cc208922a119e0d4ec49733f2b4ef367`
  reached the first post-settle read under elevated execution and supplied the
  previously missing evidence. Only the left foot reported contact: 39.5973 N
  and two raw world points on Z=0; the right reported no contact. The diagnostic
  code had transformed link-local sphere-bottom offsets. Because the measured
  foot was strongly pitched, two such points appeared about 0.17 m above the
  floor and the lower pair were shifted in XY. The revision stores the actual
  URDF sphere centers and derives world-lowest points along world Z, filters
  candidates by the measured plane, and records raw-to-sphere matches. It will
  still reject this observed one-foot/two-point state instead of turning it into
  a support polygon.
- `RST-004-F08`: the approved a3 correction ran from exact integration commit
  `a3f2899f6a968b36d43f15ca9a17f8efe4ee1e01`. After the old blind two-second
  settle, it again found only a left heel contact: 39.59727097 N and two Z=0
  raw points; the right sensor was valid with zero force and no points. Only
  left sphere index 1 reached the plane; index 0 was 2.1047 mm high and the
  other two were 0.166--0.169 m high. The adapter now retains root pose,
  quaternion/RPY, world/body velocities, leg q/qd, and both ankles' pose and
  sphere geometry before support construction, including the noncontact side.
  The harness replaces blind settling with a bounded measured startup: acquire
  real bilateral polygonal support, ramp from measured joints to the symmetric
  crouch with drive targets, then hold a verified dwell. Every increment checks
  contact/support, root tilt and speed, clearance, target tracking, and sole
  flatness. Failure aborts before the gait controller.
- `RST-004-R2-F01`: fixed after exact review of `eba801d`. The first staged
  candidate approximated sole tilt with the four-sphere footprint's diagonal
  height spread; a narrow-foot 30 degree roll could pass that proxy. The gate
  now rotates the authored sole-plane normal (ankle-link local +Z, established
  by all four sphere centers sharing one local Z) through the normalized
  measured ankle quaternion and measures its angle to world +Z. This is yaw and
  Euler-wrap invariant, rejects an upside-down sole, accepts at most the exact
  existing 12 degree limit, and rejects the reviewer's reproduced 30 degree
  pure roll.
- `RST-004-R2-F02`: fixed after the same review. Every contact-acquisition read,
  including unavailable support and nonbilateral feedback, now evaluates root
  tilt/speed, geometric root-to-sole clearance, and both measured sole-normal
  angles before another physics step. It does not claim either sole is support.
  A failed gate is stored with the causal kinematics and aborts immediately.
  CPU regressions cover unavailable support with 30 degree ankle roll and a
  0.20 m root height.
- `RST-004-F09`: exact approved integration `0aa61bc40ca829f69e02ce5f16d602a72bcee3be`
  reached bilateral support at 0.100 s with approximately 795 N left and 816 N
  right, but that single sample was an impact transient: root Z was 0.6303 m
  and upward speed was about 0.077 m/s. The first crouch-ramp target one frame
  later coincided with left contact falling to zero while four raw points
  remained; right contact was 20.44 N and the single-foot support margin was
  -79.9 mm. The durable receipt is
  `C:\Users\suyog\.codex\visualizations\2026\09\26\01a0dc8d-fcfa-7782-bd3d-8b3cb5f8fa3e\R2-smoke-0aa61bc\locomotion_smoke_status.json`;
  the Kit log is
  `C:\isaacsim\kit\logs\Kit\Isaac-Sim Python\6.1\kit_20260926_054500.log`.
  The revision uses the existing 1.2 s double-support timeout as one bounded
  pre-ramp stabilization window. Reset-seeded targets remain active without a
  post-reset command. A full 0.30 s of consecutive, advancing samples must have
  fresh positive bilateral contact, acceptable support margin, reset-target
  tracking, root stability/clearance, and both soles within 12 degrees. Contact
  or support loss resets the dwell; a hard safety violation aborts; timeout
  remains fail closed. No crouch target is sent before stability passes.
- `RST-004-R3-F01`: fixed after exact review of `f81801a`. The seconds-only
  argument check understated the budget when 0.30 s maps to a nonintegral
  number of physics steps. Preflight and runtime now call one discrete budget
  function. At 14 Hz each 0.30 s phase needs five steps, so the minimum is 16
  total steps: one acquisition step plus five each for stabilization, ramp, and
  final dwell. The previously accepted 0.9714285714 s value now fails before
  `SimulationApp`; the exact 16/14 s boundary produces a six-step pre-ramp
  window and passes.
- `RST-004-F10`: exact approved integration
  `5f18484550ac000efc4cfc6ee529c3bf48aa5962` passed pre-ramp stabilization
  and all 36 fixed-crouch ramp samples, then failed safely during verified
  dwell. At dwell step 26 the measured root had drifted to X=-0.0550 m with
  pitch -0.1492 rad and 0.162 m/s linear speed; both feet remained in contact,
  but the signed COM support margin was -0.015887 m against the unchanged
  -0.015 m gate. The harness now uses the existing bounded
  `BalanceFeedbackController` and double-support
  `ConservativeGaitTargetGenerator` on every ramp and dwell increment. This
  makes the nominal crouch target responsive to measured pitch, velocity, and
  COM offset with the existing 0.045 rad correction cap. Any unsafe input or
  existing support, tilt, speed, clearance, sole, or tracking gate still aborts
  before gait. The durable receipt is
  `C:\Users\suyog\.codex\visualizations\2026\09\26\01a0dc8d-fcfa-7782-bd3d-8b3cb5f8fa3e\R2-smoke-5f18484\locomotion_smoke_status.json`;
  the Kit log is
  `C:\isaacsim\kit\logs\Kit\Isaac-Sim Python\6.1\kit_20260926_061041.log`.
- `RST-004-F11`: exact approved integration
  `1794b3b1dd28508cb358780b76139a94a1f702c9` confirmed that the measured
  balance target was active but mapped in the destabilizing sagittal
  direction. All 36 ramp samples passed, with margin decreasing from +0.030 m
  to +0.010 m. Dwell step 22 failed at -0.016013906 m, with measured pitch
  -0.15350 rad, forward velocity -0.1804 m/s, pitch rate -0.4375 rad/s, and
  root X=-0.05428 m. The controller's positive sagittal correction had reached
  the existing +0.045 rad cap. Adding its 35/65 split to hip and ankle made
  the planted-leg pitch sum positive; flat-foot kinematics therefore commanded
  negative pelvis pitch, reinforcing the measured error. The generator now
  subtracts that same bounded sagittal correction. It does not change gains,
  caps, safety gates, lateral mapping, or state-write policy. Ramp and dwell
  events also record the exact feedback inputs and bounded correction used for
  each command. The receipt is
  `C:\Users\suyog\.codex\visualizations\2026\09\26\01a0dc8d-fcfa-7782-bd3d-8b3cb5f8fa3e\R2-smoke-1794b3b\locomotion_smoke_status.json`;
  the Kit log is
  `C:\isaacsim\kit\logs\Kit\Isaac-Sim Python\6.1\kit_20260926_062729.log`.
- `RST-004-F12`: exact reviewed integration
  `ff0ed920113b9b23ce82bb05242a835b750774fd` confirmed the sagittal sign fix
  helped but exposed insufficient drive tracking. All 36 ramp samples and 32
  dwell samples passed before margin reached -0.015725126 m at dwell step 33.
  Both measured planted-leg pitch sums were still about +0.144 rad and moving
  farther positive at +0.39/+0.34 rad/s, while both issued target sums were
  about -0.049 rad. The existing target-error gate still passed at 0.08814 rad.
  The target velocity damping is now 0.100 s, exactly the imported force
  drive's `Kd/Kp = 12/120` ratio. Through the position target this adds one
  matching velocity damping term without changing importer gains, effort
  limits, the 0.045 rad balance cap, or any physical gate. It is capped at
  0.100 s and every resulting joint target remains bounded by the existing
  0.30 rad measured target-error gate. Ramp/dwell observations now retain leg
  q/qd alongside the previously recorded root/COM/correction data. The receipt
  is
  `C:\Users\suyog\.codex\visualizations\2026\09\26\01a0dc8d-fcfa-7782-bd3d-8b3cb5f8fa3e\R2-smoke-ff0ed92\locomotion_smoke_status.json`;
  the Kit log is
  `C:\isaacsim\kit\logs\Kit\Isaac-Sim Python\6.1\kit_20260926_064457.log`.
- `RST-004-F13`: exact reviewed candidate
  `058fdebdc9e35c606aa1025850323b3745049f3e` completed all 36 ramp and 36
  dwell samples, then faulted on the fourth controller sample before swing.
  The last accepted startup sample already had margin -0.012447 m, linear
  speed 0.115 m/s, and angular speed 0.268 rad/s. In the next 25 ms the root
  moved from X=-0.05289 m to -0.05610 m and margin crossed the unchanged gate
  at -0.015720972 m; both feet stayed loaded on the measured eight-point hull.
  The first gait commands were still double-support targets close to the last
  startup target, so this was an unsafe moving handoff rather than a swing
  discontinuity. The saturated sagittal correction is now capped at 0.090 rad,
  equal to each nominal crouch hip/ankle pitch magnitude. The lateral correction
  retains its separately enforced 0.045 rad cap. The sagittal 35/65 split is
  therefore limited to 0.0315/0.0585 rad per joint; URDF limits and the existing
  0.30 rad measured target-error gate remain authoritative. Verified dwell now
  requires a complete 0.30 s consecutive handoff window under the existing
  docking tolerances of 0.035 m/s linear and 0.10 rad/s angular speed. It may
  use only the unused portion of the original two-second startup budget;
  unsettled samples reset the consecutive counter, while any existing hard
  gate still aborts immediately before gait. The receipt and sample stream are
  `C:\Users\suyog\.codex\visualizations\2026\09\26\01a0dc8d-fcfa-7782-bd3d-8b3cb5f8fa3e\R2-smoke-058fdebd-run1\locomotion_smoke_status.json`
  and sibling `repeat_00_samples.jsonl`; the Kit log is
  `C:\isaacsim\kit\logs\Kit\Isaac-Sim Python\6.1\kit_20260926_064457.log`.
- `RST-004-R2-F03`: exact review of candidate
  `8f8c1e16499625401c8f1363250f937f3aab7be6` found that its single balance
  limit also doubled unvalidated lateral authority. The disposition separates
  axis limits: sagittal remains bounded at 0.090 rad from the runtime evidence,
  while lateral remains at the reviewed 0.045 rad bound. An adversarial safe
  sample with nonzero roll, lateral velocity, and roll rate saturates lateral
  correction at 0.045 rad; its issued roll targets remain within the unchanged
  URDF and 0.30 rad measured-target-error gates.
- `RST-004-F14`: exact combined integration
  `e95721e641ea6a82ee53b4e49f118fe0574a0df4` reached the revised verified
  dwell but never produced one settled confirmation. The fixed ramp had begun
  at about 0.0425 m/s root speed and kept advancing after angular speed exceeded
  the existing 0.10 rad/s handoff tolerance. Across ramp and dwell, measured
  pitch and backward motion grew monotonically; dwell step 40 rejected margin
  -0.015189933 m at 0.10408 m/s linear and 0.2793 rad/s angular speed. The
  successor requires the existing 0.035 m/s and 0.10 rad/s handoff speeds during
  pre-ramp stabilization and before advancing each of the same 36 target-ramp
  interpolation increments. An unsettled sample repeats the current fraction
  while retaining the full measured restoring offset instead of scaling that
  offset down with crouch progress. The target remains inside the convex envelope
  of measured start, nominal crouch, and validated balanced target, plus the
  existing 0.30 rad measured-target-error gate. The original two-second startup
  bound reserves the complete final dwell; hard-gate failure or ramp-budget
  exhaustion still aborts before dwell and gait. The receipt is
  `C:\Users\suyog\.codex\visualizations\2026\09\26\01a0dc8d-fcfa-7782-bd3d-8b3cb5f8fa3e\R2-smoke-e95721e\locomotion_smoke_status.json`;
  the Kit log is
  `C:\isaacsim\kit\logs\Kit\Isaac-Sim Python\6.1\kit_20260926_073705.log`.
- `RST-004-F15`: exact integration
  `efdf53e222bfd4321f03a58811a03c44cc81dddb` failed closed during pre-ramp
  stabilization at step 126 and 1.116667 s. Root tilt reached 0.213279 rad
  against the unchanged 0.20944 rad gate; speed was 0.25449 m/s, angular
  speed was 0.55367 rad/s, and measured support margin was -0.0593401 m.
  No ramp or gait command ran. The trace starts with the configured root at
  Z=0.635 m and authored zero-pose sphere lows 4.654 mm above the floor. It
  then records free fall to Z=0.630339 m, bilateral impact near 795/816 N,
  and monotonic pitch/COM drift while the all-zero drive targets are held.
  The successor derives the **sole explicit reset** root height from the exact
  production URDF zero-pose chain and authored sole collision spheres. It uses
  Z=0.630346 m: all right sphere lows are on Z=0 and all left lows penetrate
  only 1.33615 micrometres, with a 1.33615 micrometre bilateral spread inside
  the unchanged 0.5 mm support-plane tolerance. Joint reset values remain
  all zero. After that reset, motion still uses drive position targets only;
  every contact, stability, clearance, sole-angle, tracking, and handoff gate
  remains unchanged and fail closed. The receipt is
  `C:\Users\suyog\.codex\visualizations\2026\09\26\01a0dc8d-fcfa-7782-bd3d-8b3cb5f8fa3e\R2-smoke-efdf53e\locomotion_smoke_status.json`;
  the Kit log is
  `C:\isaacsim\kit\logs\Kit\Isaac-Sim Python\6.1\kit_20260926_075615.log`.
- `RST-004-F16`: exact reviewed integration
  `09e7526ba350d62ed6b64cc41215c2105ff9eb8b` confirmed that the URDF-derived
  Z=0.630346 m reset removed the prior free-fall impact. Bilateral support was
  measured at step 2 near 151.5/168.2 N with +0.04014 m margin. The all-zero
  straight-leg hold then accumulated only 33 of the required 36 stable samples:
  at step 35 root speed crossed the unchanged 0.035 m/s handoff tolerance while
  contact and margin remained safe. Because no drive target was allowed before
  the dwell completed, backward speed, pitch, and COM displacement then grew
  monotonically until the unchanged 12 degree tilt gate rejected step 126.
  At rejection root X was -0.094669 m, pitch -0.212893 rad, speed 0.254408 m/s,
  angular speed 0.554151 rad/s, and support margin -0.0591884 m; both feet
  remained loaded near 160 N. This is an unstable straight-knee hold rather
  than an impact or contact-sensing failure.

  The successor keeps the exact supported reset and adds a bounded measured
  **pre-ramp balance recovery** using position targets only. It remains inactive
  while the initial dwell is settling normally. It activates only when measured
  root speed leaves the existing handoff tolerance while fresh bilateral
  contact, advancing timestamps, the existing support-margin gate, sole/root
  geometry, target tracking, and every hard gate remain valid. The target uses
  the reviewed balance controller and gait target generator with zero nominal
  knee flexion, so it stabilizes around the reset pose and cannot begin the
  crouch ramp. Its existing sagittal/lateral caps, Kd/Kp-derived target damping,
  URDF limits, and 0.30 rad measured-target-error gate still apply. Loss of
  contact/support or any hard-gate failure produces no recovery command and
  still aborts or exhausts the bounded window. The ramp and gait remain blocked
  until a new complete 0.30 s stable dwell succeeds. The receipt is
  `C:\Users\suyog\.codex\visualizations\2026\09\26\01a0dc8d-fcfa-7782-bd3d-8b3cb5f8fa3e\R2-smoke-09e7526\locomotion_smoke_status.json`;
  no installed Kit-log path was supplied with this handoff.
- `RST-004-F17`: exact independently reviewed integration
  `6735df61347cda74f5eb811088a6361444fc8b48` activated measured pre-ramp
  recovery at the expected step 35 but exhausted the bounded 1.2 s window
  without 36 consecutive stable samples. The recovery direction and cadence
  were correct: at activation measured left/right physical pitch-chain sums
  were +0.0194/-0.0199 rad while commanded sums were -0.0269/+0.0338 rad,
  opposing the fall, and new commands were issued at every eligible sample.
  Authority was insufficient. The evaluated sagittal correction began at only
  +0.0246466 rad and rose gradually to +0.0714021 rad by step 120; it never
  reached the already reviewed +0.090 rad cap before margin left the existing
  support gate and commands correctly stopped. Root speed nevertheless grew
  from 0.0353 m/s at activation to 0.17 m/s by step 144, with pitch -0.167 rad
  and margin -0.039 m. Ramp and gait never began.

  The successor changes pre-ramp recovery authority only. When measured body
  forward speed or pitch angular speed itself exceeds the corresponding
  existing handoff tolerance, it keeps the balance controller's measured
  direction but applies the existing reviewed 0.090 rad sagittal cap. The
  minimum-knee joint-limit margin leaves an actual zero-pose physical chain
  target of 0.085 rad; per-joint target error remains subject to the unchanged
  0.30 rad gate and every URDF limit. Evaluated and applied corrections are
  both recorded. Lateral correction remains exactly as evaluated and within
  its separately reviewed 0.045 rad cap. Unsafe input, contact/support loss,
  stale time, geometry/tracking failure, or any hard-gate failure still issues
  no recovery target. A full stable dwell remains mandatory before crouch or
  gait. The receipt is
  `C:\Users\suyog\.codex\visualizations\2026\09\26\01a0dc8d-fcfa-7782-bd3d-8b3cb5f8fa3e\R2-smoke-6735df6\locomotion_smoke_status.json`;
  the supplied Kit-log filename is `kit_20260926_084122.log`.
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
- measured-state balance input, fixed-foot restoring-direction mapping, and
  support-margin safety response;
- static inspection that direct root/joint state setters appear only in the R1
  reset path and never in locomotion.

These tests establish deterministic policy behavior on CPU. They do not prove
stable walking, sufficient drive gains/effort, correct imported foot-frame
selection, collision behavior, foot traction, contact-sensor fidelity, COM
accuracy, or recovery from a physical disturbance.

`tests/test_robot_isaac_feedback.py` adds focused CPU coverage for Isaac tensor
shape normalization, quaternion/frame conversion, sole placement, DOF binding,
mass-weighted world COM, current-contact support polygons, signed support
margin, invalid/stale/missing sensor failure, and malformed data failure. A
pitched-foot regression proves sphere centers are transformed before radius is
subtracted along world Z, high spheres are excluded, raw points are spatially
matched to authored collisions, and a one-foot/two-point state remains
unavailable. Failure regressions verify that root, joints, and both ankles'
world sphere geometry survive a support exception. Staged-startup regressions
cover target interpolation and timing, the exact 36-ramp/22-dwell margin
failure progression, an impact sample followed by contact
loss and reacquisition, reset-target hold limits, bounded timeout persistence,
unsafe root state, ramp contact/tracking failure, and durable causal diagnostics.
Static inspection confirms
that the smoke harness contains no post-reset direct root/joint state setter
call. Invalid, nonfinite, or nonpositive run arguments are also checked
through the CPU-safe preflight boundary. Regression cases mutate or remove the
acceptance spec, manifest hash, and installed-API hash and verify that preflight,
repeat, and final checks fail closed after persisting the mismatch. Launcher
sentinels confirm preflight failures do not call the runtime boundary. Static
inspection requires `fast_shutdown: false`; injected post-shutdown runners
verify durable pass/fail results with exit codes 0/1 and durable runtime errors
with exit code 2.

## Serialized one-job evidence harness

`robot_spike/production/run_locomotion_smoke.py` is an explicit heavy-job
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
physics rate. CPU preflight converts every startup phase to integer physics
steps and requires one acquisition step plus complete stabilization, ramp, and
final-dwell budgets before launching Isaac. It verifies metre stage units and launches `SimulationApp` with
`fast_shutdown: false`. Successful or exceptional shutdown therefore returns
to the reporting boundary. Pass/fail state is written before shutdown and
rewritten after `close()`; a runtime exception is likewise written as a terminal
error with `shutdown_returned: false` before close, then wrapped in the final
durable error after close. The run performs one explicit
deterministic reset per repeat, and uses only drive position targets after each
reset. Instead of sending a crouch target after one bilateral sample, it uses
the existing 1.2 s timeout as a shared pre-ramp stabilization window. The reset
targets stay active without another command. Contact or support loss resets the
consecutive counter, and the ramp remains locked until 0.30 s of fresh positive
bilateral contact, advancing timestamps, adequate support margin, target
  tracking, root/sole safety, and the existing 0.035 m/s linear and 0.10 rad/s
  angular handoff speeds has accumulated. It then interpolates from the measured
stable joint state toward the existing balance-corrected symmetric crouch in 36
nominal progress increments at 120 Hz. A progress increment advances only after
the same handoff speed gates pass; while moving, the harness repeats that small
fraction and refreshes the full measured restoring offset. That offset and the
nominal fraction remain within the convex envelope of validated start/nominal/
balanced targets and the existing measured-target-error limit. The original
startup duration reserves all steps for the required 0.30 s consecutive final
dwell. Each increment uses
the existing control gates: 12 degree root/sole tilt envelope, 0.35 m/s root
linear speed, 1.0 rad/s root angular component, 0.30 rad joint target error,
`[-0.015 m]` minimum support margin, and `[0.45, 0.80] m` root clearance. Sole
flatness applies the same 12 degree gate directly to the measured angle between
the authored sole normal and world up; it does not add or loosen a physical
threshold. During pre-ramp stabilization, contact/support loss resets the dwell
within its bound; hard instability, sole tilt, clearance, or tracking failure
aborts immediately. During ramp/dwell, any failed gate aborts before constructing
the gait controller. It writes every controller step to one JSONL measured sample stream per
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

The recorded predecessor integration attempts all failed before gait commands.
None is passing evidence for this candidate. The script itself is not evidence
that sensing, inferred support, walking, docking, or reset repeatability works.

## Scheduled Isaac validation requirements

No Isaac, PhysX, or GPU process is launched for this source revision. R2 remains pending
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
6.1 build. The failed `58f1298` and `a3f2899` runs verified the imported link
order and recorded the exact initial contact state. The a3 receipt is
`C:\Users\suyog\.codex\visualizations\2026\09\26\01a0dc8d-fcfa-7782-bd3d-8b3cb5f8fa3e\R2-smoke-a3f2899\locomotion_smoke_status.json`;
its Kit log is
`C:\isaacsim\kit\logs\Kit\Isaac-Sim Python\6.1\kit_20260926_051145.log`.
It found only two left-foot raw points with the right foot airborne after the
blind settle, so the robot did not have polygonal support for gait. Physical
behavior remains unresolved: whether the staged measured startup
state produces bilateral support, whether the strict sphere-plane inference is
ever needed in a flat supported pose, correctness of the selected sole reference
under load, mass/COM fidelity, drive authority, friction/slip, self-collision
policy, fall recovery, gait stability, docking, and repeatability all require a
later serialized Isaac run. No CPU test can promote those limits to a physical
pass.
