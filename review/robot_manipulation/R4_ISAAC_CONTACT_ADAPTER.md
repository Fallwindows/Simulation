# R4 Isaac 6.1 contact adapter candidate

## Scope and identity

This source-only candidate connects the reviewed R4 controller to measured
Isaac Sim 6.1 state. It does not claim an Isaac or PhysX execution result.

- Integration base commit: `2212593c7851833fc408c0c9317814a1de9c1ec4`
- Integration base tree: `4f1a4ae42d09008a2445824bf638535ddba69c5b`
- Owner specification: `C:\Users\suyog\Downloads\ROBOT_BACKROOM_RESTOCKING_IMPLEMENTATION_SPEC.md`
- Owner specification SHA-256: `7ea5ca5fa7558aa0a58bf94999adf545a7f6b53232fd155e2cc051cb15bcf0ad`
- Installed Isaac identity inspected without starting it: `6.1.0-rc.26+release.49347.2d230af4.gl`
- Approved combined source URDF canonical SHA-256: `25c62dd8459721ea41e7c3325ab6d4a816e05b34daa89759297151d464be84ea`
- Production URDF canonical SHA-256: `b605c9a54f4a8494d333fd7cc5a20de3b4c76ffc83e33bd8027e6bafb7a1f59c`
- Production URDF byte SHA-256 in this checkout: `16fd6716e21260cce3bbaa439bb7ed22c5bb12bbce60709b94f52cb4ae917dfd`
- Preserved model: 66 links, 65 joints, 40 revolute DOFs, 94 mesh references; no model file is changed by this candidate.

## Installed API signal

The installed file
`C:\isaacsim\exts\isaacsim.sensors.experimental.physics\isaacsim\sensors\experimental\physics\impl\contact_sensor.py`
defines `ContactSensor.get_raw_data()` as raw dictionaries containing
`body0`, `body1`, `position`, `normal`, `impulse`, `time`, and `dt`. Its own
`get_data()` implementation resolves integer body handles with
`pxr.PhysicsSchemaTools.intToSdfPath`.

`Isaac61GraspFeedbackAdapter` uses the same resolver. It accepts only exact
paths in a validated `IsaacContactBindings` object:

- imported rigid-body paths for `right_thumb_ip` and `right_thumb_dp`;
- imported rigid-body paths for `right_index_ip`, `right_middle_ip`,
  `right_ring_ip`, `right_pinky_ip`, and `right_palm`;
- `/World/Restocking/Product` and its authored collider;
- `/World/Restocking/Pickup/Support`.

Fixed `*_fingertip` marker paths, unknown bodies, collapsed body pairs, a
contact with no product side, and a product/product pair raise
`FeedbackUnavailableError`. The semantic `robot_link_name` is derived from
the exact resolved body path; no raw label is trusted.
Raw body handles must be integer objects; booleans, floating point values,
including integral-looking values, nonfinite values, and strings are rejected
before the installed path resolver is called. Every raw record must also carry
a finite world contact position and a finite nonzero contact normal. Missing or
malformed geometry invalidates the full observation rather than contributing
contact evidence.

Each raw impulse magnitude is divided by that record's positive measured
`dt`. Multiple contact points for one exact pair are summed. The aggregate
sensor force is retained only as a diagnostic. The sensor is authored with a
zero reporting threshold and radius filtering disabled, so the reviewed
controller's `minimum_contact_force_n` remains the sole grasp threshold.

## Pose, freshness, and lift

The adapter reads named arm/hand positions, product velocity, product/palm
world poses, and all five fingertip-marker world poses. Rigid bodies use
`RigidPrim`; the collision-free fingertip markers use `XformPrim` only for
geometry diagnostics. Isaac `wxyz` orientations are
normalized and converted to the controller's `xyzw` convention. Simulation,
aggregate contact, and every raw contact timestamp must be finite and fresh;
simulation time must increase strictly.

`IsaacArmLiftPort` reads the six named R3 arm DOFs, computes a bounded vertical
right-palm goal with the reviewed production kinematics, and queues bounded
joint-space waypoints. `advance()` sends only those six named targets through
`ArticulationController.command_joint_positions`. It refuses a second active
request and stops issuing targets after the requested deadline. Acceptance or
command completion is never treated as lift evidence. The R4 controller still
requires the product to leave the exact pickup support, rise by the verified
distance, retain opposing physical contact, and remain secured relative to the
measured palm pose before reporting completion.

The hand continues to command only the 16 reviewed OrcaHand finger DOFs.
`right_wrist` stays in the R3 arm set. The adapter and harness contain no
attachment constraint, product transform setter, velocity setter, kinematic
toggle, weld, teleport, or state-derived fake contact.

## Smoke harness and evidence

`run_grasp_smoke.py` delays every Isaac import until `run_isaac`. At that
boundary it imports the unchanged production URDF, selects its PhysX variant,
authors the existing aisle/restocking fixture and collision-enabled nonkinematic
pasta box, locates the single full articulation, creates a product-parented
contact sensor, and performs one explicit robot reset before the runner starts.
For this isolated R4 smoke, that sole reset places the stationary base at a
deterministic pickup pose derived from the layout product pose and R3 forward
kinematics. A bounded R3 arm approach must reach its measured palm target for
three samples before the finger controller starts. All later robot motion uses
named drive targets; the base remains stationary and all product motion is
physics.

The runner has a physics-step bound and writes an atomically replaced,
file-flushed status document before motion, after every sample, and at the
terminal result. The report contains candidate/model identity, exact bound
paths, sensor setup, the state-write policy, controller phase/counters,
adapter contact evidence, and durable error text on failure.
The command line requires the expected candidate commit and tree, hashes the
exact owner acceptance specification before SimulationApp starts, and refuses
a dirty or mismatched checkout.
The first durable receipt is written before the Isaac import and contains both
expected and observed commit, tree, acceptance-specification, combined-source
URDF, and production-URDF identities. A failed preflight preserves that full
receipt when adding terminal error details.
It also binds the byte hashes and resolved paths for
`config/scenarios/baseline_straight.yaml` and production `robot_config.json`,
and reads the exact installed build string
`6.1.0-rc.26+release.49347.2d230af4.gl` from `C:\isaacsim\VERSION` before
SimulationApp starts. After stage binding, the runtime preflight adds all seven
required link-to-rigid-body mappings and the complete product, collider, support,
and robot-body allowlist. That receipt is flushed before the explicit reset.
Each durable write uses a new same-directory temporary file and replaces the
status atomically; it never reads or merges a prior status or fixed `.tmp` file.

## First serialized setup result

Exact reviewed integration `1794b3b1dd28508cb358780b76139a94a1f702c9`
(tree `a8b7413550bfa861171f3067ff6423e4b0d1cd82`) passed its complete identity
preflight with zero mismatches. During stage setup, USD 25.11 rejected the
product orientation value because the default `AddOrientOp()` attribute was
`GfQuatf` while the builder supplied `GfQuatd`. The durable failure receipt is
`R4-grasp-1794b3b/grasp_smoke_status.json`, and the installed Kit log is
`kit_20260926_063145.log`. No reset, contact read, grasp command, or lift command
occurred.

The successor authors the unchanged source reset pose using `Gf.Quatf` with a
`Gf.Vec3f` imaginary component, matching the default orient op's value type.
Translation remains the existing `Gf.Vec3d`; rigid-body, nonkinematic, mass,
collision, material, and initialization-only reset-policy authoring are
unchanged. A CPU typed-op regression exercises this exact pose-authoring path.
The correction has not been rerun in Isaac pending fresh source review.

## First contact-sequence result

Exact reviewed integration `ff0ed920113b9b23ce82bb05242a835b750774fd`
(tree `c67f784624b4b76956dda1d08cee41d75456a5e7`) passed identity and typed
orientation setup. Its receipt is `R4-grasp-ff0ed92/grasp_smoke_status.json`;
the Kit log is `kit_20260926_064742.log`. It completed 23 preshape samples and
120 close samples, then failed `contact_not_verified` at step 143. All 144
samples reported only four product/support contact points, about 3.44 N at the
end, with no hand link, no lift request, and no physical grasp evidence.

The trace did not contain geometry, but the source identity proves the cause
of the failed setup: the robot reset root was `(0, 0, 0.635)` while the product
was at `(-4.35, 0.96, 0.9175)`, and the old runner issued no arm approach before
finger closure. The successor's pickup reset keeps the configured root height,
places the base outside the pickup board, and uses the unchanged dynamic
product pose. Per-sample evidence now includes product pose/velocity, palm and
fingertip poses, named arm/hand measurements and active-target errors, R3 tool
target/error, and every resolved raw contact body path and geometry record.
Missing contact still aborts; no attachment, product state write, or synthetic
contact was added. The prior runtime fail receipt accompanied process exit 0,
so the entry point now flushes streams and uses the established hard-exit path
to propagate its computed 0/1 result through `python.bat`. No further Isaac run
occurred before fresh review.

## First measured-approach adapter failure

Exact reviewed integration `e95721e641ea6a82ee53b4e49f118fe0574a0df4`
(tree `6212d73543a4ea3d80571dd6806a61bf274288a3`) passed identity
preflight, setup, and reset. Its durable receipt is
`R4-grasp-e95721e/grasp_smoke_status.json`; the Kit log is
`kit_20260926_073835.log`. The first approach sample at step 0 recorded four
physical product/support contacts with aggregate force about 3.455 N, zero
approach confirmations, and no finger or lift command. It then failed closed
with `arm approach measurement failed: product velocity must have shape (1,
6)`.

The installed Isaac 6.1 experimental `RigidPrim.get_velocities()` contract in
`isaacsim.core.experimental.prims/.../impl/rigid_prim.py` returns a tuple of
linear and angular arrays, each shape `(N, 3)`. The inspected installed file has
SHA-256
`4b3da8b14df262c09ef1e19b4deef67ad9e29525255d2463d4e17808ce93bb67`.
The adapter had incorrectly treated that tuple as one `(1, 6)` array. The
source correction reads both measured arrays, requires exactly one finite
three-component row from each, and concatenates them only for diagnostics. It
does not substitute zeros, set velocity, move the product, relax contact
checks, or change the approach/grasp controller. CPU tests cover the installed
split return shape and reject the old combined shape, malformed rows, and
nonfinite components. No further Isaac, PhysX, or GPU run occurred before
fresh exact review.

## First measured arm-approach result

Exact reviewed candidate `ec103a233ecbf4565a011ca694b47769f395fb7d`
(tree `2bc70ae328453665a5188deb53d3084f2e3e6f82`) exited 1 after 361
measured approach samples. Its receipt is
`R4-grasp-ec103a2/grasp_smoke_status.json`; the Kit log is
`kit_20260926_074838.log`. All waypoints had been issued by sample 80, but the
following 280 samples plateaued. The final R3 position/orientation errors were
0.352272604 m and 1.103640156 rad against unchanged 0.015 m and 0.08 rad
tolerances. There were zero confirmations, finger motion never started, and no
lift was requested.

The trace proves name-bound drive commands worked but the folded pose did not
physically track. Its final elbow target was -1.558326398 rad while the measured
joint remained at -0.720163405 rad, an error of 0.838162993 rad. Shoulder roll
retained 0.298301377 rad error. Yaw and wrist joints tracked much more closely.
The run's derived `physics.usda` has SHA-256
`3f301c7b4af5679dcd4052b613e3a630cbf1f5cb95ba391ccd62b8d2d24e2bca`;
it preserves the production URDF maximum drive forces of 12 Nm at the elbow
and 25 Nm at shoulder roll. The target therefore concentrated the horizontal
hold on the weakest loaded arm joint, and extra deadline time did not reduce
the measured error.

The bounded correction changes only the R3 pregrasp configuration. It uses a
nearly straight -0.2 rad elbow and shifts the reach to the 30 Nm shoulder pitch
joint. The R3 solver reaches the resulting tool target from zero with a solved
elbow magnitude below 0.25 rad. The deterministic reset keeps the base outside
the pickup board, and the predicted palm remains within the product's vertical
extent and one half-width from its horizontal center. URDF effort, velocity,
and joint limits, importer drives, controller tolerances/deadline, dynamic
product, contact gates, and target-only command path are unchanged. CPU
regression includes the exact measured ec103 plateau and the new joint/geometry
bounds. No further Isaac, PhysX, or GPU run occurred before fresh exact review.

## Source-only validation limits

CPU fakes cover path resolution, impulse conversion, orientation conversion,
freshness, invalid data, collapsed/marker paths, the controller's opposing
contact sequence, support departure, measured lift, and durable failure
records. They do not establish real PhysX collision reporting, imported drive
tuning, contact stability, IK reachability from the actual pickup stance, or
that the physical box can be lifted. Those remain fail-closed runtime gates.
No Isaac, SimulationApp, PhysX, or GPU process was started for this candidate.
