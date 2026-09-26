# R4 physical grasp/lift source candidate

Status: **source and CPU contract candidate only; R4 is not passed**.

This bounded candidate closes the preserved OrcaHand V1 right hand around the
single configured `pasta_box`, asks a separately owned arm controller for one
lift, and accepts success only from co-timed physical feedback.  No Isaac,
PhysX, `SimulationApp`, or GPU run was performed for this candidate.  It does
not create a weld or attachment, set product transforms or velocities, or use
the presence of a command as grasp/lift evidence.

## Frozen inputs

- Assigned base commit: `6b297645a889c376b44969b62e3e39f471584383`
- Assigned base tree: `5f9dad7cdb94069656cd6672fc1f0d4688698c31`
- Owner specification:
  `C:\Users\suyog\Downloads\ROBOT_BACKROOM_RESTOCKING_IMPLEMENTATION_SPEC.md`,
  SHA-256 `7ea5ca5fa7558aa0a58bf94999adf545a7f6b53232fd155e2cc051cb15bcf0ad`
- OrcaHand source: `orcahand/orcahand_description` commit
  `b9b349a21ee0238c62b6cf92ae7597027867adf8`, tree
  `694a70bec9d960eecb05ae5b6746259b2e43d6f6`, source
  `v1/models/urdf/orcahand_right_extended.urdf`, SHA-256
  `4ca4f81ba73038abcf1b989f164077d0a0b88bf446e19386d888028eee78e2e2`.
- Combined spike URDF canonical SHA-256:
  `25c62dd8459721ea41e7c3325ab6d4a816e05b34daa89759297151d464be84ea`.
- Production URDF canonical SHA-256:
  `b605c9a54f4a8494d333fd7cc5a20de3b4c76ffc83e33bd8027e6bafb7a1f59c`.
- Restocking layout Git blob:
  `b759d0e3c673a7d821d7e7e5b89977bd2846b7a0`.
- Retail manifest Git blob:
  `4ea4ab8cd9b39df6e74f670bdcffd0e88f280e01`.
- Baseline scenario SHA-256:
  `12d41a428dfe0e82da60584da9595b1c74090b33f375cc8be60b237941d8a5d0`.

## Actual hand joint contract

The controller sends only these 16 revolute finger DOFs through
`ArticulationController.command_joint_positions`.  `right_wrist` is excluded
because R3 assigns it to the arm kinematic chain.  All axes and limits below
come from the complete preserved production URDF and use radians.

| Joint | Axis | Lower | Upper |
|---|---:|---:|---:|
| `right_thumb_mcp` | `0 0 -1` | -0.87266 | 0.87266 |
| `right_thumb_abd` | `0 0.3420208591 0.9396923603` | -1.08211 | 0.0 |
| `right_thumb_pip` | `0 -1 0` | -0.7944 | 1.23 |
| `right_thumb_dip` | `0 -1 0` | -0.85384 | 1.45 |
| `right_index_abd` | `0 0 -1` | -1.04577 | 0.24577 |
| `right_index_mcp` | `0 -1 0` | -0.34907 | 1.65806 |
| `right_index_pip` | `0 -1 0` | -0.34907 | 1.88496 |
| `right_middle_abd` | `0 0 -1` | -0.64577 | 0.64577 |
| `right_middle_mcp` | `0 -1 0` | -0.34907 | 1.58825 |
| `right_middle_pip` | `0 -1 0` | -0.34907 | 1.8675 |
| `right_ring_abd` | `0 0 -1` | -0.47577 | 0.80577 |
| `right_ring_mcp` | `0 -1 0` | -0.34907 | 1.58825 |
| `right_ring_pip` | `0 -1 0` | -0.34907 | 1.8675 |
| `right_pinky_abd` | `0 0 -1` | -0.12244 | 1.1691 |
| `right_pinky_mcp` | `0 -1 0` | -0.34907 | 1.71042 |
| `right_pinky_pip` | `0 -1 0` | -0.34907 | 1.88496 |

The configured open, preshape, and close targets stay within these limits.  The
close pose is the articulated-finger pose already used by the combined-model
CPU validation; the intermediate preshape is a bounded engineering candidate.
It is not yet proof that the 60 mm carton depth fits the physical hand in
Isaac.

## Product and physical evidence boundary

The exact first product comes from `restocking_layout.py`:

- asset `pasta_box` / catalog product `BRONZE PASTA`;
- dimensions `(0.085, 0.060, 0.275)` m;
- rigid body `/World/Restocking/Product` and collider
  `/World/Restocking/Product/Collider`;
- mass `0.35` kg, center of mass `(0, 0, 0)` m;
- assumed static/dynamic friction `0.55/0.45` and restitution `0.05`;
- source pose `(-4.35, 0.96, 0.9175)` m with xyzw orientation
  `(0, 0, 1, 0)`;
- pickup support `/World/Restocking/Pickup/Support`, surface `z=0.78` m.

The five fixed `right_*_fingertip` marker links have zero mass and no collision
geometry, so they cannot supply physical contact.  The verifier instead
requires force-bearing product contact on a collision-bearing thumb distal
link (`right_thumb_ip` or `right_thumb_dp`) and an opposing collision-bearing
distal finger (`right_index_ip`, `right_middle_ip`, `right_ring_ip`, or
`right_pinky_ip`) or `right_palm`.

The runtime must inject an exact `RobotContactBodyMap` from each of those seven
URDF link names to its normalized imported body prim path.  Every mapped path
must be unique, below the declared robot root, and end in the same exact URDF
link name.  The controller derives contact identity from that path map.  An
optional `ContactPair.robot_link_name` is only a consistency check: marker,
unmapped robot, unrelated labeled, and path/label mismatch records fail closed.
An exact product/pickup-support body pair remains authoritative even if its
optional semantic label is stale.

Before the arm lift request, consecutive observations must establish all of:

1. contact on both opposing sides against the exact product body/collider;
2. product contact with the exact pickup support;
3. product center within 20 mm of the deterministic source pose;
4. valid, monotonic timestamps and finite measured joint/body state.

After the separate arm controller accepts one 75 mm lift request, consecutive
observations must establish all of:

1. both opposing physical contacts remain above 0.05 N;
2. contact with the pickup support is absent;
3. measured product and palm height each increased by at least 50 mm;
4. product pose relative to the palm drifted at most 18 mm and 0.20 rad.

The observation that triggers the accepted lift request defines the lift start
timestamp and an inclusive four-second deadline.  Evidence at the exact
deadline can pass; any later observation fails with `lift_not_verified`, even
if its contact and displacement values would otherwise pass.

The default budgets are 120 observations per phase, three contact confirmation
samples, and three lift confirmation samples.  Missing, stale, nonfinite, wrong
product, one-sided, lost-contact, support-retained, or relative-slip evidence
cannot reach `COMPLETE`.

## Separation of ownership

`physical_grasp.py` issues finger targets only.  It never sends arm, waist,
leg, or base targets.  Its `ArmLiftPort` carries a semantic bounded lift request
to the R3 arm owner; request acceptance does not count as success.  A runtime
implementation of that port must use the reviewed arm position-target path and
must not move the product directly.

## Unresolved runtime adapter

There is deliberately no claimed Isaac 6.1 contact/pose adapter in this source
candidate.  A later allocated runtime task must prove which installed API
provides co-timed contact pairs, collision-bearing robot link identity, product
and palm world poses, and force values.  If the installed API reports impulses,
the adapter must convert impulse to force using the measured physics step.  It
must also build the exact normalized body-path map from the imported stage; a
caller-authored semantic link label cannot substitute for that binding.

Until that adapter is implemented and independently reviewed, the default
feedback source returns no observation and the sequence fails with
`feedback_unavailable`.  The R1 record also leaves hand contact adequacy and
self-collision filtering unverified.  Therefore these CPU tests validate the
state and evidence contract only; they are not physical R4 evidence.

## CPU checks

```powershell
C:\isaacsim\python.bat -m unittest `
  tests.test_robot_physical_grasp `
  tests.test_robot_arm_reach `
  tests.test_robot_production_model.ProductionRobotModelTests.test_motion_uses_drive_targets_without_direct_state_writes `
  tests.test_robot_production_model.ProductionRobotModelTests.test_joint_targets_enforce_names_finiteness_and_limits `
  tests.test_robot_production_model.ProductionRobotModelTests.test_complete_asimov_and_orcahand_tree_is_preserved
```

Result: `Ran 24 tests ... OK` on the CPU-only revision 2 candidate.  The focused
suite covers exact URDF joints/contact links and limits, finger-only
drive targets, the normal measurement-gated sequence, wrong/one-sided contact,
missing feedback, support-retained lift, stale/nonfinite observations, and
relative slip/contact loss.  Revision 2 also covers spoofed marker paths,
path/label mismatches, a mislabeled exact support pair, the inclusive lift
deadline boundary, and observations just after and far after the deadline.
Independent Sol/high review of the exact candidate is still required.

## Revision 2 finding response

- `RST-006-F01`: fixed by deriving robot-link identity only from the validated
  normalized body-path map, checking any optional label against that result,
  rejecting marker/unmapped/inconsistent robot records, and treating the exact
  product/support pair as support evidence regardless of its optional label.
- `RST-006-F02`: fixed by latching the lift request observation time, computing
  an inclusive deadline from `maximum_lift_duration_s`, and rejecting every
  later observation before evaluating its physical success evidence.
