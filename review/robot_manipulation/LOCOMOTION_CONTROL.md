# RST-003 / R2 locomotion control candidate

Date: 2026-09-26

Status: **CPU code candidate only; scheduled Isaac validation is pending and the R2 gate has not passed**

## Routing and immutable base

- Task: `RST-003`, R2 short-path biped locomotion.
- Assigned implementation worktree:
  `C:\Users\suyog\.codex\worktrees\restock-r2-locomotion\Simulation`.
- Assigned base commit:
  `1fe5ac5c42466b3ea581b73585e919e0a8e32117`.
- Assigned base tree:
  `88c52e464ab2542ba5e3eb854b00064719f659c7`.
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

The runtime integration owner must inject a `LocomotionFeedbackSource` that
reads the actual Isaac/PhysX articulation and contact sensors. The module does
not guess Isaac API method names for link pose, contact, COM, or velocity reads.
This keeps version-specific physics glue explicit at the integration boundary.

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

## Scheduled Isaac validation requirements

No Isaac, PhysX, or GPU process is launched by this task. R2 remains pending
until the orchestrator schedules one heavy job and an independent reviewer
checks the exact integrated candidate. That validation must at minimum:

1. import and settle the exact 40-DOF free-root production articulation;
2. bind the feedback adapter to measured root state, bilateral sole/contact
   state, leg `q`/`qd`, and a defensible COM/support polygon calculation;
3. verify the foot reference frame and contact target height used by the plan;
4. tune articulation drives without adding root-state writes;
5. walk a short collision-free path at the bounded speed and step length;
6. show alternating physical swing, contact-gated touchdown, stop, and dock;
7. record falls, slips, timeouts, tracking error, actual root motion, and exact
   source/config identities rather than treating a command trace as motion;
8. repeat reset and the short-path run to check reproducibility.

Runtime API details for link/sole pose, contact reporting, COM/support polygon,
and the Isaac Sim 6.1 articulation readback shape are deliberately unresolved
here. They must be verified against the installed runtime during the scheduled
integration, not invented in CPU-only code.
