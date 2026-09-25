# Dynamic camera transform consumer contract

RGB/LiDAR association and technical current-scan projection support the articulated
camera head recorded by the M3 capture runtime. The base pose remains the corrected
`map -> sensor_rig` pose. LiDAR remains fixed under `sensor_rig`. The camera chain is:

`sensor_rig -> camera_link -> camera_optical_frame`

`sensor_transforms.json` may contain one `dynamic_transform_artifacts` declaration
for `sensor_rig -> camera_link`. The declaration binds
`camera_head_transforms.json` by canonical relative path, byte size, SHA-256, and
schema version. The same path, size, and SHA-256 must occur exactly once in
`capture_manifest.json`. Captures without that declaration retain the historical
static transform behavior.

The static and dynamic graph edges are disjoint. A dynamic capture must omit the
static `sensor_rig -> camera_link` row while retaining the static
`camera_link -> camera_optical_frame` row. A dynamic declaration combined with the
same static edge is rejected. A legacy capture without a declaration must provide a
complete static path from the LiDAR through the rig to the optical frame.

For a dynamic capture, consumers independently verify the artifact bytes, schema,
frames, units, interpolation policy, cadence, closed time range, source provenance,
unit quaternions, and the static optical-child transform. Sample timestamps must be
finite, unique, and strictly increasing. Every RGB timestamp must match an exported
head sample within 1 microsecond. Missing files, stale hashes, duplicate declarations,
nonmonotonic samples, out-of-range queries, and frame-chain mismatches fail closed.

At each selected RGB image timestamp, projection first moves the LiDAR return from
the scan-time rig pose into the corrected map frame, then into the image-time rig
pose. It composes that result with the artifact's image-time
`sensor_rig -> camera_link` transform and the static
`camera_link -> camera_optical_frame` transform. The artifact's quaternion already
contains the configured mount rotation multiplied by the head articulation; consumers
do not apply the mount twice. Technical projection and selection caches include both
the raw scan timestamp and selected RGB timestamp, retain original return indices,
and remain bounded.

Perception manifests expose the verified camera-transform receipt under
`localization.camera_transform_provenance`. Technical view derivations expose the
same binding as `camera_head_transform`. The renderer rehashes the artifact before
publishing its manifest, and the presentation validator resolves the catalog-bound
`sensor_transforms.json`, reloads the referenced artifact, recomputes its size and
SHA-256, cross-checks the capture-manifest file entry, and compares the resulting
receipt with every view receipt. The RGB role emitter and RGB bundle validator apply
the same recomputation so an articulated capture cannot produce an accepted RGB role
after its transform artifact is removed or replaced.
