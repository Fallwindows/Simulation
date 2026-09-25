# M2 dynamic camera transform contract

## Motion ownership

The mobile `sensor_rig` and `lidar_link` remain aligned with aisle travel. The
right-shelf look is an articulated `sensor_rig -> camera_link` transform. The
rig ground-truth pose continues to describe the mobile base, not the camera.

The deterministic source is `config/trajectories/walking.yaml`, SHA-256
`c16ecf1371b215aaca753d849b00b95484a82536167cfbcb0fc0e8090d055d16`.
It samples 0 through 20.5 seconds inclusively at 30 Hz (616 samples). The two
camera-head peaks are at 9 and 17 seconds. Each has -23 degrees yaw and +10
degrees ROS camera-link pitch. The mobile base lateral offset is -0.04 m at
each peak.

The framing target for each peak is `products_and_price_rail`. The full content
bounds used by the analytic test include product fronts, neighboring facings,
the shelf lip, and the complete price rail from z=1.08 m to z=1.65 m. The
foreground support body extends down to z=0.38 m and is deliberately cropped;
this is encoded as `allow_foreground_support_crop: true` and tested directly.

## Runtime TF contract

At every simulation update the runtime evaluates one trajectory sample for the
next measured Isaac tick, applies its base pose to `/World/SensorRig`, and
applies the composed camera mount rotation to
`/World/SensorRig/camera_link`. After the update it publishes both dynamic TFs
with the post-update Isaac `/clock` timestamp:

1. `sim_world -> truth_sensor_rig`, the base ground-truth pose.
2. `sensor_rig -> camera_link`, the configured camera translation and the
   configured mount quaternion multiplied by the sampled head quaternion.

`camera_link -> camera_optical_frame` remains static with translation
`[0, 0, 0]` m and ROS xyzw rotation `[0.5, -0.5, 0.5, -0.5]`.
`sensor_rig -> lidar_link` also remains static. The dynamic camera transform is
removed from `/tf_static`.

RGB and CameraInfo use frame `camera_optical_frame`. Representative capture
metadata records the composed world pose of `camera_optical_frame`. Tests prove
that ROS optical +Z and the USD camera's rendered local -Z produce the same
world viewing ray.

## Durable replay artifact

`camera_head_transforms.json` has schema `grocery.camera_head_transforms`,
version 1. It contains:

- `frames`: parent `sensor_rig`, child `camera_link`, optical child
  `camera_optical_frame`.
- Direction `parent_to_child`; metres, seconds, and ROS xyzw quaternions.
- Timestamp domain `Isaac simulation time (/clock)`.
- Translation interpolation `linear`, rotation interpolation
  `shortest_arc_quaternion_slerp_xyzw`, and no extrapolation outside the closed
  0 through duration interval.
- Exact source trajectory path and SHA-256 plus Git commit and tree.
- The static optical child transform and 616 ordered samples.

`capture/sensor_transforms.json` discovers the artifact through:

```json
{
  "dynamic_transform_artifacts": [
    {
      "parent_frame": "sensor_rig",
      "child_frame": "camera_link",
      "path": "camera_head_transforms.json",
      "sha256": "<artifact sha256>",
      "size_bytes": 0,
      "schema_version": 1
    }
  ]
}
```

The producer fills the actual byte size and SHA-256 after writing the artifact.
Every observed RGB timestamp must match an exported head sample within 1e-6
seconds. The runtime fails the capture when that check fails and records the
observed count, matched count, tolerance, and maximum absolute error in
`camera_head_stamp_alignment`.
