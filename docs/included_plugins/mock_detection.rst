Mock Detection
==============

``MockDetection`` publishes noisy poses of nearby models as
``geometry_msgs/msg/PoseArray``. It selects targets by model-name prefix,
range and planar field of view, then uses a Box2D ray to reject occluded
models. It provides a simple perception input without rendering camera images.

Poses use a planar sensor frame with +X forward, +Y left and yaw about +Z;
this is not an optical-camera frame. The target pose is transformed into that
frame after applying an optional target-local offset.

Configuration
-------------

.. code-block:: yaml

  plugins:
    - type: MockDetection
      name: detection_sensor
      body: base_link                   # required
      model_prefixes: [pallet, box]     # required, nonempty prefix list
      topic: detections                 # optional, default detections
      frame: detection_frame            # optional, defaults to plugin name
      origin: [0.2, 0.0, 0.0]           # optional, default [0, 0, 0], body x/y/yaw
      broadcast_tf: true                # optional, default true
      update_rate: 10.0                 # optional, default 10.0 Hz
      min_range: 0.3                    # optional, default 0.3 m
      max_range: 5.0                    # optional, default 5.0 m
      fov: 1.22                        # optional, default 1.22 rad, full horizontal FOV
      layers: [all]                     # optional, default [all], occluding layers
      max_target_elevation: 0.5         # optional, default .inf, maximum body elevation
      pose_offset: [0.0, 0.0, 0.0]      # optional, default [0, 0, 0], target x/y/yaw
      noise_std_dev: [0.01, 0.01, 0.02] # optional, default [0, 0, 0], m/m/rad
      dropout_probability: 0.05         # optional, default 0.0, probability per frame

      broadcast_target_tf: false       # optional, default false
      target_frame_prefix: detected_    # optional, default detected_
      world_frame: map                  # optional, default map

Use ``max_range > min_range``, positive ``fov`` in radians and a dropout
probability in [0, 1]. ``noise_std_dev`` contains standard deviations, unlike
the variance-valued noise options on the drive and IMU plugins. Supply
nonnegative values. The ``layers`` filter controls ray occlusion, not which
model prefixes are eligible.

ROS interfaces and visibility
-----------------------------

``topic`` publishes a ``PoseArray`` in the namespaced sensor frame. Relative
topic names inherit the model namespace. ``broadcast_tf`` publishes the
body-to-sensor mount transform at the configured update rate; its translation
Z is zero, and there is no ``origin_z`` parameter for this plugin.

A model is considered once using a body encountered during world traversal.
Use simple single-body targets when a precise detection origin matters.
Range and FOV checks use that body's origin before ``pose_offset`` is applied.
``max_target_elevation`` compares the body's elevation metadata, not a complete
vertical camera view. Sensor fixtures and the detecting robot's own bodies
do not occlude the ray.

No message is published when there are no detections or when a whole frame
is dropped; an empty ``PoseArray`` is not sent. Consumers must account for
stale data. The message contains neither object IDs nor class labels, and
pose order is not a stable target identifier.

Remembered target transforms
----------------------------

With ``broadcast_target_tf: true``, each successful observation updates a
world-anchored frame named ``target_frame_prefix + model_name``, additionally
namespaced by the detecting model. The frame is re-broadcast at later updates,
even when the target leaves view, a frame is dropped, or the target disappears.
It stays at its last observed noisy pose; there is no expiry timer.

``world_frame`` is used as the parent name without transforming coordinates,
so it must match the actual simulation world frame. These transforms are
last-seen observations, not continuously tracked ground truth. They are
independent of the ``broadcast_tf`` mount-transform switch.
