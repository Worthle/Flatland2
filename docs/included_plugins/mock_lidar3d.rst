.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Mock 3D Lidar
=============

``MockLidar3D`` publishes ``sensor_msgs/msg/PointCloud2`` for localization and
perception experiments around planar Box2D motion. It supports two modes:

* **PCD sampling:** a configured point-cloud map supplies static geometry.
  Samples are selected by azimuth bin and beam elevation, then combined with
  approximate returns from model footprints and their height spans.
* **Extrusion:** without a PCD map, 2D ray hits are repeated at configured
  sensor-relative heights. An optional synthetic ceiling adds points above
  the sensor.

The cloud contains float32 ``x``, ``y``, ``z`` and ``intensity`` fields in the
sensor frame. Intensity is a constant 100; there are no ring IDs or per-point
timestamps. The sensor faces +X, with yaw about +Z.

Common configuration
--------------------

.. code-block:: yaml

  plugins:
    - type: MockLidar3D
      name: lidar3d
      body: base_link                  # required
      # Optional; parser default is ABSOLUTE /lidar_points.
      # This relative value follows the model namespace.
      topic: lidar_points
      frame: lidar3d                  # optional, default plugin name
      broadcast_tf: true              # optional, default true
      origin: [0.0, 0.0, 0.0]         # optional, body-local x/y/yaw; default zeros
      origin_z: 1.0                   # optional, default 1.0 m
      update_rate: 10.0               # optional, default 10.0 Hz
      min_range: 0.5                  # optional, default 0.5 m
      max_range: 50.0                 # optional, default 50.0 m
      num_rays: 720                   # optional, default 720, full-circle azimuth bins
      range_noise_std_dev: 0.01       # optional, default 0.01 m
      layers: [all]                   # optional, default [all], Box2D ray filtering

      # Optional; default "" selects extrusion mode.
      pcd_path: package://flatland_examples/maps/room/room.pcd
      sampling_threads: 0             # optional; 0 automatic, or 1..64 workers
      elevations_deg: [-15, -12, -9, -6, -3, 0, 3, 6, 9, 12, 15, 18, 21, 24, 27, 30]
      elev_tolerance_deg: 1.4         # optional, default 1.4 degrees
      default_object_height: 0.3      # optional, default 0.3 m

The listed elevation sequence is the default. Use at least eight azimuth bins,
``max_range > min_range`` and nonnegative range-noise standard deviation.
``sampling_threads: 0`` selects up to eight workers based on hardware
concurrency. Use ``1`` to request serial PCD sampling.

PCD sampling details
--------------------

``pcd_path`` accepts a filesystem path or an installed
``package://package/path`` URI. A relative filesystem path is relative to the
server process's working directory, not the model YAML. Prefer a package URI
for portable models. The loader expects a well-formed ASCII or uncompressed
binary PCD with XYZ as the first three fields. For binary input, use one
float32 value per field; compressed PCD and arbitrary PCD field layouts are
not supported. The companion examples package's room PCD is a compatible example.

The map must use the simulation's world coordinates and metres; the plugin
does not transform it through TF. Sampling uses ``origin_z`` as the sensor's
world Z and does not add the parent body's elevation. The mount TF also uses
``origin_z`` as its local Z offset. Mount this sensor on an unelevated base
body to keep those conventions consistent.

The closest available return is kept for each azimuth/elevation bin. Static
PCD points are unaffected by ``layers``; that filter applies to Box2D rays.
Model returns use body elevation through elevation plus extrusion height,
falling back to ``default_object_height`` when extrusion height is zero.
This is a footprint/height approximation, not arbitrary 3D mesh ray tracing.

Extrusion configuration
-----------------------

For extrusion mode, omit ``pcd_path`` or set it to an empty string and add the
following optional keys inside the same plugin entry:

.. code-block:: yaml

  pcd_path: ""
  heights: [2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 6.0] # default, sensor-relative Z, m
  ceiling_height: 0.0        # default, sensor-relative Z, m; <= 0 disables ceiling
  ceiling_max_range: 10.0    # default m
  ceiling_ring_step: 1.0     # default m; must be positive for ceiling generation
  ceiling_azimuths: 36       # default count; use a positive integer

These heights are written directly into cloud Z coordinates; they are not
world elevations and ``origin_z`` is not subtracted from them. Extruded hits
are produced at every listed height regardless of real object height. Ceiling
points are synthetic rings and are not occlusion-tested. Elevation-beam
settings and PCD sampling workers do not affect this mode.

ROS interfaces and timing
-------------------------

The publisher uses ROS sensor-data QoS: best effort, volatile, depth one.
Configure subscribers accordingly. The plugin skips cloud generation when
there are no subscribers. Body-to-sensor TF follows ``broadcast_tf`` and the
update timer even without a cloud subscriber.

PCD scans snapshot the sensor and dynamic geometry at capture time and complete
asynchronously. The output stamp is that capture time. A busy scan skips a
later capture instead of queuing scans or blocking physics, so actual output
rate can fall below ``update_rate``. Extrusion mode publishes synchronously.
There is no rolling-scan motion distortion, full material response or 3D
vehicle dynamics in either mode.
