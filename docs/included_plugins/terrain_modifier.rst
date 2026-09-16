.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Terrain Modifier
================

``TerrainModifier`` changes a model body's planar velocity while its origin
lies inside a configured world-coordinate rectangle. A patch can add slip,
lateral drift, oscillating velocity disturbances and a planar approximation
of gravity along a slope. It does not change map geometry, body elevation,
roll or pitch, and exposes no ROS topics or services of its own.

Place this plugin after the drive plugin in the model's ``plugins`` list so
its velocity changes are applied after propulsion during ``BeforePhysicsStep``.
For overlapping patches, only the first matching patch is used. Patch membership
uses the body origin, not footprint overlap.

Configuration
-------------

The following illustrative patch uses generic values. Omit a ``slip``,
``rubble`` or ``slope`` map to disable that effect. If the map is present, its
``enable`` key defaults to true.

.. code-block:: yaml

  plugins:
    - type: TerrainModifier
      name: terrain
      body: base_link                   # required
      enable: true                      # optional, default true
      seed: 7                           # optional, default 0 (random seed)
      min_speed: 0.001                   # optional, default 0.001; see below
      apply_only_if_moving: true         # accepted, default true; see below
      patches:                          # optional, default no patches
        - name: rough_patch             # optional, default patch
          box: [-2.0, -2.0, 2.0, 2.0]  # [xmin, ymin, xmax, ymax], world metres
          slip:
            enable: true
            s_max_v: 0.2                # default 0.0, max forward-speed loss fraction
            s_max_w: 0.15               # default 0.0, max yaw-rate loss fraction
            p_at_max: 0.01              # default 0.01, Gaussian clipping-tail probability
            hold_time: 0.4              # default 0.4 s between slip samples
            drift_max: 0.05             # default 0.0 m/s, lateral velocity increment
          rubble:
            enable: true
            n_waves: 3                  # default 3, summed sine components
            amp_xy: 0.02                # default 0.0 m/s, XY velocity amplitude
            amp_yaw: 0.03               # default 0.0 rad/s, yaw-rate amplitude
            f_min: 8.0                  # default 8.0 Hz
            f_max: 18.0                 # default 18.0 Hz
            speed_gain: 0.0             # default 0.0, amplitude gain from current speed
          slope:
            enable: true
            angle_deg: 3.0              # default 0.0 degrees
            downhill_heading_deg: 0.0   # default 0.0, world heading; +X is zero
            rolling_damp: 0.1           # default 0.0, exponential damping in 1/s
            v_cap: 1.0                  # default 0.0 (no slope forward-speed cap), m/s
            side_slip_gain: 0.0         # default 0.0, extra slip when facing across slope

``box`` defaults to ``[0, 0, 0, 0]`` if omitted. Its endpoints are sorted per
axis and boundaries are inclusive. Set an explicit nonzero area for useful
patches. A nonzero ``seed`` selects a repeatable random sequence for the same
simulation setup and stepping; zero uses a random-device seed.

Effect details
--------------

* Slip samples nonnegative forward/yaw loss factors, clips them at the
  configured maxima, and holds them for ``hold_time``. A nonpositive hold time
  is replaced by a simulation step. ``p_at_max`` controls the Gaussian tail
  clipped at the maximum; it is not the probability of entering a slippery
  patch. Use fractions between zero and one and a probability strictly
  between zero and one.
* Rubble adds velocity increments, not terrain displacement. Use positive
  frequencies with ``f_min <= f_max`` and positive ``n_waves``; zero waves
  produces no oscillation. ``speed_gain`` adds amplitude proportional to
  current linear speed or absolute yaw rate.
* Slope adds the forward component of ``9.81 * sin(angle)`` acceleration in
  the downhill direction, optionally damps it and caps forward speed.
  ``side_slip_gain`` scales the slip effects when the body faces across the
  slope. It does not itself implement sideways gravitational acceleration.

In the current implementation, slope is applied even at rest. Slip and rubble
are applied only when linear speed or absolute yaw rate reaches ``min_speed``
(the same numeric threshold is used for m/s and rad/s). Although
``apply_only_if_moving`` is parsed, its value is not consulted by the update
code; setting it false does not remove that gate.

Velocity modifications happen each simulation step and can depend on step
size and plugin order. Use them for controlled disturbances, not as a
calibrated terrain or tire model. The companion package's ``flatland_examples/config/terrain.yaml``
contains a simpler slip-and-rubble example used by ``terrain:=true`` in the
simulation launch file.
