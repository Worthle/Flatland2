Forklift
========

``Forklift`` controls the elevation and appearance of existing fork bodies.
It accepts a fractional lift target, animates the bodies' display height, and
publishes commanded lift state and current height. It does not drive the
vehicle or attach a payload: use a drive plugin and :doc:`link_attacher` for
those tasks. :doc:`fork_controller` adds absolute-height goals and load feedback.

Configuration
-------------

.. code-block:: yaml

  plugins:
    - type: Forklift
      name: fork_lift
      bodies: [fork_left, fork_right] # required, one or more existing bodies
      lift_height: 1.0               # optional, default 0.2 m, maximum elevation
      fork_thickness: 0.1            # optional, default 0.1 m, visual extrusion
      lift_speed: 0.25               # optional, default 0.0 m/s (instant)
      initial_lifted: false          # optional, default false
      lowered_color: [1.0, 0.0, 0.0, 0.75] # optional, default RGBA
      lifted_color: [0.0, 1.0, 0.0, 0.75]  # optional, default RGBA
      service: fork/lift             # optional, default fork/lift
      state_topic: fork/state        # optional, default fork/state
      height_topic: fork/height      # optional, default fork/height
      update_rate: 20.0              # optional, default .inf, Hz

Use nonnegative ``lift_height`` and ``fork_thickness``. A positive
``lift_speed`` moves toward the target at that speed on each simulation step;
a zero or negative value changes height immediately. At startup,
``initial_lifted: true`` starts at ``lift_height`` without animation. Otherwise
the forks start at zero elevation. Colors interpolate according to current
height. ``update_rate`` controls periodic state output, not animation speed.

ROS interfaces
--------------

* ``service`` is ``flatland_msgs/srv/LiftFork``. Its request contains
  ``float64 fraction`` in [0, 1], where zero lowers completely and one commands
  ``lift_height``. Out-of-range fractions are rejected. The response contains
  ``bool success`` and ``string message``; success means the target was
  accepted, not that animation has finished.
* ``state_topic`` is ``std_msgs/msg/Bool``. It is true when the **target** height
  is above zero. It is not a measured end-of-travel or payload-presence signal.
* ``height_topic`` is ``std_msgs/msg/Float32`` with the current elevation in m.

Relative topic and service names inherit the model namespace. For an
unnamespaced model with the configuration above:

.. code-block:: bash

  ros2 service call /fork/lift flatland_msgs/srv/LiftFork '{fraction: 0.5}'
  ros2 topic echo /fork/height

The example commands 0.5 m because its configured maximum is 1.0 m. Use
``ForkController`` when the client should command metres directly or needs a
``fork/goal_reached`` signal. Keep one ``Forklift`` instance per model when
using sibling fork plugins.

Simulation scope
----------------

The plugin changes body elevation, extrusion and color; it does not create a
vertical Box2D joint or add vertical collision physics. Planar footprints
remain present. Sensor plugins may use the elevation metadata for filtering,
and ``LinkAttacher`` can copy it to a carried object. The bundled forklift
model demonstrates the three cooperating fork plugins.
