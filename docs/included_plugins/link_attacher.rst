.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Link Attacher
=============

``LinkAttacher`` joins another model to the robot with a runtime Box2D weld.
It holds the current relative pose, suppresses collisions between the two
models while attached, and can make the payload's visual elevation follow a
fork body. Detaching removes the weld and lets a raised object's elevation
settle toward the floor.

Configuration
-------------

.. code-block:: yaml

  plugins:
    - type: LinkAttacher
      name: payload_attacher
      body: base_link                 # required, existing attachment body
      model_prefixes: [pallet, box]    # required, nonempty list for auto-selection
      capture_point: [0.8, 0.0]       # optional, default [0, 0], body-local x/y, m
      capture_range: 0.5              # optional, default 1.0 m
      attach_service: fork/attach     # optional, default attach
      detach_service: fork/detach     # optional, default detach
      state_topic: fork/attached_model # optional, default attached_model
      elevation_source_body: fork_left # optional, default "" (no height following)
      carry_elevation_offset: 0.0     # optional, default 0.0 m
      drop_speed: 0.5                 # optional, default 0.5 m/s
      update_rate: 10.0               # optional, default 10.0 Hz, state output

The named attachment and elevation-source bodies belong to this model. Use
a positive ``capture_range`` and ``drop_speed``. A zero drop speed holds a
released object's height; a negative value would increase it. Use dynamic,
rigidly connected payload models with a representative first body near the
capture point. The final range check and weld use the payload's first body.

Services and state
------------------

``attach_service`` uses ``flatland_msgs/srv/Attach``:

* ``model_name: ''`` selects the nearest candidate matching a configured name
  prefix and lying within ``capture_range`` of the transformed capture point.
* A nonempty ``model_name`` selects that model explicitly. It must still pass
  the capture-range check, but **does not need to match** ``model_prefixes``.
* Only one model can be attached at a time. Requests fail if a load is already
  attached, the target cannot be found or is out of range, or the physics
  world is currently locked.
* The response contains ``bool success`` and ``string message``.

``detach_service`` uses ``std_srvs/srv/Trigger`` with an empty request and
returns success/message. ``state_topic`` publishes ``std_msgs/msg/String``:
the attached model's name, or an empty string when nothing is attached.
Relative names inherit the robot namespace.

For the unnamespaced configuration above:

.. code-block:: bash

  ros2 service call /fork/attach flatland_msgs/srv/Attach "{model_name: ''}"
  ros2 topic echo /fork/attached_model
  ros2 service call /fork/detach std_srvs/srv/Trigger '{}'

Behavior and limitations
------------------------

Attachment does not snap the object to the capture point: the weld preserves
its relative position and orientation at attachment time. Arrange the robot
and load before calling the service. Prefix-based discovery and the final
first-body check can differ for multi-body models, so single-body rigid loads
are the simplest candidates.

When ``elevation_source_body`` is set, every payload body receives that body's
elevation plus ``carry_elevation_offset`` each physics step. On release, height
falls linearly toward zero at ``drop_speed``. This is an animation, not gravity
or a collision-aware 3D drop. Only one released model settles at a time; a
previously settling model is grounded when a new one takes its place.

The plugin restores collision group zero when detaching; it does not remember
arbitrary pre-existing fixture group indices. It handles deletion of an
attached target by clearing its attachment state. Keep one instance per model
when using :doc:`fork_controller`. Combine with :doc:`forklift` to animate forks.
