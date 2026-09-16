Fork Controller
===============

``ForkController`` provides absolute-height commands and simplified fork/load
feedback. It coordinates :doc:`forklift` and :doc:`link_attacher` on the same
model through their in-process interfaces. It does not replace either plugin.

A height command is clamped to [0, ``Forklift.lift_height``] and passed to the
fork animation. Optional automatic pickup and release occur when fork height
crosses configured thresholds.

Configuration
-------------

Add one ``Forklift`` and one ``LinkAttacher`` entry to the same model first;
their pages describe the required body and payload configuration. Then add:

.. code-block:: yaml

  plugins:
    - type: ForkController
      name: fork_controller

      # All keys below are optional and show their defaults.
      fork_goal_topic: fork/goal_height
      fork_pose_topic: fork/pose
      fork_goal_completed_topic: fork/goal_reached
      weight_topic: fork/weight
      contact_left_topic: fork/contact_left
      contact_right_topic: fork/contact_right
      collision_left_topic: fork/collision_left
      collision_right_topic: fork/collision_right
      load_state_topic: fork/loaded
      pose_frame: base_link

      goal_tolerance: 0.02             # m
      contact_range: 0.2               # m, proximity feedback
      loaded_weight: 300.0             # nominal kg reported while attached
      auto_attach: true
      attach_height: 0.05              # m, upward crossing triggers pickup
      detach_height: 0.02              # m, downward crossing triggers release
      update_rate: 20.0                # Hz, feedback publications

There is no ``body`` parameter. ``pose_frame`` names a TF frame, namespaced by
the model; it does not resolve a body or perform a coordinate transform. The
published pose has x/y zero and z equal to fork elevation, with identity
orientation. Configure that frame consistently with your robot model.

ROS interfaces
--------------

.. list-table::
   :header-rows: 1
   :widths: 30 33 37

   * - Configuration key
     - ROS 2 type
     - Meaning
   * - ``fork_goal_topic``
     - ``std_msgs/msg/Float32`` (input)
     - Absolute target elevation in metres
   * - ``fork_pose_topic``
     - ``geometry_msgs/msg/PoseStamped``
     - Current fork height as pose Z
   * - ``fork_goal_completed_topic``
     - ``std_msgs/msg/Bool``
     - Height within ``goal_tolerance``; true before any goal
   * - ``weight_topic``
     - ``std_msgs/msg/Float64``
     - ``loaded_weight`` when attached, otherwise zero
   * - ``contact_left_topic``, ``contact_right_topic``
     - ``std_msgs/msg/Bool``
     - The same proximity/attachment value on both topics
   * - ``collision_left_topic``, ``collision_right_topic``
     - ``std_msgs/msg/Bool``
     - Always false in this implementation
   * - ``load_state_topic``
     - ``std_msgs/msg/Bool``
     - Whether ``LinkAttacher`` currently holds a model

Relative topic names inherit the model namespace. For an unnamespaced model:

.. code-block:: bash

  ros2 topic pub --once /fork/goal_height std_msgs/msg/Float32 '{data: 0.5}'
  ros2 topic echo /fork/goal_reached

Automatic pickup and feedback limits
------------------------------------

With ``auto_attach: true``, crossing ``attach_height`` upward calls the
attacher's nearest-candidate path. Crossing ``detach_height`` downward releases
the load. Merely waiting above the pickup threshold does not retry attachment.
Choose ``0 <= detach_height < attach_height <= lift_height``. Set
``auto_attach: false`` for manual control through the attacher's services.

Contact is true while attached, or when a prefix-matching candidate lies
within both the attacher's ``capture_range`` and this plugin's ``contact_range``.
These are capture-point proximity checks, not independent left/right physical
contact sensors. Weight is a configured constant, not a computed load mass or
force measurement. Goal completion checks height only, not pickup success.

Without ``Forklift``, goals are ignored and reported height stays zero. Without
``LinkAttacher``, automatic pickup, contact and loaded feedback are unavailable.
A missing dependency is logged. The controller does not command vehicle motion
or communicate with lifting hardware.
