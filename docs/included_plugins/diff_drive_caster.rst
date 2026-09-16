Diff Drive Caster
=================

``DiffDriveCaster`` combines differential-drive commands with approximate
passive caster alignment and contact impulses. It reads forward speed from
``geometry_msgs/msg/Twist.linear.x`` and yaw rate from ``angular.z``; lateral
commands are ignored. Misaligned casters can introduce sideways motion or yaw
even when the command requests straight travel.

Each configured caster has a local +X contact offset, set by ``trail``. The
plugin applies lateral and rolling-resistance impulses to the drive body at
that contact, then rotates the caster toward a trailing orientation. This
is a tunable planar approximation of caster behavior.

Configuration
-------------

Define the base and caster bodies and connect the casters with suitable free
revolute joints before loading the plugin. Use a dynamic base with nonzero
mass and rotational inertia. The plugin resolves existing bodies; it does
not create wheels or joints. A full example is
``flatland_examples/robot/castor_wheeled_differential_robot.model.yaml``.

.. code-block:: yaml

  plugins:
    - type: DiffDriveCaster
      name: caster_drive
      body: base_link                 # required
      twist_sub: cmd_vel              # optional, default cmd_vel

      # Optional, default [] (no caster constraints).
      casters:
        - body: caster_left           # required within each caster entry
          trail: 0.05                 # optional, default 0.05 m, along local +X
        - body: caster_right
          trail: 0.05

      # Optional, default 0.0: 0 removes lateral slip, 1 allows it fully.
      caster_lat_mu: 0.15
      # Optional, default 0.0: 0 has no rolling resistance, 1 removes it fully.
      caster_long_mu: 0.02
      # Optional, default 8.0; positive finite alignment gain, 1/s.
      caster_alignment_rate: 8.0

      # Optional dynamics maps; zero disables each limit.
      # Omitted deceleration_limit inherits acceleration_limit.
      linear_dynamics:                 # m/s and m/s^2
        velocity_limit: 1.0
        acceleration_limit: 0.5
      angular_dynamics:                # rad/s and rad/s^2
        velocity_limit: 1.0
        acceleration_limit: 1.5

      pub_rate: 30.0                  # optional, default .inf, Hz
      enable_odom_pub: true            # optional, default true
      enable_odom_tf_pub: true         # optional, default true
      enable_twist_pub: true           # optional, default true
      twist_in_local_frame: true      # optional, default true

The two ``caster_*_mu`` values are clamped to [0, 1] when used. They are
velocity-removal factors, not Coulomb friction coefficients. Alignment is
forced only when the contact speed exceeds 0.05 m/s; below that speed the
plugin sets the caster angular velocity to zero.

ROS interfaces and behavior
---------------------------

* Subscribes to ``twist_sub`` as ``geometry_msgs/msg/Twist``.
* Publishes noisy odometry, absolute ground-truth odometry and pose, and an
  optional ``geometry_msgs/msg/TwistWithCovarianceStamped`` velocity estimate.
* Can broadcast ``odom`` to the namespaced drive-body frame. Disable
  ``enable_odom_tf_pub`` when another node publishes that transform.

See :doc:`drive_options` for all topic, frame, noise and covariance defaults.
The last command persists until replaced; there is no timeout. The plugin
sets commanded base velocity before applying caster impulses on each step,
so it should be the only propulsion plugin on that body. It does not support
``DiffDrive``'s ``use_id_model`` or OE-model configuration keys.
