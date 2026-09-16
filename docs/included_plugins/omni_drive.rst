.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Omni Drive
==========

``OmniDrive`` drives a planar body using two independently steered wheel
assemblies (turrets). Each turret receives an
``ackermann_msgs/msg/AckermannDriveStamped`` message: ``drive.speed`` is signed
wheel speed in m/s and ``drive.steering_angle`` is in radians relative to body
+X. Other command fields are ignored. Matching lateral wheel velocities allow
sideways travel; differences between turret velocities produce rotation.

The plugin solves the body velocity from the turret locations, limits each
wheel's speed and steering response, and updates steering joints for
visualization. Optional passive caster joints follow the local direction of
travel. Use distinct turret positions, preferably a front/rear pair separated
along body X as in ``flatland_examples/robot/omni_drive_robot.model.yaml``.

Configuration
-------------

Both turret joints must be revolute joints attached to ``body``. Optional
caster joints must be separate revolute joints attached to that same body;
the plugin disables their limits and motors. It does not create the joints.

.. code-block:: yaml

  plugins:
    - type: OmniDrive
      name: omni_drive
      body: base_link                   # required
      turret1_joint: front_turret_joint  # required
      turret2_joint: rear_turret_joint   # required
      caster_joints: []                 # optional, default []
      caster_alignment_rate: 8.0        # optional, positive finite 1/s

      # Optional command topics; defaults are ackermann_cmd_1 and _2.
      turret1_sub: drive/turret1/command
      turret2_sub: drive/turret2/command

      # Optional, default 0.0 (unbounded), radians; shared by both turrets.
      max_steer_angle: 0.0

      # Optional maps; zero disables a limit. An omitted deceleration
      # limit inherits acceleration_limit.
      turret1_linear_dynamics:          # m/s and m/s^2
        velocity_limit: 1.0
        acceleration_limit: 0.5
      turret2_linear_dynamics:
        velocity_limit: 1.0
        acceleration_limit: 0.5
      turret1_steering_dynamics:        # steering rad/s and rad/s^2
        velocity_limit: 1.5
        acceleration_limit: 3.0
      turret2_steering_dynamics:
        velocity_limit: 1.5
        acceleration_limit: 3.0

      # Optional feedback topics, shown with their defaults.
      turret_angles_pub: turret_angles/measured
      wrpms_pub: drive/wheel_speeds
      turret1_cmd_pub: drive/turret1/measured
      turret2_cmd_pub: drive/turret2/measured

      pub_rate: 30.0                    # optional, default .inf, Hz
      enable_odom_pub: true              # optional, default true
      enable_odom_tf_pub: true           # optional, default true
      enable_twist_pub: true             # optional, default true
      twist_in_local_frame: true        # optional, default true

ROS interfaces and behavior
---------------------------

* The two command topics accept ``ackermann_msgs/msg/AckermannDriveStamped``.
  ``OmniDrive`` itself does not subscribe to ``cmd_vel``. The companion package's
  ``omni_command.py`` node converts ``Twist`` commands into turret commands.
* ``turret_angles_pub`` publishes ``flatland_msgs/msg/ChannelValuesFloating``
  with ``value: [turret1_angle, turret2_angle]`` in radians.
* ``wrpms_pub`` uses the same message with
  ``value: [turret1_speed, turret2_speed]`` in **m/s**, despite the configuration
  key's historical RPM wording. It does not convert speed to wheel RPM.
* ``turret1_cmd_pub`` and ``turret2_cmd_pub`` publish measured, limited turret
  state as ``ackermann_msgs/msg/AckermannDriveStamped``.
* Turret feedback is published every physics step. ``pub_rate`` controls the
  odometry, ground-truth, twist and odometry-TF updates, not turret feedback.

See :doc:`drive_options` for shared odometry topics, frames and noise options.
Commands persist independently on each turret until replaced; there is no
command timeout. Send stop commands to both turrets. Wheel velocities are
converted directly to body motion, so incompatible wheel commands do not
produce a detailed tire-slip simulation. Use only one drive plugin per body.
