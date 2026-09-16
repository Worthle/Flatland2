.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Tricycle Drive Ackermann
========================

``TricycleDriveAckermann`` drives a model with one steering wheel and two fixed
rear wheels, using ``ackermann_msgs/msg/AckermannDriveStamped`` commands.
``drive.speed`` is the steering wheel's signed rolling speed in m/s, and
``drive.steering_angle`` is its angle in radians. The other command fields are
not used. This is the Ackermann-message counterpart of :doc:`tricycle_drive`.

The plugin derives wheelbase and rear axle geometry from the model's joints.
For wheel speed ``v`` and steering angle ``delta``, the rear axle center moves
forward at ``v * cos(delta)`` and turns at ``v * sin(delta) / wheelbase``.
The configured limits shape wheel speed and steering response before the
plugin sets the body's planar velocity.

Model requirements
------------------

* ``body`` must name an existing dynamic body with nonzero mass and inertia.
* The front wheel joint must be revolute; both rear wheel joints must be welds.
  All three joints must connect a wheel to ``body``.
* Each joint anchor must be at the origin of its wheel body. The main body's
  origin may be elsewhere; anchors are converted to its local coordinates.
* Rear wheel anchors must be distinct, and the perpendicular projection of
  the front anchor onto the rear axle must be the axle midpoint. Use a
  nonzero wheelbase and align the vehicle's forward direction with body +X.

Configuration
-------------

Add this entry to the model's ``plugins`` list after defining the named bodies
and joints. The companion package's ``flatland_examples/robot/forklift.model.yaml`` provides a complete
model example; the names below are illustrative.

.. code-block:: yaml

  plugins:
    - type: TricycleDriveAckermann
      name: ackermann_drive
      body: base_link                       # required
      front_wheel_joint: front_wheel_joint   # required, revolute
      rear_left_wheel_joint: rear_left_joint # required, weld
      rear_right_wheel_joint: rear_right_joint # required, weld

      # Optional; default is cmd_vel, despite accepting Ackermann messages.
      twist_sub: ackermann_cmd

      # Optional; defaults to 0.0 (no steering-angle bound), radians.
      max_steer_angle: 0.6

      # Optional maps. Each limit defaults to 0.0 (disabled), except
      # deceleration_limit, which defaults to acceleration_limit.
      linear_dynamics:                     # wheel speed, m/s and m/s^2
        velocity_limit: 1.5
        acceleration_limit: 0.5
        deceleration_limit: 0.8
      angular_dynamics:                    # steering rate, rad/s and rad/s^2
        velocity_limit: 0.8
        acceleration_limit: 1.5
        deceleration_limit: 1.5

      # Optional; default .inf (every simulation step), Hz.
      pub_rate: 30.0
      odom_frame_id: odom                  # default odom
      ground_truth_frame_id: map           # default map
      odom_pub: odom                       # default odom
      ground_truth_pub: ground_truth/odom  # default ground_truth/odom
      ground_truth_pose_pub: ground_truth/pose # default ground_truth/pose

The legacy keys ``max_angular_velocity`` and ``max_steer_acceleration`` default
to zero and supply steering limits when their corresponding
``angular_dynamics`` limits are zero. Prefer the nested maps for new models.
See :doc:`drive_options` for the supported odometry noise and covariance keys.

ROS interfaces and behavior
---------------------------

* Subscribes to ``twist_sub`` as ``ackermann_msgs/msg/AckermannDriveStamped``.
* Publishes ``odom_pub`` and ``ground_truth_pub`` as ``nav_msgs/msg/Odometry``,
  and ``ground_truth_pose_pub`` as
  ``geometry_msgs/msg/PoseWithCovarianceStamped``.
* Does not publish odometry TF. The companion launch uses ``odometry_tf.py``;
  arrange an equivalent broadcaster when launching the server directly.
* The last command persists until replaced. There is no command timeout;
  send a zero-speed command to stop.

For the unnamespaced example above:

.. code-block:: bash

  ros2 topic pub --once /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped \
    '{drive: {speed: 0.3, steering_angle: 0.15}}'

Steering changes the wheel visualization and the velocity calculation. This
plugin does not simulate tire deformation or a full suspension. Choose one
drive plugin per body; :doc:`system_id_drive` is an alternative propulsion model.
