.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png


Diff Drive
==========
This plugin provides common features for a differential drive robots. All
velocities and odometries are w.r.t. the robot origin

* Subscribes to a topic publishing `geometry_msgs/Twist <http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html>`_
  messages, and move the robot at the desired forward and rotation velocities

* Publishes to two topics with `nav_msgs/Odometry <http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html>`_
  messages, one for robot odometry which has noise, the other the ground truth
  odometry

.. code-block:: yaml

  plugins:

      # required, specify DiffDrive type to load the plugin
    - type: DiffDrive 

      # required, name of the plugin
      name: turtlebot_drive 

      # required, body of a model to set velocities and obtain odometry
      body: base

      # optional, defaults to odom, the name of the odom frame
      odom_frame_id: odom

      # optional, defaults to inf, rate to publish odometry at, in Hz
      pub_rate: .inf

      # optional, defaults to "cmd_vel", the topic to subscribe for velocity
      # commands
      twist_sub: cmd_vel

      # optional, defaults to "odom", the topic to advertise for
      # publish noisy odometry
      odom_pub: odom

      # optional, defaults to "ground_truth/odom", the topic to advertise for publish
      # no noise ground truth odometry
      ground_truth_pub: ground_truth/odom

      # optional, defaults to "twist", the topic to publish noisy local frame velocity
      # that simulates encoder readings
      twist_pub: twist
      
      # optional, defaults to true, enables the advertising and publishing of both
      # ground truth and noisy odometry
      enable_odom_pub: true
      
      # optional, defaults to true, enables the advertising and publishing of noisy local
      # frame velocity
      enable_twist_pub: true

      # optional, defaults to [0, 0, 0], corresponds to noise on [x, y, yaw], 
      # the variances of gaussian noise to apply to the pose components of the
      # odometry message
      odom_pose_noise: [0, 0, 0]

      # optional, defaults to [0, 0, 0], corresponds to noise on 
      # [x velocity, y velocity, yaw rate], the variances of gaussian noise to
      # apply to the twist components of the odometry message
      odom_twist_noise: [0, 0, 0]

      # optional, defaults to the diagonal [x, y, yaw] components replaced by 
      # odom_pose_noise with all other values equals zero, must have length of 36, 
      # represents a 6x6 covariance matrix for x, y, z, roll, pitch, yaw. 
      # This does not involve in any of the noise calculation, it is simply 
      # the output values of odometry pose covariance
      odom_pose_covariance: [0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0]

      # optional, defaults to the diagonal [x velocity, y velocity, yaw rate] 
      # components replaced by odom_twist_noise with all other values equals zero,
      # must have length of 36, represents a 6x6 covariance matrix for rates x, 
      # y, z, roll, pitch, yaw. This does not involve in any of the noise 
      # calculation, it is simply the output values of odometry twist covariance
      odom_twist_covariance: [0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0]

Flatland 2 ROS 2 extensions
---------------------------

The current ROS 2 defaults are ``odom_pub: odom`` and
``ground_truth_pub: ground_truth/odom``. There is also a
``ground_truth_pose_pub`` topic (default
``ground_truth/pose``) carrying ``geometry_msgs/msg/PoseWithCovarianceStamped``.
See :doc:`drive_options` for frame conventions, publication switches,
``enable_odom_tf_pub``, and noise/covariance options. Commands arrive as
``geometry_msgs/msg/Twist`` and persist until replaced by another command.

Optional ``linear_dynamics`` and ``angular_dynamics`` maps accept
``velocity_limit``, ``acceleration_limit`` and ``deceleration_limit`` as described
in :doc:`drive_options`. The angular map limits body yaw rate, not steering angle.

The ``use_id_model`` option (default false) selects two independent discrete
output-error (OE) response models, one for forward speed and one for yaw rate.
When enabled, both maps below are required, and they replace the dynamics-limit
path. Add these keys inside the ``DiffDrive`` entry:

.. code-block:: yaml

  use_id_model: true
  # Synthetic first-order examples, not measured vehicle coefficients.
  linear_oe_model:
    B: [0.2]
    F: [-0.8]
    nk: 0
    Ts: 0.1
  angular_oe_model:
    B: [0.2]
    F: [-0.8]
    nk: 0
    Ts: 0.1

``B`` contains numerator coefficients. ``F`` contains feedback coefficients
**without** the leading one. The implemented recurrence is
``y(k) = sum(B[i] * u(k - nk - i)) - sum(F[j] * y(k - 1 - j))``.
Both lists must be nonempty in the plugin YAML. ``nk`` is a nonnegative delay
in samples and ``Ts`` is a finite positive sample period in seconds, equal for
both models. Histories start at zero; the plugin accumulates physics time to
step the models at ``Ts`` and holds each result between updates. There is no
built-in command timeout or learned-model stability validation.

Use :doc:`diff_drive_caster` for passive caster disturbances, or
:doc:`system_id_drive` for coupled polynomial NARX models and Ackermann input.
These are alternative drive plugins; do not stack them on the same body.
