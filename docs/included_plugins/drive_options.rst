.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Shared Drive Options
====================

The options below describe the current ROS 2 implementations of
:doc:`diff_drive`, :doc:`diff_drive_caster`, :doc:`omni_drive`,
:doc:`tricycle_drive_ackermann` and :doc:`system_id_drive`. They are optional
keys inside the individual plugin entry, not a separate plugin.

Topics, frames and publication rate
-----------------------------------

.. code-block:: yaml

  odom_frame_id: odom                   # default odom
  ground_truth_frame_id: map            # default map
  odom_pub: odom                        # default odom
  ground_truth_pub: ground_truth/odom   # default ground_truth/odom
  ground_truth_pose_pub: ground_truth/pose # default ground_truth/pose
  pub_rate: .inf                       # default .inf, every simulation step

``odom_pub`` is ``nav_msgs/msg/Odometry`` relative to the initial spawn pose,
with axes aligned to the initial heading. ``ground_truth_pub`` is
``nav_msgs/msg/Odometry`` with absolute world pose.
``ground_truth_pose_pub`` is ``geometry_msgs/msg/PoseWithCovarianceStamped``
with absolute world pose. Ground-truth messages do not receive the configured
odometry noise. Their frame name must agree with the simulation's world frame;
changing ``ground_truth_frame_id`` does not transform their coordinates.

Relative topic names inherit the model namespace. Odom and body TF frame IDs
are also namespaced by the model; ``ground_truth_frame_id`` is used as supplied.
For example, a model in namespace ``robot1`` with default odometry settings
publishes on ``/robot1/odom`` with parent frame ``robot1/odom``.

Noise and covariance
--------------------

.. code-block:: yaml

  odom_pose_noise: [0.0, 0.0, 0.0]      # default; variances for x, y, yaw
  odom_twist_noise: [0.0, 0.0, 0.0]     # default; variances for vx, vy, yaw rate

Use nonnegative variances. Pose variances have units m^2, m^2 and rad^2;
twist variances have units (m/s)^2, (m/s)^2 and (rad/s)^2. Each sample receives
independent Gaussian noise with standard deviation equal to the square root
of the configured variance. These options model measurement noise, not a
random-walk error integrated into the robot's physical pose.

Optional ``odom_pose_covariance`` and ``odom_twist_covariance`` each contain
36 row-major entries for a 6 by 6 matrix in ROS order
``[x, y, z, roll, pitch, yaw]``. By default only the x, y and yaw diagonal
entries (indices 0, 7 and 35) are set from the corresponding noise variances.
Overriding the covariance changes message metadata, not noise generation.

Additional options for DiffDrive, DiffDriveCaster and OmniDrive
---------------------------------------------------------------

Only these three plugins accept the following keys:

.. code-block:: yaml

  enable_odom_pub: true                 # default true
  enable_odom_tf_pub: true              # default true
  enable_twist_pub: true               # default true
  twist_in_local_frame: true           # default true
  twist_pub: twist                     # default twist

``enable_odom_pub`` controls all three odometry/ground-truth publishers.
``enable_odom_tf_pub`` independently controls the odom-to-body TF broadcast.
``enable_twist_pub`` controls a
``geometry_msgs/msg/TwistWithCovarianceStamped`` velocity message. The default
``twist_in_local_frame: true`` expresses odometry twist in body axes; keep this
setting for standard ROS odometry consumers. Setting it false uses world-axis
linear components in the odometry twist. The separate ``twist_pub`` message
uses body axes.

``TricycleDriveAckermann`` and ``SystemIdDrive`` always publish their three
configured odometry/pose topics and do not provide an odometry-TF broadcaster.
Do not pass the five additional keys above to those plugins.

Dynamics limits
---------------

The kinematic drives accept the nested dynamics maps listed on their pages.
Each map supports ``velocity_limit`` and ``acceleration_limit`` (default 0,
disabled), and ``deceleration_limit`` (defaults to ``acceleration_limit``).
Use positive limits to enable them. For steering maps, velocity and
acceleration mean steering-angle rate and its rate of change. ``SystemIdDrive``
uses its model's response and does not accept these dynamics maps.
