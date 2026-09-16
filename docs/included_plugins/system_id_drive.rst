System ID Drive
===============

``SystemIdDrive`` replaces kinematic propulsion with a previously identified
polynomial model. A nonlinear autoregressive model with external inputs
(NARX) predicts forward velocity and yaw rate from commands and past outputs.
Those predictions are written to the body's planar velocities each physics
step. Box2D still handles the model's geometry and contacts.

The plugin loads a model; it does not identify or train one at runtime. Use it
as the sole drive plugin on the body. No wheel joints are required. For
independent linear output-error (OE) filters with ``Twist`` input, see the
``DiffDrive`` extension described in :doc:`diff_drive` instead.

Configuration
-------------

.. code-block:: yaml

  plugins:
    - type: SystemIdDrive
      name: identified_drive
      body: base_link                  # required
      model_path: package://flatland_examples/config/forklift_dynamics.yaml # required
      command_sub: ackermann_cmd       # optional, default ackermann_cmd
      command_mapping: ackermann      # optional, default and only supported value
      linear_output: meas_linear_velocity   # optional, default as shown
      angular_output: meas_angular_velocity # optional, default as shown
      cmd_timeout: 0.5                 # optional, default 0.5 simulation seconds
      pub_rate: 30.0                   # optional, default .inf, Hz
      odom_frame_id: odom              # optional, default odom
      ground_truth_frame_id: map       # optional, default map
      odom_pub: odom                   # optional, default odom
      ground_truth_pub: ground_truth/odom # optional, default ground_truth/odom
      ground_truth_pose_pub: ground_truth/pose # optional, default as shown

``model_path`` accepts a filesystem path or installed ``package://`` URI.
Relative filesystem paths resolve from the server's working directory.
The file in the optional ``flatland_examples`` package contains synthetic
teaching coefficients, not a measured vehicle identification dataset. It is a runnable format example, not a
calibrated forklift model.

Command and output mapping
--------------------------

``command_sub`` accepts ``ackermann_msgs/msg/AckermannDriveStamped``.
``command_mapping: ackermann`` exposes exactly two possible external inputs:

* ``cmd_speed`` from ``drive.speed`` in m/s.
* ``cmd_steering_angle`` from ``drive.steering_angle`` in radians.

Every external input required by the loaded model must have one of those
names. An input may also name another sub-model output to use its prediction
history. ``linear_output`` and ``angular_output`` must select outputs present
in the file and represent forward velocity in m/s and yaw rate in rad/s.
Unsupported mapping names, unknown command inputs and missing outputs are
rejected at initialization.

Model file
----------

The YAML contains a positive top-level ``sample_rate_hz`` and a nonempty
``sub_models`` map. Each sub-model has:

* ``output_name`` (defaults to its map key).
* ``input_names`` in the order used by ``x1``, ``x2``, and so on.
* A nonempty ``terms`` list. Each term has numeric ``theta`` and optional
  ``factors``. A factor is ``[variable, lag]``. ``y`` means that sub-model's
  own past output, while ``x1`` means its first input. Omitted or empty factors
  define a constant term.

The following small example predicts both velocities and is intentionally
synthetic:

.. code-block:: yaml

  sample_rate_hz: 10.0
  sub_models:
    meas_linear_velocity:
      input_names: [cmd_speed]
      terms:
        - {theta: 0.7, factors: [[y, 1]]}
        - {theta: 0.3, factors: [[x1, 1]]}
    meas_angular_velocity:
      input_names: [cmd_steering_angle]
      terms:
        - {theta: 0.7, factors: [[y, 1]]}
        - {theta: 0.15, factors: [[x1, 1]]}

Own-output and coupled predicted-output lags must be at least one. External
command lags may be zero. Histories start at zero. All sub-model outputs for
a step use previous prediction histories, then advance together. The runtime
evaluates the explicit terms; identification metadata such as ``ylag``,
``xlag`` and ``degree`` does not substitute for that list.

Timing, odometry and limitations
--------------------------------

The model timer uses the file's ``sample_rate_hz``. Its predictions are held
between model updates. Set the physics update rate at least as high as the
model rate, preferably an integer multiple; this plugin performs at most one
model update per physics step and does not catch up with multiple updates.

Before any command, or after ``cmd_timeout`` has elapsed, the model receives
zero inputs. A zero or negative timeout disables expiry after the first
command. This is not an immediate brake: predicted motion can decay according
to the model's stored history. Command freshness uses simulation time, not the
message header timestamp.

Odometry topics and noise/covariance keys are described in :doc:`drive_options`.
This plugin does not publish odometry TF. Ground-truth pose is the actual
world pose, but the published twist reports the model's predicted velocities,
which can differ from realized motion during contact. There are no additional
speed limits or stability checks for the learned dynamics. Validate a model's
response before relying on it. Use ``ros2 run flatland_plugins narx_sim_tool``
to replay command CSV files through the model offline.
