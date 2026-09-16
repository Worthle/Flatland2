IMU
===

``Imu`` publishes a planar inertial measurement as ``sensor_msgs/msg/Imu``,
with separate noisy and ground-truth topics. It obtains yaw and yaw rate from
the attached body's motion and estimates linear acceleration from consecutive
velocity samples. Roll, pitch and vertical acceleration are not simulated,
and gravity is not added to the output.

Configuration
-------------

.. code-block:: yaml

  plugins:
    - type: Imu                     # case-sensitive plugin type
      name: imu_sensor
      body: base_link               # required, existing body
      imu_frame_id: imu             # optional, default imu
      imu_pub: imu/data             # optional, default imu/filtered
      ground_truth_pub: imu/ground_truth # optional, default imu/ground_truth
      enable_imu_pub: true          # optional, default true, both output topics
      broadcast_tf: true            # optional, default true

      # Optional in the parser, whose default is .inf. Supply a finite,
      # positive rate because acceleration uses this value as a multiplier.
      pub_rate: 50.0                # Hz; choose a realizable simulation rate

      # Optional, default [0, 0, 0]. These are VARIANCES, not std deviations.
      orientation_noise: [0.0, 0.0, 0.0001]        # roll, pitch, yaw; rad^2
      angular_velocity_noise: [0.0, 0.0, 0.0001]   # x, y, z; (rad/s)^2
      linear_acceleration_noise: [0.01, 0.01, 0.0] # x, y, z; (m/s^2)^2

Optional ``orientation_covariance``, ``angular_velocity_covariance`` and
``linear_acceleration_covariance`` each accept nine row-major values for a
3 by 3 matrix. Their defaults place the corresponding three noise variances
on the diagonal, with zero elsewhere. Covariance overrides affect message
metadata only. Noise is actually added to yaw, angular velocity Z and linear
acceleration X/Y; the other axes remain at their planar defaults.

ROS interfaces and limitations
------------------------------

Both output topics use the namespaced ``imu_frame_id``. Relative topic names
inherit the model namespace. With ``broadcast_tf: true``, the plugin broadcasts
an identity transform from the body to that IMU frame on every physics step,
independently of the publication enable flag. There is no configurable sensor
mount offset in this plugin; use an appropriately placed body if needed.

Set an explicit finite ``pub_rate`` as shown above. The current acceleration
calculation multiplies a velocity difference by the configured rate, rather
than dividing by the actual elapsed sample time. The parser's ``.inf`` default
can therefore yield non-finite acceleration. A rate that the physics timestep
cannot realize also affects acceleration accuracy. Yaw rate uses the realized
angle difference and simulation time after the first sample.

The ground-truth topic omits the configured measurement noise but remains a
simplified planar measurement. This plugin is not a full 3D IMU or a calibrated
model of bias, gravity, vibration and sensor mounting dynamics.
