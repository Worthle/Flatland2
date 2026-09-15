# External AMR integrations

Tested with HDL Localization, SLAM Toolbox, AMCL, and OpenTCS through an MQTT bridge. Run these packages alongside Flatland 2 using the simulator's ROS interfaces; they are not bundled in the Docker image.

Use ROS 2 Humble, matching `ROS_DOMAIN_ID`, a compatible DDS configuration, and `use_sim_time: true` in every external ROS node. The default Compose domain is 42 and its RMW is CycloneDDS. The simulator publishes `/clock`.

## TF and localization

The default chain is `map -> odom -> base_link -> sensor/body frames`. `map -> odom` is the configured spawn transform. `/odom` starts at zero in the spawn frame; `/ground_truth/pose` and `/ground_truth/odom` are absolute in `map`.

Start with `LOCALIZATION=external ./docker.sh up -d` to suppress the simulator's `map -> odom`. Your localizer then owns that transform. The simulator continues to publish `odom -> base_link` from its odometry. If an external estimator also publishes `odom -> base_link`, disable or replace the `odometry_tf.py` launch node to give that edge a single publisher. Ground truth remains available for evaluation.

For 2D localization, the inputs are `/scan`, `/odom`, and the TF chain. For 3D localization, `/lidar_points` is `sensor_msgs/PointCloud2`, `/imu/data` is `sensor_msgs/Imu`, and the global reference is `maps/warehouse/warehouse.pcd`. The cloud uses a generic 16-channel example; it is not a calibrated model of a commercial lidar. Sensor synthesis and vehicle motion retain the simulator's planar assumptions.

## SLAM Toolbox

[SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox/tree/humble) supports mapping and localization using scans, odometry transforms, and serialized pose graphs. Run it in your own Humble workspace or container, with these parameter values adapted to its launch configuration:

```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: true
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping
```

Use `localization:=external` in Flatland2. When mapping, also start the simulator with `publish_map:=false` so SLAM Toolbox owns `/map`. Drive through the warehouse, serialize the resulting pose graph, and switch to SLAM Toolbox's localization configuration when needed. The supplied PGM/YAML is an occupancy map; it is not a serialized SLAM Toolbox session.

For namespaced robots, topics become `/robot1/scan` and `/robot1/odom`, while model frame names retain Flatland's underscore convention: `robot1_base_link`, `robot1_odom`, `robot1_laser`, and `robot1_lidar3d`. Configure the external localizer accordingly.

## MQTT and OpenTCS

[OpenTCS](https://github.com/openTCS/opentcs) manages transport orders and vehicle communication through adapters. An external MQTT bridge can connect a vehicle adapter or fleet interface to a ROS navigation/controller stack. Flatland2 itself contains no broker, MQTT client, transport-order executor, VDA controller, or OpenTCS vehicle adapter.

A typical connection is:

```text
OpenTCS vehicle adapter <-> MQTT broker/bridge <-> ROS navigation/controller
                                                       |
                                     cmd_vel or ackermann_cmd
                                                       v
                                                  Flatland2
```

The bridge should translate transport requests into your navigation stack's goals. The controller supplies `/cmd_vel` for differential and omni models, or `/ackermann_cmd` for the forklift. State feedback can use `/odom`, external localization, `/fork/height`, `/fork/loaded`, and `/fork/attached_model`. Fork operations can call `/fork/attach`, `/fork/lift`, and `/fork/detach`, or publish absolute height on `/fork/goal_height`.

Define the MQTT topic names, payload schemas, order identifiers, units, reconnect behavior, and completion acknowledgments in your adapter. Do not treat a transport order as a velocity command: route planning, traffic coordination, navigation, and task completion belong to the external stack. Keep broker credentials in that stack's configuration.
