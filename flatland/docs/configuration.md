# Configuration and interfaces

Robot YAML is in `robot/`, objects in `objects/`, warehouse data in `maps/warehouse/`, and optional behavior examples in `config/`. PCD and identification-model paths accept absolute paths or `package://package/path` URIs resolved through the ROS package index. The launch generates temporary copies for runtime settings; the installed models remain reusable through `/spawn_model`. Directly spawned robot YAML publishes `map -> base_link` ground-truth TF; the launch replaces that with its configurable `map -> odom -> base_link` chain.

## Launch arguments

| Argument | Default | Purpose |
| --- | --- | --- |
| `robot` | `turtlebot` | One of the four supplied models |
| `robot_namespace` | empty | Topic prefix; TF uses the same prefix followed by `_` |
| `x`, `y`, `yaw` | `0`, `-2`, `0` | Spawn pose in metres and radians |
| `show_viz` | `false` | Start the Flatland RViz application |
| `objects` | `true` | Spawn the generic pallet, box and charger scene |
| `terrain` | `false` | Enable the example rough patch at X −1…1, Y −6…−4 |
| `localization` | `ground_truth` | `external` delegates `map -> odom` to a localizer |
| `publish_map` | `true` | Disable when an external mapping node publishes `/map` |
| `drive_model` | `physics` | `identified` selects the synthetic forklift NARX example |

The world runs with a requested 100 Hz update rate and 0.01 s physics step. Expensive point-cloud sensing can reduce wall-clock real-time performance; ROS timestamps still advance in simulation time. RViz marker updates are 15 Hz.

Compose starts the omni robot with RViz; the table lists the launch file's own defaults. To pass other launch arguments, stop the service and run:

```bash
./docker.sh down
./docker.sh run --rm flatland ros2 launch flatland simulation.launch.py \
  robot:=castor_wheeled_differential_robot terrain:=true show_viz:=true
```

## Custom maps and models

Add maps under `flatland/maps/`. An occupancy map needs an image such as a PGM and ROS map YAML containing its `image`, `resolution`, `origin`, `negate`, `occupied_thresh`, and `free_thresh`. For mock 3D lidar, add an XYZ float32 PCD in the same coordinate frame and units.

The demo selects the warehouse explicitly in `launch/simulation.launch.py`. To use another map, copy `maps/warehouse/warehouse.world.yaml` beside your map and update the launch's world-file path, layer map path, map-server `yaml_filename`, and lidar `pcd_path` to your files. The supplied robots use the `all` layer filter; adjust it if you want selective sensing or collisions. Choose a free spawn position and disable or relocate the demo objects and terrain patches for the new layout. If you have no PCD, set the launch's lidar `pcd_path` to an empty string to use simple wall extrusion.

To make a robot, copy a model into `flatland/robot/` and edit its bodies, footprints, joints, sensors, and drive plugin. Box2D polygon footprints must be convex and have at most eight vertices; combine several footprints for more complex shapes. Add reusable objects in `flatland/objects/` in the same format. Rebuild the Docker image to install new files.

Spawn a custom model into a running scene with an installed path and a free pose:

```bash
./docker.sh exec flatland /docker-entrypoint.sh ros2 service call /spawn_model \
  flatland_msgs/srv/SpawnModel \
  '{yaml_path: "/flatland_ws/install/flatland/share/flatland/robot/my_robot.model.yaml", name: "my_robot", ns: "my_robot", pose: {x: 0.0, y: -4.0, theta: 0.0}}'
```

The demo launch's `robot` argument accepts the four supplied models. To use your own as the primary robot, add its name to `ROBOTS` in the launch and adapt any drive-specific adapter or TF settings. Custom worlds can also be loaded directly by `flatland_server` through its `world_path` ROS parameter.

## ROS interfaces

| Topic | Message | Meaning |
| --- | --- | --- |
| `cmd_vel` | `geometry_msgs/Twist` | m/s and rad/s; lateral speed is supported by the omni model |
| `ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` | Forklift drive speed (m/s), steering angle (rad) |
| `drive/turret{1,2}/command` | `ackermann_msgs/AckermannDriveStamped` | Independent omni wheel speed and steering |
| `drive/turret{1,2}/measured` | `ackermann_msgs/AckermannDriveStamped` | Simulated turret state |
| `odom` | `nav_msgs/Odometry` | Spawn-relative pose and body-frame velocity |
| `ground_truth/odom` | `nav_msgs/Odometry` | Absolute world pose and body-frame velocity |
| `ground_truth/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Absolute world pose |
| `scan` | `sensor_msgs/LaserScan` | 2D range measurements |
| `lidar_points` | `sensor_msgs/PointCloud2` | Approximate XYZ returns from the PCD and dynamic objects |
| `imu/data` | `sensor_msgs/Imu` | Simulated inertial measurements |
| `detections` | `geometry_msgs/PoseArray` | Matching object poses in camera coordinates, +X forward |
| `fork/goal_height` | `std_msgs/Float32` | Absolute commanded height (metres) |
| `fork/height`, `fork/pose` | `std_msgs/Float32`, `geometry_msgs/PoseStamped` | Animated fork height |
| `fork/goal_reached`, `fork/loaded` | `std_msgs/Bool` | Height goal and payload state |
| `fork/attached_model` | `std_msgs/String` | Attached model name; empty when detached |
| `fork/weight` | `std_msgs/Float64` | Configured mock payload weight, in kg |
| `fork/contact_left`, `fork/contact_right` | `std_msgs/Bool` | Capture-point proximity signals |

Topics in this table are relative to the model namespace. `/map`, `/clock`, `/tf`, and the world services are shared. The optional `fork/collision_left` and `fork/collision_right` feedback is always false: the plugin has no independent fork-tip collision sensor. Use the collision/bumper plugins when contact measurements are required.

The demo omni command adapter owns both turret command topics. `omni_teleop.py` sends body-frame `cmd_vel` through that adapter, so only one node commands the turrets. If an external controller sends turret commands directly, remove the `omni_command.py` node from the launch. The adapter stops after 0.5 s without a `cmd_vel` command and preserves the last steering angles at rest. Base drive plugins otherwise retain the most recent command. The NARX drive has its own configurable command timeout.

World services are `/spawn_model` (`flatland_msgs/SpawnModel`), `/move_model` (`flatland_msgs/MoveModel`), `/delete_model` (`flatland_msgs/DeleteModel`), and `/pause`, `/resume`, `/toggle_pause` (`std_srvs/Empty`). `/fork/lift` takes `fraction` in [0,1]; `/fork/attach` takes `model_name`; `/fork/detach` uses `std_srvs/Trigger`. Inspect fields with `ros2 interface show` inside the container.

## Keyboard control

Each drive type has its own keyboard controls, publishing to `cmd_vel` or `ackermann_cmd`.

```bash
# TurtleBot or caster differential robot
./docker.sh exec flatland /docker-entrypoint.sh ros2 run flatland diff_teleop.py
# Omni robot
./docker.sh exec flatland /docker-entrypoint.sh ros2 run flatland omni_teleop.py
# Forklift
./docker.sh exec flatland /docker-entrypoint.sh ros2 run flatland ackermann_teleop.py
```

| Action | Differential | Omni | Ackermann |
| --- | --- | --- | --- |
| Forward / reverse | W / X | W / X | W / S or Up / Down adjust speed |
| Turn left / right | A / D | J / L rotate | A / D or Left / Right adjust steering |
| Strafe left / right | — | A / D | — |
| Diagonal movement | Q/E/Z/C drive arcs | Q/E/Z/C translate diagonally | — |
| Stop | S or Space | S or Space | Space brakes, keeping steering |
| Align / stop rotation | — | K stops rotation | Tab centers steering, keeping speed |
| Adjust both speeds | T / G, +10% / −10% | T / G, +10% / −10% | — |
| Adjust linear speed | Y / H | Y / H | W / S or Up / Down |
| Adjust angular speed | U / J | U / N | A / D or Left / Right |
| Exit and stop | Ctrl-C | Ctrl-C | Q or Ctrl-C |

Differential and omni movement ramps smoothly while a key is held. They start slowing down after 0.75 s without a movement or speed key; `key_timeout` is a ROS parameter. S and Space stop immediately. Differential defaults are 0.3 m/s and 0.2 rad/s; omni defaults are 0.3 m/s and 0.5 rad/s. Set `linear_speed` and `angular_speed` with ROS parameters.

Ackermann commands stay active until changed or stopped, and are republished at 5 Hz. W/S (or Up/Down) adjust speed and A/D (or Left/Right) adjust steering by one fifth of the configured limit per press. S decreases speed; Space is the brake. Optional positional arguments are `max_speed max_steering_angle topic`, for example `ackermann_teleop.py 1.2 1.2 ackermann_cmd`. Defaults are 2.0 m/s and 1.57 rad; the drive plugin still applies its own limits. Space leaves steering unchanged, Tab leaves speed unchanged, and exit stops and centers the wheels.

All scripts accept ROS remaps, for example `omni_teleop.py --ros-args -r __ns:=/robot1`. Differential teleop also accepts a positional namespace, such as `diff_teleop.py robot1`. Use an interactive terminal; the scripts restore terminal settings and send a final stop on exit.

## Drive and terrain models

`DiffDrive` supports optional `use_id_model`, `linear_oe_model` and `angular_oe_model`. Each OE block contains `B`, `F` (excluding its leading 1), `nk` (input delay), and `Ts` (seconds). Both axes must use the same positive sample time. For example, `B: [0.2]`, `F: [-0.8]`, `nk: 1`, `Ts: 0.1` is a synthetic first-order lag.

`SystemIdDrive` loads a polynomial NARX YAML and maps `cmd_speed` and `cmd_steering_angle` from Ackermann commands to predicted forward and yaw velocities. Replace the propulsion plugin, leaving the body geometry and sensors independent. `config/forklift_dynamics.yaml` has synthetic teaching coefficients, with a small-angle steering approximation; it contains no measured vehicle identification data. `narx_sim_tool` can replay a CSV against a model for offline validation.

Caster behavior is controlled by the caster bodies, `trail`, `caster_lat_mu`, `caster_long_mu`, and `caster_alignment_rate`. `caster_lat_mu` is a slip fraction (0 blocks lateral slip; 1 allows it), while `caster_long_mu` is the fraction of rolling velocity removed. The supplied model begins with misaligned casters, 0.20 m trails, and a slower alignment rate of 2 s⁻¹. Straight starts, reversals, and turns expose the resulting speed and yaw transients. The caster plugin uses an approximate lateral-impulse and alignment model.

`TerrainModifier` applies rectangular patches after propulsion. Slip modifies planar speed/yaw, drift adds lateral velocity, rubble adds vibration, and slope projects a gravity term into planar motion. It does not pitch the robot or deform its footprint. `config/terrain.yaml` uses a fixed seed for repeatable terrain sampling.

The omni model has powered turrets at front-left and rear-right, with passive casters at front-right and rear-left. `OmniDrive.caster_joints` selects their revolute pivots; `caster_alignment_rate` controls how quickly the wheels trail the local pivot velocity during translation and rotation. These idlers use a simple planar alignment model; the stronger slip/drag transients remain part of `DiffDriveCaster`.

## Perception and visualization

`MockDetection` matches model-name prefixes. Tune range, FOV, pose noise, dropout, and occluding layers. It emits no frame when nothing is detected. Optional target TFs remember the last observed world pose and continue publishing it; they are not a tracker with confidence or expiry. No rendered image pipeline is provided.

`MockLidar3D` samples a reference PCD using nearest returns per azimuth/elevation bin. Its dynamic-object approximation uses planar raycasts and configured heights; it is not volumetric ray tracing. Supply XYZ float32 PCD data in the same world frame. `sampling_threads: 0` automatically uses up to eight workers for dense maps; set a positive value to limit CPU concurrency per robot. Workers preserve nearest-return ordering and process the full reference cloud. The supplied models publish at 20 Hz. Dense PCD projection runs asynchronously, while capture-time dynamic-object raycasts stay on the physics thread. Only one scan is in flight; a slow scan skips the next capture instead of delaying physics or building a backlog. Published timestamps retain the capture time so TF and localization use the matching pose. The static projection is reused only when the sensor pose is exactly unchanged; dynamic-object raycasts and range noise are refreshed each scan. An empty `pcd_path` enables fixed-height extrusion from planar walls.

Bodies accept `extrude` and `elevation` in metres. Forks and attached loads animate those values, and the body TF includes relative elevation. The Flatland RViz application adds model-spawn and pause tools and subscribes to its transient-local debug topic list to create marker displays automatically. When using `robot_namespace`, adjust the RViz sensor topic selections to that namespace.

### RViz performance

The supplied lidar display uses `Style: Points`, `Size (Pixels): 2` and `Decay Time: 0`. Changing to spheres or accumulating many scans increases rendering work. Start through `./docker.sh` to use NVIDIA automatically when Docker can access it, with CPU rendering as the fallback. There is one Compose configuration for both. Physics and sensor sampling still run on the CPU.

### Wheel visuals

A body can replace its footprint extrusion with a tire, rim, hub, spokes and tread:

```yaml
wheel_visual: {radius: 0.14, width: 0.10, center: [0, 0]}
```

Dimensions are in metres. The axle follows the body's local Y axis; local X is the rolling direction. The wheel center is one radius above the body's elevation. A nonzero planar `center` also draws a caster support fork. The spokes and tread rotate by signed travel distance divided by radius, including reverse travel and independent wheel motion during turns.

`visual_z_offset` shifts only the displayed body vertically. It lets the chassis clear the tires without changing the collision footprints, mass, sensor elevation or TF. The examples derive tire dimensions from their existing wheel footprints and use invisible joint-debug markers. These are animated visual wheels, not an additional 3D contact model.

The default 2D scan is blue and the 3D cloud is red, both using RViz's flat-color transformer.
