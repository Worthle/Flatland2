# Flatland2 compared with Avidbots Flatland

Baseline: [Avidbots Flatland master at `6601c6560190372f304a45f3a0f758ea9ca8ea5b`](https://github.com/avidbots/flatland/tree/6601c6560190372f304a45f3a0f758ea9ca8ea5b). This comparison covers source and configuration relative to ROS 1. Avidbots also maintains ROS 2 branches; the table does not imply that all ROS 2 support originated in this fork. See [upstream history](../UPSTREAM.md) for the Git integration base.

## Inherited from ROS1

The Box2D world, YAML bodies and footprints, collision layers, revolute/weld joints, plugin loading, simulation time, model spawn/move/delete services, and pause/resume are inherited. So are differential and tricycle drives, laser, IMU, GPS, bumper and boolean collision sensors, Tween motion, random walls, model TF, YAML/Lua preprocessing, and interactive model tools. Those capabilities should not be credited as new ROS 2 features.

## Added or substantially extended

| Area | ROS1 baseline | ROS 2 reference and standalone release |
| --- | --- | --- |
| ROS integration | catkin, roscpp, ROS1 interfaces and RViz | ament/colcon, rclcpp/rclpy, generated ROS2 interfaces, pluginlib, TF2, DDS QoS and RViz2 |
| Ackermann commands | Tricycle drive using `Twist` | `TricycleDriveAckermann` using `AckermannDriveStamped`; configurable steering dynamics and motion limits |
| Omnidirectional motion | No dedicated omni plugin | `OmniDrive` with two independent steering turrets, measured turret state and wheel-speed channels |
| Casters | No caster drive plugin | `DiffDriveCaster` with lateral/rolling resistance, trailing contact offsets and approximate alignment dynamics |
| Direct dynamics | Kinematic drive and limiters | Discrete OE models in `DiffDrive`, plus cross-coupled polynomial NARX/NARMAX free-run through `SystemIdDrive` |
| Offline model evaluation | No equivalent tool | `narx_sim_tool` replays CSV commands, including optional seeded output histories |
| Terrain | Flat planar world | `TerrainModifier`: bounded regions with slip, lateral drift, rubble vibration and planar slope effects |
| Fork operation | No fork controller | Animated lift fractions/heights, configurable speed and thickness, and state publication |
| Payload handling | Static model joints | Runtime attachment/detachment, nearest-target capture, collision suppression while carried, height following and settling after release |
| Fork feedback | No equivalent | Generic `ForkController` with height goals, proximity contacts, mock load/weight feedback and optional automatic pickup/dropoff |
| Pseudo-3D | Planar model display | Body extrusion and elevation, raised forks/loads, and filled geometry in RViz |
| 3D sensing | No 3D lidar plugin | `MockLidar3D`: PCD-based spherical return sampling, dynamic-object height spans, or fixed-height wall extrusion |
| Mock perception | No camera detection plugin | Model-name selection, range/FOV, raycast visibility, Gaussian pose noise, dropped frames and optional last-observed target TFs |
| 2D laser filtering | Layer filtering and raycasting | Per-body exclusions, upside-down mounting option and maximum body elevation filtering |
| Pose output | Drive odometry | Additional absolute pose topics for external evaluation and integration |
| Visualization application | ROS1 custom RViz application | RViz2 VisualizationFrame, custom spawn/pause tools and automatic debug MarkerArray display discovery |
| Bringup and assets | Separate example environments | Separate `flatland_examples` companion: Docker bringup, four generic models, objects, a synthetic room map, teleop and external-localization TF selection |


## Release cleanup and corrections

The primary workspace contained 561 files under `src`, plus separate runtime assets, external autonomy launch/configuration, fleet deployment configuration, and local caches. The release inventory read the source and configuration trees, compared the engine files with the pinned upstream snapshot, and scanned retained text, comments, filenames and asset paths. The private Java font cache and Git history are excluded from the release.

Company robot models, site maps, measured identification coefficients, fleet-order scripts, destination replication, controller/perception deployment wiring, private image names, machine-specific mounts, vendor sensor defaults, and duplicate runtime asset trees were removed. Generic robot geometry, objects, mock perception and fork control replace the application-specific configurations.

The core metapackage is `flatland`; the public bringup package is `flatland_examples` in a separate checkout. The simulator packages are sibling directories at the core repository root. Existing `flatland_server`, `flatland_plugins`, `flatland_msgs`, and `flatland_viz` package names remain stable. The old fork-driver adapter was refactored into `ForkController` with relative generic topics. The unused `CommandFlatland` compatibility service was removed because its handlers only logged success. ROS1-only rostest fixtures, the unbuilt ROS1 benchmark executable, obsolete example maps, duplicate caster source and large commented-out implementations were removed; the release uses ROS2 runtime tests.

Release corrections include removing double topic namespaces in sensor/fork plugins, using the IMU's namespaced frame, validating wheel anchors in the wheel frame, correcting spawn-relative odometry at nonzero headings, publishing absolute ground truth consistently, retaining body elevation in TF, validating OE sample times, making caster alignment speed configurable, resolving installed assets through package URIs, and accepting CRLF CSV input with malformed-row checks in the replay tool.

## Scope of the simulation

Pseudo-3D display, 3D lidar output and external 3D localization support do not make the engine a six-degree-of-freedom physics simulator. Terrain effects remain planar. Mock perception returns poses rather than camera images. Fork contact and load feedback are simplified signals. Direct identified models replace propulsion; they do not infer collision geometry. An external stack's historical use is separate from the standalone build and runtime tests.
