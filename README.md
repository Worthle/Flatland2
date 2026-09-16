# Flatland 2

Flatland 2 is a community-maintained ROS 2 Humble extension of
[Avidbots Flatland](https://github.com/avidbots/flatland), maintained by
[Levent Soysal (Worthle)](https://github.com/Worthle). It uses bundled Box2D
for planar robotics simulation. This is an independent fork, not an official
Avidbots release or an endorsed successor.

This repository contains the simulator core. Runnable robot demonstrations live
in the separate [flatland_examples checkout](../flatland_examples/README.md).
The examples provide four robot models, a small synthetic room, objects,
keyboard controls and Docker bringup. They depend on the core; the core builds
and runs independently.

## Packages

| Package | Purpose |
| --- | --- |
| `flatland` | Metapackage depending on the four simulator packages; installed licenses and attribution |
| `flatland_server` | World loading, Box2D simulation, model services and simulation time |
| `flatland_plugins` | Drive, sensor, terrain and payload plugins |
| `flatland_msgs` | ROS messages and services |
| `flatland_viz` | RViz2 application and interactive tools |

The metapackage uses upstream's `flatland` name. Demo launch files belong to
`flatland_examples`, so their command is `ros2 launch flatland_examples simulation.launch.py`.

## Features

- Differential, caster differential, Ackermann and omnidirectional drives.
- Optional OE and NARX vehicle dynamics, plus an offline NARX replay tool.
- Terrain patches, forklift motion, payload attachment and release.
- Laser, IMU, GPS, bumper and collision plugins; mock 3D lidar and detections.
- Runtime model spawning, moving, deletion, pause/resume and ROS namespaces.
- YAML models, body extrusion, wheel visuals and RViz interaction.

**Physics remains planar.** Heights, fork motion and mock 3D sensors add
visualization and sensor outputs around 2D Box2D motion. They do not provide
six-degree-of-freedom rigid-body physics.

## Build the core

Docker supplies ROS 2 Humble and the build dependencies:

```bash
docker build --build-arg BUILD_JOBS=2 -t flatland2:humble-core .
```

For a native build, place this repository under a Humble workspace's `src/`
directory, source ROS, then run from the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Add the `flatland_examples` checkout under the same `src/` directory when you
want the demos. See its [setup instructions](../flatland_examples/README.md)
for Docker and native launches.

## Run your own world

With a native installation sourced:

```bash
ros2 run flatland_server flatland_server --ros-args \
  -p world_path:=/absolute/path/to/world.yaml \
  -p update_rate:=100.0 -p step_size:=0.01
```

The core accepts your world and model YAML directly. For Docker, mount the
whole directory containing the world and its referenced models/maps:

```bash
docker run --rm --network host -v "$PWD/my_world:/world:ro" \
  flatland2:humble-core ros2 run flatland_server flatland_server --ros-args \
  -p world_path:=/world/world.yaml -p update_rate:=100.0 -p step_size:=0.01
```

[Quick start](docs/quick_start.rst) explains world and model configuration.
The [plugin catalog](docs/included_plugins/plugin_catalog.rst) lists the
current plugins; [documentation build instructions](docs/README.md) produce
the browsable reference. Some inherited tutorials still describe ROS 1.

## Validate

```bash
docker run --rm --network none -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  flatland2:humble-core python3 /opt/flatland-tests/server_smoke.py
docker run --rm --network none flatland2:humble-core \
  python3 /opt/flatland-tests/dynamics.py
```

The core checks use tiny synthetic test fixtures and need no example package.
The companion provides the broader robot, teleop and sensor integration tests.
See [validation](docs/validation.md) for coverage and limitations.

[Branch scope](UPSTREAM.md) · [Contributing](CONTRIBUTING.md) ·
[Publishing](PUBLISHING.md) · [ROS 1 comparison](docs/ros1-comparison.md) ·
[BSD 3-Clause license](LICENSE) · [Third-party notices](flatland/THIRD_PARTY_NOTICES.md)
