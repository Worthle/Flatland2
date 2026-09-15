# Release validation

## Branch export validation — 15 September 2026

The `flatland2` publication checkout was checked after moving the six ROS
packages to the repository root and updating Docker and documentation paths:

- A full Docker build completed all six packages using the separate
  `flatland2:humble-fork` image tag.
- All seven host-side Docker launcher checks passed, along with shell syntax
  and Compose configuration validation.
- Colcon discovered exactly six packages. Their package XML and all 19 package
  Python files parsed, and local links in the ten current Markdown documents
  resolved.
- All 254 core package files and the root license matched the source workspace
  byte for byte. Existing source formatting, including imported whitespace,
  was preserved; this was not a clang-tidy or formatting audit.
- Asset verification passed both from the checkout and from the installed
  Docker image, including all four robot models and the 2,277,784-point map.
- The installed NARX dynamics check passed forward/reverse/stop replay, CRLF
  input and malformed-row handling.
- The installed TurtleBot smoke test with `--spawn-probe` passed sensor, map,
  TF, motion, odometry, pause/resume, and namespaced model spawn/delete checks.

The runtime checks ran in isolated containers without mounts from the original
workspace. The original `flatland2:humble` image was unchanged. The Humble GitHub
Actions workflow runs the same build and focused checks; it has not yet run on
GitHub. RViz graphical inspection and the broader robot matrix below were not
repeated for this layout change.

## Earlier runtime validation

The following results were recorded on 12 September 2026 with the ROS 2 Humble
Docker image on Linux x86-64 in the source workspace. They are retained as
historical validation; reproduction commands below use this branch's layout
and Docker tag.

## Build and assets

- Docker builds all six ament packages from source: `flatland_msgs`, `flatland_server`, `flatland_plugins`, `flatland_viz`, `flatland_core`, and `flatland`.
- The asset check validates the four robot models, body/joint references, registered plugin types, default spawn clearance, and source/output SHA-256 digests.
- The AWS-derived cloud contains 2,277,784 finite XYZ samples. The occupancy map is 300 × 440 cells at 0.05 m resolution. Surface samples in the occupancy height band project onto occupied cells, allowing one cell of rasterization rounding.
- Regenerating the dataset from the bundled collision meshes reproduced the PCD, PGM, YAML, and provenance byte for byte.
- Python syntax, package XML, Compose configuration, relative documentation links, retained source identifiers, and third-party notices were reviewed. The original Box2D, ThreadPool, and Tweeny files are preserved byte for byte.

## Runtime checks

Each smoke test starts a simulator in an isolated container, commands motion, and shuts down its processes. All models are checked for laser, IMU and point-cloud output, expected sensor frames, odometry, ground-truth poses, and pause/resume behavior. Tests start at a nonzero heading to check that spawn-relative odometry and absolute world poses remain consistent.

| Scenario | Additional checks |
| --- | --- |
| TurtleBot | Ground-truth TF chain and direct spawning of another installed robot YAML through `/spawn_model`, including package asset resolution, namespaced scan/TF and deletion |
| Caster differential robot | Misaligned casters introduce measurable lateral displacement or yaw during a straight start |
| Omni robot, `robot2` namespace | Lateral command produces lateral travel; external localization leaves `map -> odom` unpublished and mapping mode leaves `/map` unpublished |
| Forklift, physics drive | Pallet detection, attach, lift above 0.85 m, loaded state, detach and delete |
| Forklift, identified drive, `robot1` namespace | The same payload operations with NARX propulsion and namespaced interfaces |
| TurtleBot inside the rough-floor patch | Slip/rubble loading and measurable lateral or yaw disturbance |

The offline NARX replay check compares forward, reverse, and stopping responses with an analytic first-order recurrence. It also checks CRLF CSV input and rejection of rows with missing fields.

RViz2 was launched on an X11 display using software OpenGL. The warehouse, robot extrusion, fork geometry, occupancy map and lidar were inspected with an OK global TF status. Graphics behavior depends on the host display and driver. The inspected software-rendering session emitted nonfatal Ogre/Mesa shader warnings. The current README image comes from the later NVIDIA wheel inspection below.

## Reproduce

From the repository root:

```bash
./docker.sh build
docker run --rm --network none -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp flatland2:humble-fork \
  python3 /flatland_ws/install/flatland/share/flatland/tests/verify_assets.py
docker run --rm --network none -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp flatland2:humble-fork \
  python3 /flatland_ws/install/flatland/share/flatland/tests/dynamics.py
docker run --rm --network none -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp flatland2:humble-fork \
  python3 /flatland_ws/install/flatland/share/flatland/tests/smoke.py --robot turtlebot --spawn-probe
```

Repeat the last command with the following smoke-test arguments:

```text
--robot castor_wheeled_differential_robot
--robot omni_drive_robot --namespace robot2 --external
--robot forklift
--robot forklift --drive-model identified --namespace robot1
--robot turtlebot --terrain
```

Use the [dataset regeneration command](../maps/warehouse/README.md) to rebuild the reference maps. Compare all three maps and `provenance.json` together.

## Limits

These are build, asset and focused functional checks, not exhaustive plugin coverage, performance benchmarks, or physical calibration. The historical ROS1 rostests were not carried over as working ROS2 tests. GPS, bumper, Tween and random-wall behavior were not individually exercised in this release. The direct OE path is included and reviewed; the runtime identification scenario uses NARX.

HDL Localization, AMCL, SLAM Toolbox and MQTT/OpenTCS were tested in earlier workspaces, separately from the standalone checks listed here. No external stack, broker, fleet adapter or measured vehicle identification dataset is bundled.

## Lidar performance and wheel rendering update

The dense warehouse PCD is sampled in parallel without downsampling it or reducing beam counts. A serial-versus-parallel comparison over eight sensor poses matched every occupied return exactly and exercised cached poses and equal-distance ties. On the development host, projection time fell from approximately 119 ms to 19 ms with eight workers. An isolated simulator measurement with the cloud subscribed improved from approximately 0.63 to 0.89 real-time factor; these timings depend on CPU load and hardware.

OpenGL probes confirmed NVIDIA RTX 5000 rendering and llvmpipe software rendering. The default cloud display now uses points instead of the more expensive default point-cloud glyphs.

The single Compose setup was checked through `docker.sh`: the container selected NVIDIA RTX 5000 graphics normally and llvmpipe when the launcher was presented with no NVIDIA runtime. Seven host-side checks cover GPU preference, a missing runtime, a failed NVIDIA probe, no devices, the first-build image probe, argument forwarding, and preservation of Docker/Compose errors. The rebuilt image also passed the caster differential robot smoke test.

Run the launcher checks from the repository root with `python3 flatland/tests/docker_launcher.py`; they simulate Docker responses and need no Docker daemon or GPU.

The wheel check passed for cylindrical tire dimensions, floor clearance and visible rolling in both directions. All four robot runtime smoke tests also passed after the rendering update. A close RViz inspection showed blue 2D scans, red 3D points, visible tires/rims and an OK global status at approximately 31 FPS with NVIDIA rendering. Its expected rotation is signed distance divided by configured radius. The updated models preserve their prior collision footprints, poses, densities and sensor elevations.

```bash
docker run --rm --network none -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp flatland2:humble-fork \
  python3 /flatland_ws/install/flatland/share/flatland/tests/wheel_visuals.py
docker run --rm flatland2:humble-fork bash -c '
  g++ -O3 -std=c++17 -pthread -I/flatland_ws/install/flatland_plugins/include \
    /flatland_ws/install/flatland/share/flatland/tests/pcd_sampler.cpp -o /tmp/pcd_sampler_check &&
  /tmp/pcd_sampler_check /flatland_ws/install/flatland/share/flatland/maps/warehouse/warehouse.pcd'
```

## Drive-specific teleop and omni casters

The three installed teleops (`diff_teleop.py`, `omni_teleop.py`, and `ackermann_teleop.py`) were exercised through pseudo-terminals and ROS subscriptions. Checks cover the documented movement keys, smooth ramps, key-repeat delay, release timeout, speed adjustment, ROS namespaces, terminal restoration, and final stop on Ctrl-C, Q, or SIGTERM as appropriate. The Ackermann test also covers WASD, split arrow-key escape sequences, persistent commands, speed/steering limits, Space braking and Tab centering.

An end-to-end omni check drives the actual robot with A/D, W/X and J/L. Both strafe directions preserve heading, forward/reverse motion follows the body frame, and rotation stays about the body center. The two new opposite-corner casters align with pivot motion; all four wheels have cylindrical tire visuals. Stopping retains the powered turrets' steering angles. The namespaced omni smoke test also passed with external localization.

```bash
docker run --rm --network none -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp flatland2:humble-fork \
  python3 /flatland_ws/install/flatland/share/flatland/tests/teleop_controls.py
docker run --rm --network none -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp flatland2:humble-fork \
  python3 /flatland_ws/install/flatland/share/flatland/tests/omni_motion.py
```

## 3D lidar refresh rate

The supplied robots now request 20 Hz instead of 5 Hz. Dense PCD sampling runs asynchronously with one scan in flight, while dynamic-object snapshots and publishing stay on the physics thread. Clouds retain their capture timestamps, and the publisher and RViz use a queue depth of one.

Measurements on the development host, with the omni robot moving and `/lidar_points` subscribed:

| Configuration | Clouds per wall-clock second | Simulation real-time factor |
| --- | --- | --- |
| Previous 5 Hz configuration | 4.63 | 0.93 |
| 20 Hz with blocking sampling | 14.86 | 0.74 |
| 20 Hz with asynchronous sampling | 19.95 | 1.00 |

The full 2,277,784-point warehouse map and 360 × 16 scan layout are unchanged. The updated run produced 5,760 returns per scan; its 95th-percentile arrival interval was about 60 ms. Timings depend on CPU load and hardware. A slower sampler skips capture periods rather than building a scan queue or delaying physics.

The snapshot test reconstructed 60 moving scans into known world points using their capture-time poses, including a translated and rotated sensor mount. Live sensor deletion completed successfully. The forklift smoke test passed with asynchronous lidar during motion, attachment, lifting, detachment, deletion, and pause/resume.

```bash
docker run --rm --network none -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp flatland2:humble-fork \
  python3 /flatland_ws/install/flatland/share/flatland/tests/lidar_cadence.py
docker run --rm --network none -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp flatland2:humble-fork \
  python3 /flatland_ws/install/flatland/share/flatland/tests/lidar_snapshot.py
```
