# Core validation

The core builds and runs independently of `flatland_examples`. Its Docker image
contains five packages: `flatland`, `flatland_msgs`, `flatland_server`,
`flatland_plugins`, and `flatland_viz`.

## Reproduce

```bash
docker build --build-arg BUILD_JOBS=2 -t flatland2:humble-core .
docker run --rm --network none -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  flatland2:humble-core python3 /opt/flatland-tests/server_smoke.py
docker run --rm --network none flatland2:humble-core \
  python3 /opt/flatland-tests/dynamics.py
```

`server_smoke.py` loads a tiny line-segment room and a circular robot from
`tests/fixtures/`. It checks plugin loading, laser ranges, commanded motion,
odometry and pause/resume. `dynamics.py` compares NARX forward/reverse/stop
responses with an analytic recurrence and checks CSV error handling. Both scripts
can also be run directly after sourcing a native Humble workspace.

The companion examples own asset validation and the broader robot, TF, payload,
teleop, wheel and lidar checks. See their
[validation guide](../../flatland_examples/flatland_examples/docs/validation.md).

## Limits

These checks do not cover every plugin or calibrate the physics. They do not
exercise graphical RViz rendering or an external localization/fleet stack.
The historical ROS 1 rostests are not a working ROS 2 test suite.

## Split verification — 16 September 2026

A fresh Docker build completed all five core packages with the demo assets and
Nav2 bringup dependencies removed from this repository. The core image passed
both runtime scripts above. Its ROS package index contained exactly the five
core packages, and the metapackage's component notices and four bundled license
files were installed successfully.

The companion image then built from that core image. Installed asset validation,
all four robot smoke scenarios, and all three teleop checks passed. The tests
included namespaced TurtleBot spawn/delete, omni external localization and
namespaced forklift NARX propulsion with payload operations. The host Docker
launcher checks and Compose configuration passed. Package discovery and manifests
confirmed that only the examples depend on the core.

The strict Sphinx HTML build, current Markdown links, Python syntax, YAML/XML and
package asset references passed. Tests ran in isolated headless containers; GUI
rendering and external autonomy stacks were not retested. The workflows were
updated locally but have not been run on GitHub.
