# Contributing to Flatland 2

Target ROS 2 Humble on Ubuntu 22.04. The `flatland2` branch contains the Box2D
simulator, plugins and demo assets. Keep ROS package names and public interfaces
compatible where practical, and document intentional changes.

## Development layout

The six ROS packages live at the repository root. For a native colcon build,
place this repository in a workspace's `src/` directory, source ROS 2 Humble,
and run from the workspace root:

```bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch flatland simulation.launch.py show_viz:=false
```

The Docker workflow in [README.md](README.md) supplies the supported dependency
environment and does not need a host ROS installation.

## Before submitting changes

Run `python3 flatland/tests/docker_launcher.py`, rebuild the Docker image,
and run the asset and relevant runtime checks in
[validation.md](flatland/docs/validation.md). The Humble GitHub Actions workflow
builds all packages and runs the asset, dynamics and TurtleBot spawn checks.
Add focused tests when changing behavior; the historical ROS 1 rostests are not
a working ROS 2 test suite.

Follow the surrounding code style and the upstream `.clang-format` for C++.
The original contribution guide requested clang-format-3.8 and clang-tidy-3.8;
the current Docker workflow does not supply those old tools or claim to run
their checks. Avoid mass formatting unrelated files.

Retain copyright headers and licenses in derived and vendored code. Include
the source and license of new assets and update the
[third-party notices](flatland/docs/THIRD_PARTY_NOTICES.md) when needed.

For upstream contributions, use a focused branch based on the branch agreed
with Avidbots' maintainers. See [UPSTREAM.md](UPSTREAM.md) for the scope and
history of this larger fork.
