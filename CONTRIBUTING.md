# Contributing to Flatland 2

Target ROS 2 Humble on Ubuntu 22.04. This repository contains the five core ROS
packages and their documentation. Robot-specific launch files, maps, models,
teleop and integration tests belong in the separate `flatland_examples` checkout.
Keep public interfaces compatible where practical and document intentional changes.

## Development

Place the core checkout under your workspace's `src/` directory. Optionally
place the examples checkout beside it. From the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The [README](README.md) describes core Docker builds and world loading.
The examples README describes demo launches. Core runtime dependencies must not
point back to `flatland_examples`.

## Before submitting changes

Rebuild the core image and run the checks in [validation.md](docs/validation.md).
The Humble workflow builds the five packages and runs the core server smoke
and NARX replay checks. For changes affecting robot behavior, also run the
relevant companion integration tests. The historical ROS 1 rostests are not a
working ROS 2 test suite.

Follow the surrounding code style and `.clang-format`. Older developer scripts
reference clang-format-3.8 and clang-tidy-3.8; the Docker build does not supply
those tools or claim to run their checks. Avoid mass formatting unrelated files.

Retain copyright headers and licenses in derived and vendored code. Update
[third-party notices](flatland/THIRD_PARTY_NOTICES.md) when adding dependencies.
Keep core legal notices in `flatland/` so they are installed with the metapackage.
Build the [Sphinx documentation](docs/README.md) after editing plugin references.

For upstream contributions, use a focused branch based on the branch agreed
with Avidbots' maintainers. See [UPSTREAM.md](UPSTREAM.md) for this fork's scope.
