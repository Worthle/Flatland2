# Upstream history and branch scope

## Origin

- Upstream repository: <https://github.com/avidbots/flatland>.
- Integration base: `ros2-humble` at
  [`fc3f233ed1a8009658f9eed827d1adea1cf6a511`](https://github.com/avidbots/flatland/commit/fc3f233ed1a8009658f9eed827d1adea1cf6a511).
- Public branch name: `flatland2`.
- Scope: ROS 2 Humble, bundled Box2D, robot plugins, mock sensors,
  RViz2 and core Docker builds.

The local Flatland 2 workspace had no commits or Git remote when this branch was
prepared. Its Flatland source was imported onto the existing upstream Humble
history. This preserves a common ancestor for Git comparisons and pull requests;
it does not reconstruct the development history of the imported files or claim
that this commit was their original source revision.

The [ROS 1 comparison](docs/ros1-comparison.md) uses a separate pinned
`master` revision as a feature baseline. Upstream already has ROS 2 branches;
ROS 2 support should not be described as originating entirely in this fork.

## Repository layout

| Directory | Purpose |
| --- | --- |
| `flatland/` | Core metapackage and installed licenses/notices |
| `flatland_msgs/` | ROS messages and services |
| `flatland_server/` | Box2D simulation server |
| `flatland_plugins/` | Drive, sensor and world plugins |
| `flatland_viz/` | RViz2 application and tools |
| `tests/` | Independent core runtime checks and small synthetic fixtures |
| `docs/`, `scripts/` | Plugin references, inherited documentation and developer utilities |

The metapackage retains upstream's `flatland` name and depends on the four
simulator packages. The separate `flatland_examples` package depends on it and
owns robot models, maps, objects, launch files, teleop and demo integration tests.
The core has no dependency on the examples. The earlier `flatland_core` name is
no longer used. The inherited `2.0.0` package versions do not indicate an official
Avidbots release.

The import replaces package contents with the current Flatland 2 implementation.
It includes the earlier removal of ROS 1-only tests and examples described in
the [comparison](docs/ros1-comparison.md). The old `flatland_box2d` and `flatland_rviz_plugins` packages are replaced by
the bundled server physics and the tools in `flatland_viz`, respectively.
The outdated Travis job is removed and CI builds the core independently and runs ROS 2 runtime checks.

LevPhysics, LevSim, local editor settings and generated build output are outside
this branch. Flatland 2 runs without the separate `Lev/` project. Its 3D visuals
and mock sensors still operate around planar Box2D physics.

## License and attribution

The [BSD 3-Clause license](LICENSE) permits redistribution and modification under
its conditions. Preserve Avidbots' copyright, the license conditions and
disclaimer, along with the separate licenses for bundled dependencies and assets.
The existing contribution copyright identifies Levent Soysal's Flatland 2 work.
Do not imply endorsement or an official successor release. Full component notices
are in [THIRD_PARTY_NOTICES.md](flatland/THIRD_PARTY_NOTICES.md).

## Relationship to Avidbots

A GitHub fork can carry this branch independently. Creating a branch inside
`avidbots/flatland` requires upstream write access or a maintainer to create it.
A pull request proposes a merge into an existing target branch; it does not
automatically create an upstream branch called `flatland2`.

This is a substantial simulator extension. For upstream review, agree on the
target and scope with maintainers, then extract focused fixes or features into
separate branches. The prepared `flatland2` branch can be shared on its own
without an upstream merge.
