# Contributing

Target ROS 2 Humble. Document plugin parameters, ROS interfaces, and visualization options.

- Add GoogleTest unit tests where practical, using `ament_cmake_gtest`.
- Format C++ with `clang-format-3.8 --style=file`.
- Run `clang-tidy-14` on the ROS 2 C++17 compilation database.
- Preserve copyright headers and dependency licenses.

Build as described in the [README](README.md), then run:

```bash
colcon test --return-code-on-test-failure
colcon test-result --verbose
```

For the C++ checks, build with `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`,
then run `scripts/ci_prebuild.sh` from the repository and
`scripts/ci_postbuild.sh /path/to/workspace/build`.
