FROM ros:humble AS dependencies
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-colcon-common-extensions python3-yaml \
    python3-numpy python3-scipy python3-opencv python3-pip \
    libboost-all-dev libyaml-cpp-dev liblua5.1-0-dev libopencv-dev libeigen3-dev \
    ros-humble-pluginlib ros-humble-rmw-cyclonedds-cpp \
    ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
    ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-sensor-msgs \
    ros-humble-visualization-msgs ros-humble-interactive-markers \
    ros-humble-ackermann-msgs ros-humble-rviz2 ros-humble-rviz-common \
    ros-humble-rviz-default-plugins ros-humble-rviz-rendering qtbase5-dev \
    ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir pycollada==0.9.3
FROM dependencies AS runtime
WORKDIR /flatland_ws
COPY flatland_core/ src/flatland_core/
COPY flatland_msgs/ src/flatland_msgs/
COPY flatland_server/ src/flatland_server/
COPY flatland_plugins/ src/flatland_plugins/
COPY flatland_viz/ src/flatland_viz/
ARG BUILD_JOBS=2
RUN source /opt/ros/humble/setup.bash && \
    MAKEFLAGS=-j${BUILD_JOBS} colcon build --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release && rm -rf build log
COPY flatland/ src/flatland/
RUN source /opt/ros/humble/setup.bash && source install/setup.bash && \
    colcon build --base-paths src/flatland --executor sequential && rm -rf build log
COPY docker-entrypoint.sh /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["ros2", "launch", "flatland", "simulation.launch.py", "show_viz:=false"]
