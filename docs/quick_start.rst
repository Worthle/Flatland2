.. image:: _static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: _static/flatland_logo2.png

Quick Start
====================================
For ready-to-run robots, use ``flatland_examples`` and launch
``ros2 launch flatland_examples simulation.launch.py show_viz:=true``.

Flatland uses YAML files to setup the simulation, much like how Gazebo uses URDF
files.

1. Create a YAML file called world.yaml with the following content

   Here we define a world with a layer called "map" and one model "turtlebot". See 
   :doc:`core_functions/world` for more details.

  .. code-block:: yaml

    properties: {}
    layers:
      - name: "map" 
        map: "conestogo_office.yaml"
        color: [0.4, 0.4, 0.4, 1]
    models:  
      - name: turtlebot 
        model: "turtlebot.model.yaml"

2. One way of defining a layer map is to use data generated from 
   `ROS map_server <http://wiki.ros.org/map_server>`_, if you have map from 
   map_server you can replace map.yaml  with the path to the YAML file from map 
   server, and skip the rest of this section. See :doc:`core_functions/layers` 
   for more details.

   Otherwise, create a file named conestogo_office.yaml with the following 
   content, download the map image below, and place both the yaml file and the
   image in the same directory as world.yaml

  .. code-block:: yaml

    image: conestogo_office.png
    resolution: 0.050000
    origin: [-16.600000, -6.650000, 0.000000]
    negate: 0
    occupied_thresh: 0.65
    free_thresh: 0.196

  .. image:: _static/conestogo_office.png
    :target: _static/conestogo_office.png 

3. To define a model, create a file named turtlebot.model.yaml with the following
   content and place it in the same directory as world.yaml. For more details 
   about the model yaml format, see :doc:`core_functions/models`.

  .. code-block:: yaml

    bodies:  
      - name: base
        pose: [0, 0, 0] 
        type: dynamic  
        color: [1, 1, 1, 0.4] 

        footprints:
          - type: circle
            radius: 0.5
            density: 1

          - type: polygon
            points: [[-.45, -.05], [-.45, 0.05], [-.35, 0.05], [-.35, -0.05]]
            density: 1

          - type: polygon
            points: [[-.125, -.4], [-.125, -.3], [.125, -.3], [.125, -.4]]
            density: 1

          - type: polygon
            points: [[-.125, .4], [-.125, .3], [.125, .3], [.125, .4]]
            density: 1            
              
    plugins:
      - type: DiffDrive 
        name: turtlebot_drive 
        body: base

4. After building the core and sourcing the workspace, start the server with
   the absolute path to world.yaml. You can send `Twist <http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html>`_
   commands to /cmd_vel to move the robot.

  .. code-block:: bash

    ros2 run flatland_server flatland_server --ros-args \
      -p world_path:=/absolute/path/to/world.yaml \
      -p update_rate:=100.0 -p step_size:=0.01