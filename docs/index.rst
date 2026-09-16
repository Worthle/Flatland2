Welcome to Flatland's documentation!
====================================
Flatland is a performance centric 2D robot simulator started at Avidbots Corp.. 
It is intended for use as a light weight alternative to Gazebo Simulator for 
ground robots on a flat surface. Flatland uses Box2D for physics simulation and 
it is built to integrate directly with ROS. Flatland loads its simulation 
environment from YAML files and provide a plugin system for extending its 
functionalities. 

The code is open source and `available on Github <https://github.com/avidbots/flatland>`_,
BSD3 License.

Class APIs are documented `here <http://flatland-simulator-api.readthedocs.io/>`_.

Flatland 2 adds drive, terrain, sensor, and payload plugins, plus body extrusion,
mesh and wheel visuals in RViz2. See :doc:`included_plugins/plugin_catalog`
and :doc:`visualization` for configuration.

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   overview
   quick_start
   visualization

.. toctree::
   :maxdepth: 2
   :caption: Flatland Tutorials

   flatland_tutorials/create_plugin.rst
   flatland_tutorials/create_model.rst
   flatland_tutorials/spawn_model.rst
   flatland_tutorials/custom_robot.rst
   
.. toctree::
   :maxdepth: 2
   :caption: Core Functionalities

   core_functions/ros_launch
   core_functions/world
   core_functions/layers
   core_functions/models
   core_functions/yaml_preprocessor
   core_functions/ros_services
   core_functions/model_plugins
   core_functions/joystick


.. toctree::
   :maxdepth: 2
   :caption: Built-in Plugins

   included_plugins/plugin_catalog
   included_plugins/drive_options
   included_plugins/bumper
   included_plugins/bool_sensor
   included_plugins/diff_drive
   included_plugins/diff_drive_caster
   included_plugins/tricycle_drive
   included_plugins/tricycle_drive_ackermann
   included_plugins/omni_drive
   included_plugins/system_id_drive
   included_plugins/laser
   included_plugins/imu
   included_plugins/mock_detection
   included_plugins/mock_lidar3d
   included_plugins/model_tf_publisher
   included_plugins/tween
   included_plugins/gps
   included_plugins/forklift
   included_plugins/link_attacher
   included_plugins/fork_controller
   included_plugins/terrain_modifier
   included_plugins/random_wall
