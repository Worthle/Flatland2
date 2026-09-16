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

.. note::

   This branch contains the Flatland 2 ROS 2 Humble extensions. Start with the
   :doc:`included_plugins/plugin_catalog` for the current plugin inventory,
   including the types added since upstream Humble. The new plugin references
   describe the public implementation; some inherited tutorials retain ROS 1
   commands. Use the repository README for the current Docker and ROS 2 launch
   workflow. Robot assets and teleop live in the separate ``flatland_examples``
   companion package; plugin configuration examples referring to that package
   require it to be installed.

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   overview
   quick_start

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
