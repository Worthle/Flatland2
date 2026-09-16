.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Plugin Catalog for Flatland 2
=============================

Plugin types and configuration references:

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Plugin
     - Purpose
   * - :doc:`DiffDriveCaster <diff_drive_caster>`
     - Differential drive with passive caster effects
   * - :doc:`TricycleDriveAckermann <tricycle_drive_ackermann>`
     - Steering-wheel drive using Ackermann commands
   * - :doc:`OmniDrive <omni_drive>`
     - Two independently steered wheel assemblies
   * - :doc:`SystemIdDrive <system_id_drive>`
     - Coupled NARX propulsion predictions
   * - :doc:`TerrainModifier <terrain_modifier>`
     - Planar slip, rubble, and slope disturbances
   * - :doc:`Forklift <forklift>`
     - Fractional lift commands and fork elevation
   * - :doc:`LinkAttacher <link_attacher>`
     - Attach, carry, and release payloads
   * - :doc:`ForkController <fork_controller>`
     - Height goals and payload feedback
   * - :doc:`MockLidar3D <mock_lidar3d>`
     - Point-cloud sampling or extruded 2D hits
   * - :doc:`MockDetection <mock_detection>`
     - Visible target poses with noise and optional TFs
   * - :doc:`Imu <imu>`
     - Planar inertial measurements
   * - :doc:`DiffDrive <diff_drive>`
     - Differential drive with optional OE response models
   * - :doc:`Laser <laser>`
     - 2D scanning with body and elevation filters
   * - :doc:`BoolSensor <bool_sensor>`
     - Boolean collision feedback
   * - :doc:`Bumper <bumper>`
     - Contact information
   * - :doc:`Gps <gps>`
     - Simulated GPS
   * - :doc:`ModelTfPublisher <model_tf_publisher>`
     - Transforms between model bodies
   * - :doc:`TricycleDrive <tricycle_drive>`
     - Steering-wheel drive using Twist commands
   * - :doc:`Tween <tween>`
     - Scripted motion between poses
   * - :doc:`RandomWall <random_wall>`
     - Offset wall edges

Using the reference pages
-------------------------

Each model plugin goes in the model YAML's ``plugins`` list. ``RandomWall``
is the exception: it belongs in the world YAML's ``plugins`` list. Every entry
needs a case-sensitive ``type`` and a ``name`` unique within that model (or
among world plugins). Model plugins also support the common
``enabled: false`` switch to skip loading an entry.

Examples show plugin entries and expect the referenced bodies/joints to
already exist. Required fields and parser defaults are identified in the
comments. Example values that differ from the defaults are illustrative,
not calibrated vehicle parameters. Many parsers reject unknown YAML keys.

Unless a page says otherwise, distances are metres, angles radians, rates Hz,
and time intervals simulation seconds. Relative ROS topic/service names inherit
the model namespace. A leading slash makes a topic absolute; TF frame IDs
follow each plugin's frame-resolution rules rather than ROS topic remapping.

The simulator remains planar. Fork elevation and 3D sensor outputs do not add
six-degree-of-freedom vehicle physics. Choose one propulsion plugin per body,
and put ``TerrainModifier`` after it when combining them. Fork handling uses
``Forklift``, ``LinkAttacher`` and ``ForkController`` together.

