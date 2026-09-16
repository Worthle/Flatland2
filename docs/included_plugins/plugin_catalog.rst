.. image:: ../_static/flatland_logo2.png
    :width: 250px
    :align: right
    :target: ../_static/flatland_logo2.png

Plugin Catalog for Flatland 2
=============================

This catalog covers the 20 plugin types registered in this branch's
``flatland_plugins/flatland_plugins.xml``. The comparison baseline is
Avidbots' ``ros2-humble`` at commit
``fc3f233ed1a8009658f9eed827d1adea1cf6a511``, whose registry is named
``flatland_plugins/plugin_description.xml``.

All nine baseline plugin types remain available. Eleven additional types are
registered in Flatland 2. This describes availability relative to that commit;
it does not claim that every additional type was independently invented here
or absent from every other upstream branch. Source copyright notices retain
their own attribution.

.. list-table::
   :header-rows: 1
   :widths: 28 22 50

   * - Plugin type / reference
     - Compared with Humble
     - Purpose
   * - :doc:`BoolSensor <bool_sensor>`
     - Retained
     - Boolean collision feedback
   * - :doc:`Bumper <bumper>`
     - Retained
     - Contact information
   * - :doc:`DiffDrive <diff_drive>`
     - Retained and extended
     - Differential drive, including optional OE response models
   * - :doc:`DiffDriveCaster <diff_drive_caster>`
     - Additional
     - Differential drive with passive caster effects
   * - :doc:`ForkController <fork_controller>`
     - Additional
     - Absolute fork-height goals and simplified payload feedback
   * - :doc:`Forklift <forklift>`
     - Additional
     - Fractional lift commands and fork elevation animation
   * - :doc:`Gps <gps>`
     - Retained
     - Simulated GPS receiver
   * - :doc:`Imu <imu>`
     - Additional
     - Planar inertial measurements
   * - :doc:`Laser <laser>`
     - Retained and extended
     - 2D laser scanning, including body/elevation filtering
   * - :doc:`LinkAttacher <link_attacher>`
     - Additional
     - Weld, carry and release a payload model
   * - :doc:`MockDetection <mock_detection>`
     - Additional
     - Noisy visible-target poses and optional last-seen TFs
   * - :doc:`MockLidar3D <mock_lidar3d>`
     - Additional
     - Point-cloud map sampling or extruded 2D hits
   * - :doc:`ModelTfPublisher <model_tf_publisher>`
     - Retained
     - Transforms between model bodies
   * - :doc:`OmniDrive <omni_drive>`
     - Additional
     - Two independently steered wheel assemblies
   * - :doc:`RandomWall <random_wall>`
     - Retained; previously undocumented
     - World plugin that adds offset wall edges
   * - :doc:`SystemIdDrive <system_id_drive>`
     - Additional
     - Coupled NARX propulsion predictions
   * - :doc:`TerrainModifier <terrain_modifier>`
     - Additional
     - Planar slip, rubble and slope disturbances
   * - :doc:`TricycleDrive <tricycle_drive>`
     - Retained
     - Steering-wheel drive using Twist commands
   * - :doc:`TricycleDriveAckermann <tricycle_drive_ackermann>`
     - Additional
     - Steering-wheel drive using Ackermann commands
   * - :doc:`Tween <tween>`
     - Retained
     - Scripted motion between poses

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

The added pages describe the public Flatland 2 source, with portable names
and bundled example assets. Historical tutorials and the original plugin
pages elsewhere in this documentation may retain ROS 1 commands; use the
repository README for the ROS 2 Humble build and launch workflow.
