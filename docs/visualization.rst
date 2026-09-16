Visualization
=============

Flatland 2 displays polygon bodies as filled prisms, supports mesh resources,
and animates wheels, forks, and carried payloads. These visuals use the planar
Box2D body pose with an additional height; collisions remain two-dimensional.

Body appearance
---------------

Add these fields to a body in a model YAML file:

.. code-block:: yaml

   bodies:
     - name: base_link
       type: dynamic
       color: [0.2, 0.5, 0.8, 1.0]
       extrude: 0.4
       elevation: 0.1
       visual_z_offset: 0.0
       footprints:
         - type: polygon
           density: 1.0
           points: [[-0.4, -0.3], [0.4, -0.3], [0.4, 0.3], [-0.4, 0.3]]

``extrude`` sets the height of polygon visuals in metres. ``elevation`` raises
the body; fork and payload plugins can change it at runtime. ``visual_z_offset``
shifts only the rendered geometry, leaving collisions, sensor height, and TF
unchanged. These fields default to zero. The server's ``default_extrude_height``
parameter supplies a fallback when a body has no positive extrusion height.

For a mesh, use ``visual_mesh: package://my_package/meshes/body.dae``.
The mesh uses its embedded materials and unit scale; footprints still determine
planar collisions. ``visual_z_offset`` can align the mesh with the body.

Wheel visuals
-------------

A wheel body can use this instead of ``visual_mesh``:

.. code-block:: yaml

   wheel_visual:
     radius: 0.1
     width: 0.06
     center: [-0.08, 0.0]

Radius and width must be positive. ``center`` locates the axle relative to the
body origin; its default is ``[0, 0]``. An offset axle also displays a caster
bracket. Tires, rims, hubs, spokes, and tread turn with the wheel's travelled
distance. Steering follows the body's planar orientation. A body can use either
``wheel_visual`` or ``visual_mesh``.

Forks and payloads
------------------

:doc:`included_plugins/forklift` changes fork elevation.
:doc:`included_plugins/link_attacher` makes an attached payload follow the forks
and settle after release. :doc:`included_plugins/fork_controller` adds height
goals and payload feedback.

RViz2
-----

Start the server with ``show_viz:=true`` as a ROS parameter to publish debug
markers, then run ``ros2 run flatland_viz flatland_viz``. The application adds
MarkerArray displays for topics announced on ``/flatland_server/debug/topics``.
Use ``map`` as the fixed frame. Spawn and pause tools are available in the toolbar.

The examples launch starts both the simulator and RViz2:

.. code-block:: bash

   ros2 launch flatland_examples simulation.launch.py robot:=omni_drive_robot show_viz:=true
