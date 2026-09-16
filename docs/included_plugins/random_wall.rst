Random Wall
===========

``RandomWall`` is an inherited **world plugin**. It randomly chooses existing
wall edges in a layer and adds offset edges plus joining side walls, producing
extra planar wall geometry at world initialization. It was already registered
in Avidbots' ROS 2 Humble branch; this page fills a documentation gap.

Configuration
-------------

Put this entry in the world YAML's top-level ``plugins`` list, not in a model's
plugin list. The named layer and robot must already be defined in that world's
``layers`` and ``models`` sections.

.. code-block:: yaml

  plugins:
    - type: RandomWall
      name: random_walls
      layer: walls          # optional parser default ""; set an existing layer name
      robot_name: robot     # optional parser default ""; set a model name in the world
      num_of_walls: 2        # optional, default 0; number of existing edges to select
      wall_wall_dist: 0.5   # optional, default 1.0 m, coordinate offset
      double_wall: false    # optional, default false; add on both sides if true

``robot_name`` identifies a world model instance, not a namespace or plugin.
The plugin reads its initial ``pose`` from the world YAML and transforms it
into the layer frame to determine which side receives the offset geometry.
``double_wall`` also adds geometry on the opposite side. The robot's later
motion does not regenerate the walls.

Layer requirements and limits
-----------------------------

Use a layer whose fixtures are Box2D edges, such as a line-segment map. The
implementation casts every fixture in the selected layer to an edge shape;
polygon or circle fixtures are unsuitable. Set ``num_of_walls`` no larger
than the number of available edges: this bound is not checked before indexing.
A value of zero still requires a valid layer and robot.

The distance is a coordinate offset: axis-aligned walls are shifted along
one axis, while angled walls shift in both X and Y. It is not a uniform
perpendicular clearance for arbitrary wall angles. Keep the robot's initial
pose off the selected wall lines, and inspect the generated geometry before
using it as an environment.

Selection is seeded from the system clock, with no YAML seed option. The
plugin has no ROS topics or services and does not rewrite the occupancy map
or PCD file, so generated collision walls may disagree with separately
published map assets. It creates no height metadata or full 3D walls.
