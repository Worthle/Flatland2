# AWS warehouse reference data

Source: [AWS RoboMaker Small Warehouse World](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world/tree/3c23a698bf0b4e366ddf8b084af507c519bd3483), `worlds/no_roof_small_warehouse.world`. The original repository is archived; this package vendors the required source assets at a fixed revision. See [source/LICENSE](source/LICENSE).

The converter loads each instance's collision meshes, applies COLLADA scene transforms and centimetre-to-metre units, then applies SDF mesh scale and model/link/collision poses. It samples mesh surfaces with a deterministic barycentric lattice and deduplicates nearby samples. The binary PCD contains float32 `x y z`; there is no synthetic extrusion of the PGM into a cloud.

The PGM uses the same transformed triangles, clipped to heights 0.12–1.8 m, with a conservative projection onto 5 cm cells. World coordinates are unchanged, Z points upward, and image row zero is the largest Y. Free cells are 254, occupied cells 0. The half-metre exterior margin is represented as free space; walls bound the navigable warehouse. Floor and overhead geometry remain in the PCD. Dynamic demo objects are spawned separately and are not baked into either reference map.

The three output files, geometry counts, bounds, revision, and SHA-256 digests are recorded in [provenance.json](provenance.json). The bundled SDF and meshes are reference data; Gazebo is not needed to run Flatland2.

Regenerate from the repository root using the built image:

```bash
docker run --rm --user "$(id -u):$(id -g)" \
  -v "$PWD/flatland/maps/warehouse:/data" flatland2:humble-fork \
  python3 /flatland_ws/install/flatland/share/flatland/tools/generate_warehouse.py \
  --source /data/source --output /data
```

Commit the generated PCD, PGM, YAML, and provenance together. The occupancy projection intentionally cannot represent shelf clearances or arbitrary 3D collision behavior.
