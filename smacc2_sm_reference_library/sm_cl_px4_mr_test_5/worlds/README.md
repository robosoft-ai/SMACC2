# worlds/

The cave world is not vendored (it is ~100 MB of Gazebo Fuel tiles). Run

```
ros2 run sm_cl_px4_mr_test_5 fetch_cave_world.sh
```

once: it downloads Open Robotics' DARPA SubT *Cave Circuit Practice 01* (CC-BY-4.0) from
Gazebo Fuel into `~/.gz/fuel`, patches it for PX4 SITL with `scripts/patch_cave_world.py`
(Fuel host rename, artifact props and Ignition-only plugins dropped, `<spherical_coordinates>`
added) and writes `~/.gz/sm_cl_px4_mr_test_5/worlds/cave_circuit_practice_01.sdf`, where
`start_cave_sitl.sh` expects it (`SM5_WORLD_DIR` overrides the directory).

`scripts/cave_route_from_sdf.py <world.sdf> <world.dot>` prints the tile layout and the
tile chain from the base station, in gz ENU and in PX4 NED relative to the spawn point;
the route in `config/mission_constants.hpp` was authored from it.
