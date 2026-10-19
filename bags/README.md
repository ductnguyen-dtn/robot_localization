# bags/

## pf_gauntlet_demo

A 45 second recording of the particle filter localizing the Neato in the
`neato2_gazebo` gauntlet world, against a map built from the same world
with `slam_toolbox` (`maps/neato_gauntlet.yaml`).

The robot drives a loop (forward, turn, forward, turn, forward) while the
filter runs. Topics recorded:

- `/odom`, `/accel`, `/bump`, `/cmd_vel`, `/scan`, `/stable_scan`: the
  robot's normal sensor and command topics.
- `/tf`, `/tf_static`: includes the filter's own `map -> odom` correction,
  not just `odom -> base_link`.
- `/particle_cloud`: the filter's 300 particles and their weights, at
  every update.
- `/map`: the map the filter is localizing against.

To play it back and watch the particle cloud converge in RViz:

```bash
ros2 bag play bags/pf_gauntlet_demo --clock
```

Add displays for `/particle_cloud`, `/map`, `/tf`, and `/scan` (set the
fixed frame to `map`). The robot starts right where the map's origin is
(both were anchored to the same spawn pose), so the particle cloud should
start as a wide scatter and visibly tighten within the first few seconds
as it locks onto the walls.
