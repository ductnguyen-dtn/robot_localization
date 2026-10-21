# bags/

## pf_clean_demo (current, primary evidence)

A 220 second recording of the particle filter localizing the Neato in the
`neato2_gazebo` gauntlet world, against `maps/neato_gauntlet_clean.yaml`, a
map built by driving the robot manually (see
[Challenges and Next Steps](../README.md#challenges-and-next-steps) in the
main writeup for why manual driving replaced the scripted driving used for
the first map).

Topics recorded:

- `/odom`, `/cmd_vel`, `/scan`, `/stable_scan`, `/bump`: the robot's normal
  sensor and command topics.
- `/tf`, `/tf_static`: includes the filter's own `map -> odom` correction,
  not just `odom -> base_footprint`.
- `/particle_cloud`: the filter's 300 particles and their weights, at every
  update.
- `/map`: the map the filter is localizing against.

To play it back and watch the particle cloud converge in RViz:

```bash
ros2 bag play bags/pf_clean_demo --clock
```

Add displays for `/particle_cloud`, `/map`, `/tf`, and `/stable_scan` (set
the fixed frame to `map`). The cloud converges to a tight cluster within
the first few seconds and holds there for the rest of the recording; a
snapshot of this is in
[`media/particle_cloud_converged.png`](../media/particle_cloud_converged.png).

## pf_gauntlet_demo (earlier run, kept for comparison)

A 45 second recording from an earlier pass, made before two fixes
described in the main writeup: it runs against the first, since-superseded
`maps/neato_gauntlet.yaml` (which has scan-matching artifacts from
odometry corrupted by scripted driving hitting walls, see
[`media/map_before_corrupted.png`](../media/map_before_corrupted.png)), and
against a version of `pf.py` using `base_link` as a workaround for a frame
naming bug in `neato_packages` that has since been fixed upstream (`pf.py`
now uses the corrected `base_footprint` frame, matching the current
`neato_packages`).

Kept here rather than deleted so the "before" state in the writeup has
actual supporting data, not just a description.

```bash
ros2 bag play bags/pf_gauntlet_demo --clock
```
