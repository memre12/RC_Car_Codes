# lattice_planner_pkg

Local obstacle-avoidance planning for the RC race car. Three nodes:

```
/scan ──> scan_to_grid ──> /occupancy_grid ──┬──> mission_planner ──> /switch_to_centerline
                                             │                              │
/path (centerline, latched) ────────────────>┴──> lattice_planner_node ────┘
/pf/pose/odom ──────────────────────────────────>          │
                                                           v
                                                    /selected_path ──> race_car_controller
```

- **scan_to_grid** — converts each laser scan into a local occupancy grid
  centered on the robot (`base_link`). Obstacle endpoints are marked lethal
  (100) and inflated with a circular mask (`inflation_radius`, cost
  `inflation_cost`). No ray tracing: the planner only needs lethal cells.
- **mission_planner** — checks a corridor (`path_width`) around the
  centerline up to `lookahead_distance` ahead of the car against the grid,
  and publishes the mode switch at 10 Hz: `true` = path clear, follow the
  centerline; `false` = obstacle, engage the lattice planner.
- **lattice_planner_node** — in centerline mode forwards the global path
  unchanged (including immediately on the lattice→centerline transition).
  In lattice mode it transforms a window of the centerline into the robot
  frame, generates laterally offset candidate trajectories (cubic Hermite
  splines), collision-checks them against the grid and publishes the best
  one back in the global frame.

## Candidate selection (Lattice::Generator)

1. If the centerline candidate (offset 0) is clear → take it, always.
2. Otherwise keep the previously chosen lateral offset if still clear
   (temporal consistency, avoids oscillating between sides).
3. Otherwise the cheapest clear candidate: cost = |offset|, ×0.8 if on the
   same side as the current offset, + distance from the current offset.
4. If everything is blocked → fall back to the centerline instead of
   stopping (blocked candidates are usually false positives at this point).

## Parameters

### lattice_planner_node

| Parameter | Default | Meaning |
|---|---|---|
| `planner_frequency` | 20.0 | max replans per second (rate-limits high-rate odometry) |
| `planner_horizon` | 10.0 | forward window of the centerline used (m) |
| `centerline_window_behind/ahead` | 10 / 100 | path indices kept around the closest point |
| `min/max_speed` | 0.5 / 10.0 | m/s range for lookahead scaling |
| `min/max_lookahead` | 3.0 / 15.0 | anchor-point distance range (m) |
| `lethal_threshold` | 50 | grid cost above which a cell blocks a path |
| `path_resolution` | 0.05 | spline sample spacing (m) |
| `candidate_offsets` | ±1.0 … 0.0 | lateral offsets of the candidates (m) |

### scan_to_grid

| Parameter | Default | Meaning |
|---|---|---|
| `grid_resolution` | 0.05 | m/cell |
| `grid_size_x/y` | 6.0 | grid extent (m); launch file uses 10.0 |
| `inflation_radius` | 0.15 | obstacle inflation (m); vehicle half-width is 0.158 |
| `inflation_cost` | 80 | cost written into inflated cells |

### mission_planner

| Parameter | Default | Meaning |
|---|---|---|
| `obstacle_threshold` | 50.0 | grid cost that counts as an obstacle; launch uses 10.0 |
| `lookahead_distance` | 5.0 | corridor length ahead of the car (m); launch uses 4.0 |
| `path_width` | 1.5 | corridor width (m); launch uses 0.5 |

## Performance notes (Jetson Xavier 16 GB)

The package is tuned to run comfortably alongside SLAM/particle filter:

- **Release build by default.** The CMakeLists forces
  `CMAKE_BUILD_TYPE=Release` when unset — an unoptimized build was the
  single biggest source of slowness on the Jetson.
- **Rate-limited planning.** Planning runs at most `planner_frequency`
  (20 Hz) regardless of how fast odometry arrives.
- **No per-point quaternion math.** Yaw is extracted once per cycle, not
  once per path pose; all frame transforms reuse precomputed sin/cos.
- **No repeated allocations.** scan_to_grid reuses its grid buffer and
  caches the per-beam sin/cos table; the inflation and corridor masks are
  precomputed offset lists instead of nested loops with `hypot` per cell.
- **No parameter lookups in hot paths.** mission_planner reads its
  parameters once at startup (each `get_parameter` call takes a lock).
- **Debug topics are free when unused.** Candidate and closest-point
  markers are only built if something subscribes to them.
- The unused TF listener was removed from the planner node — it was
  buffering the whole `/tf` stream for nothing.

Build:

```
colcon build --packages-select lattice_planner_pkg --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Run:

```
ros2 launch lattice_planner_pkg lattice_planner.launch.py
```
