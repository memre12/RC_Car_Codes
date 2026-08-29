# race_car_controller

Pure pursuit path-tracking controller for the RC race car (Jetson Xavier + VESC).

## What it does

Follows the path published by the lattice planner (`/selected_path`) using
localized odometry from the particle filter, and drives the car through the
VESC:

```
/selected_path (nav_msgs/Path)   ─┐
/pf/pose/odom  (nav_msgs/Odometry)├─> race_car_controller ──> /commands/servo/position (0..1)
/target_speed  (std_msgs/Float64) ┘         │                 /commands/motor/speed    (ERPM)
/joy           (sensor_msgs/Joy) ───────────┘  (autonomous on/off toggle, button 0)
```

While autonomous mode is off, every joystick message publishes neutral servo
(`servo_center`) and zero ERPM so the car stays stopped.

## Steering law

Classic pure pursuit with an explicit proportional gain, expressed in the
real vehicle's parameters (see `race_car_parameters.txt`):

```
alpha = heading error to the lookahead point            [rad]
Ld    = actual distance to the lookahead point          [m]
delta = atan2(2 * wheelbase * sin(alpha), Ld)           geometric Ackermann angle
u     = clamp(steering_gain * delta / max_steer_angle, -1, 1)
servo = servo_center + u * (servo_max - servo_min) / 2
```

- `wheelbase = 0.28 m` and `max_steer_angle_deg = 29.85` are the measured
  values of the car.
- `steering_gain = 1.0` would be the pure geometric mapping (a computed
  Ackermann angle of 29.85° = full servo lock).
- The default `steering_gain = 2.35` **exactly reproduces the response tuned
  on track with the previous implementation** (which used `theta * 2.2` on a
  ±45° scale with a 0.45 m wheelbase): the servo slope per degree of heading
  error and the saturation point (full lock at δ ≈ 12.7°) are identical.
  Lower it towards 1.0 for calmer steering, raise it for more aggressive
  turn-in.

## Speed handling (m/s everywhere, ERPM only at the VESC boundary)

`/target_speed` is in **m/s**. It both feeds the motor command and scales
the lookahead distance linearly:

```
Ld = lookahead_distance * (target_speed / min_speed)
Ld is clamped to [min_look_ahead_distance, max_look_ahead_distance]
```

With the defaults (`lookahead_distance = 1.3`, `min_speed = 0.6`):
0.6 m/s → 1.3 m, 1.1 m/s → 2.4 m (upper clamp).

The conversion to ERPM happens only when publishing to the VESC:

```
ERPM = v * 60 * (motor_poles / 2) * gear_ratio / (2π * wheel_radius)
```

VESC ERPM is electrical RPM = mechanical RPM × pole **pairs** (the formula
originally noted in `race_car_parameters.txt` divided by the pole count and
underestimated speed by 2×). With the measured drivetrain (14 poles, gear
2.769, wheel radius 0.055 m): **1 m/s = 3365.4 ERPM**, so the old working
range 2000–3000 ERPM is 0.6–0.9 m/s. The VESC speed feedback on
`/sensors/core` is converted back to m/s with the same factor.

## Parameters (config/params.yaml)

| Parameter | Default | Meaning |
|---|---|---|
| `wheelbase` | 0.28 | meters, measured |
| `max_steer_angle_deg` | 29.85 | physical steering limit, measured |
| `steering_gain` | 2.35 | P gain on the pure pursuit angle |
| `servo_min` / `servo_max` / `servo_center` | 0.2 / 0.8 / 0.5 | VESC servo range |
| `lookahead_distance` | 1.3 | Ld at `min_speed` (m) |
| `min/max_look_ahead_distance` | 0.6 / 2.4 | Ld clamp (m) |
| `min_speed` | 0.6 | m/s anchor for the Ld scaling (~2000 ERPM) |
| `max_speed` | 1.8 | m/s safety ceiling for motor commands (~6000 ERPM) |
| `motor_poles` | 14 | drivetrain, for the m/s → ERPM conversion |
| `gear_ratio` | 2.769 | drivetrain, for the m/s → ERPM conversion |
| `wheel_radius` | 0.055 | meters, for the m/s → ERPM conversion |
| `toggle_cooldown_sec` | 0.5 | joystick toggle debounce |

Topic names are also parameters (`path_topic`, `odom_topic`,
`servo_topic_pub`, `motor_topic_pub`, `target_speed_topic`,
`vesc_state_topic`, `joy_topic`).

## Performance notes (Jetson Xavier)

- Build in Release (the CMakeLists now defaults to it):
  `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release`
- The per-callback `RCLCPP_INFO` in the servo mapping was removed — console
  I/O at odometry rate was a real source of latency spikes.
- The RViz lookahead marker is only serialized when something subscribes to
  it, so it costs nothing while racing.
- The closest-point search uses squared distances (no `sqrt` per point) and
  the lookahead walk is bounded, so a degenerate path cannot hang the node.

## Run

```
ros2 launch race_car_controller launcher.launch.py
```
