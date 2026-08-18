# Tuning Guide

Practical reference for tuning the Odin Navigation Stack (global planning, dynamic obstacle memory, NeuPAN). Use this guide to look up what a parameter does, in what range to move it, and which knob to reach for first when the robot misbehaves.

## Overview

The stack has three configuration surfaces:

| File | Owns |
|---|---|
| `NeuPAN/neupan/ros/configs/planner.yaml` | NeuPAN MPC weights, robot kinematics, optimizer settings |
| `NeuPAN/neupan/ros/configs/config.yaml` | NeuPAN ROS topics, odom feedback, prealign, stuck-escape |
| `ros_ws/src/map_planner/launch/whole.launch` | Global A\* args, fake360 args, goal state machine |

After editing:

| Change | Required action |
|---|---|
| Any `*.yaml` | Restart `neupan_ros.py` |
| `whole.launch` `<arg>` | Restart `roslaunch map_planner whole.launch` |
| C++ source under `map_planner/` or `fake360/` | `catkin_make --pkg map_planner` (or `--pkg fake360`), then restart |

## Parameters

### NeuPAN — `planner.yaml`

#### MPC framework

| Parameter | Type | Default | Description |
|---|---|---|---|
| `receding` | int | 8 | MPC horizon length (steps). Each step is `step_time` seconds. Larger = better lookahead, slower solve. |
| `step_time` | float | 0.1 | Horizon step size (s). Rarely changed. |
| `ref_speed` | float | 0.5 | Forward speed target (m/s). |
| `collision_threshold` | float | 0.05 | Distance below which a sample is treated as a collision (triggers escape). |

#### Robot kinematics (`robot:`)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `kinematics` | string | `"diff"` | Differential drive. |
| `max_speed` | `[v, w]` | `[0.5, 0.7]` | Command saturation: linear (m/s) and angular (rad/s). |
| `max_acce` | `[a, α]` | `[0.5, 0.7]` | Acceleration saturation. |
| `length` / `width` | float | 0.7 / 0.35 | Robot footprint, used by NeuPAN for clearance checks. |

> Length and width reflect physical robot capability; treat them as platform constants, not tuning knobs.
> If you need to modify the length and width parameters, you also need to retrain the DUNE checkpoint.

#### Tracking and avoidance weights (`adjust:`)

These are the day-to-day tuning knobs. Each entry is a relative weight in the MPC cost function.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `q_s` | float | 1.0 | Lateral position (x, y) tracking. Higher = hugs path tightly. |
| `q_theta` | float | 1.0 | Heading (yaw) tracking. Higher = head locks onto path direction. |
| `p_u` | float | 1.0 | Penalty for `v` deviation from `ref_speed`. Raise if the robot won't reach target speed. |
| `p_w` | float | 0.8 | Penalty on absolute `omega`. Raise to dampen wobble. |
| `p_w_cross` | float | 0.0 | Cross-cycle `omega` anchoring. Anti-limit-cycle weapon, start at 1.0 if `p_w` alone isn't enough. |
| `eta` | float | 12.0 | Reward for routing around obstacles. Higher = actively detours. |
| `ro_obs` | float | 400 | Repulsive strength of obstacle points. |
| `d_max` | float | 0.25 | Lateral obstacle inflation (m). Higher = wider buffer (struggles in narrow corridors). |
| `d_min` | float | 0.02 | NRMP inner boundary. Do not change. |
| `bk` | float | 0.4 | Proximal cost weight (smoothness). |

#### Initial path (`ipath:`)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `interval` | float | 0.1 | Resampling interval along the input path (m). |
| `arrive_threshold` | float | 0.2 | Distance to path end that triggers `/neupan/arrive` (m). |
| `close_threshold` | float | 0.1 | "At waypoint" threshold (m). |
| `ind_range` | int | 30 | Search-window size when locating the closest path point (# of samples). |
| `arrive_index_threshold` | int | 1 | Backup arrive criterion (index distance to end). |
| `curve_style` | string | `"line"` | Used by waypoints input: `"line"` / `"dubins"` / `"reeds"`. |

#### PAN optimizer (`pan:`)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `iter_num` | int | 2 | PAN inner iterations per cycle. |
| `dune_max_num` | int | 100 | Max obstacle points fed to DUNE. |
| `nrmp_max_num` | int | 10 | Max obstacle points fed to NRMP. |
| `dune_checkpoint` | path | `model_5000.pth` | DUNE neural network weights. |

### NeuPAN — `config.yaml`

#### Topics

| Parameter | Type | Default | Description |
|---|---|---|---|
| `topic.scan` | string | `/scan_360` | LaserScan input. Switch to `/scan` if `use_fake360:=false`. |
| `topic.cmd_vel` | string | `/cmd_vel` | Command output. |
| `topic.path` | string | `/initial_path` | Reference path input (`nav_msgs/Path`). |
| `topic.waypoints` | string | `/waypoints` | Waypoint input (`nav_msgs/Path`); NeuPAN connects them itself. |
| `topic.goal` | string | `/neupan/goal` | Single-goal input (`PoseStamped`); NeuPAN draws a straight line to it. |
| `topic.arrive` | string | `/neupan/arrive` | "Arrived" event published by NeuPAN. |
| `topic.odom` | string | `/odin1/odometry` | Odometry input for `odom_feedback`. |
| `refresh_initial_path` | bool | true | Reload the path on every new message; set false to load only once. |
| `include_initial_path_direction` | bool | false | Use `pose.orientation` from the path; otherwise infer yaw from neighbouring points. |

#### Smith predictor (`smith_predictor:`)

Compensates the cmd-to-effect delay: the MPC plans from the state the robot is *predicted* to be in after `tau = solve_time + fixed_lag`, instead of its current pose. Tune when commands feel phase-lagged (overshoot at corners, late reactions).

| Parameter | Type | Default | Description |
|---|---|---|---|
| `enable` | bool | true | Master switch. Disable to plan from the measured pose (legacy behaviour). |
| `fixed_lag_sec` | float | 0.02 | Fixed actuator lag (s) added to `tau`. Raise if the base is slow to execute commands. |
| `use_solve_time` | bool | true | Add the last MPC solve time to `tau`. Keep on unless solve time is very jittery. |
| `max_tau_sec` | float | 0.15 | Upper cap on `tau` (s). Prevents over-prediction after an occasional slow solve. |
| `velocity_source` | string | `"odom"` | Velocity used for the forward roll: `odom` (falls back to last cmd when odom is stale/missing) or `cmd`. |

#### Scan preprocessing

Filters applied to the incoming LaserScan before points are fed to the planner.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `scan_range` | `[min, max]` | `[0.2, 4.5]` | Keep only returns within this distance band (m). Raise `min` to reject self-body hits; lower `max` to shrink the obstacle horizon. |
| `scan_angle` | `[min, max]` | `[-3.14, 3.14]` | Keep only beams within this angular band (rad). Narrow it to ignore rear returns. |
| `scan_downsample` | int | 1 | Keep every N-th beam. Raise to cut CPU load on dense scans. |
| `flip_angle` | bool | false | Reverse the beam angle order; set true only if the scan is mirrored. |

#### Visualization markers

| Parameter | Type | Default | Description |
|---|---|---|---|
| `marker_size` | float | 0.05 | Cube edge length (m) of DUNE/NRMP point markers in RViz. Cosmetic only. |
| `marker_z` | float | 0.3 | Height (m) of the robot footprint marker. Cosmetic only. |

#### Command smoothing

| Parameter | Type | Default | Description |
|---|---|---|---|
| `cmd_w_lpf_alpha` | float | 0.0 | First-order low-pass on `omega` before rate limiting: `w = a*prev + (1-a)*new`. 0 disables; try 0.3-0.7 to dampen jerky rotation, at the cost of turn responsiveness. |

#### Pre-alignment (`prealign:`)

Optional rotate-in-place override: when the heading error to the path is large *and* a front obstacle is close, the robot stops and turns toward the path before handing control back to NeuPAN (with hysteresis on exit). Enable if the robot enters doorways/corridors at a poor angle.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `enable` | bool | false | Master switch (disabled by default). |
| `heading_threshold` | float | 0.5 | Heading error (rad, ~30°) above which pre-align may engage. |
| `obs_distance_threshold` | float | 1.0 | Front-cone obstacle must be closer than this (m) to engage. |
| `exit_heading_threshold` | float | 0.2 | Exit once error drops below this (rad, ~12°). Keep well below `heading_threshold` for hysteresis. |
| `exit_obs_distance_threshold` | float | 1.5 | Also exit once front clearance exceeds this (m). |
| `cone_half_width_rad` | float | 0.524 | Half-width (rad, ±30°) of the forward cone used for the obstacle check. |
| `angular_kp` | float | 1.5 | P-gain of the rotation: `w = clip(kp * heading_err, ±max_w)`. |
| `max_w` | float | 0.6 | Rotation speed cap (rad/s). Must not exceed `robot.max_speed[1]`. |
| `ref_avg_count` | int | 5 | First N path yaws averaged (vector mean) to get a stable target heading. |

#### Adaptive d_max (`adaptive_d_max:`)

Optional runtime override of `d_max`: periodically estimates the local corridor half-width from scan points and sets `d_max = clip(half_width - robot_half_width - safety_margin, min, max)`. Enable when the environment mixes open areas (want a wide buffer) and narrow doorways (need a small one).

| Parameter | Type | Default | Description |
|---|---|---|---|
| `enable` | bool | false | Master switch (disabled by default). When on, it overwrites the static `d_max` from `planner.yaml`. |
| `min` | float | 0.05 | Lower clamp on the resulting `d_max` (m). |
| `max` | float | 0.25 | Upper clamp on the resulting `d_max` (m). |
| `safety_margin` | float | 0.05 | Subtracted from the free half-width (m); also extends the sampling window slightly behind the robot. |
| `forward_window_m` | float | 2.0 | Only points up to this far ahead (robot frame) are used for the estimate. |
| `lateral_window_m` | float | 2.0 | Only points within ± this laterally are used; also the assumed clearance when one side is empty. |
| `update_period_sec` | float | 0.5 | Re-estimate at most every this many seconds. |

#### Stuck escape (`stuck_escape:`)

Recovery state machine: frames where the robot is not moving (or commanding ~zero) *with* a close front obstacle are counted; at a low count an early A\* replan is requested, and at `trigger_frames` the robot backs up while rotating toward the clearer side until timeout or clearance, then requests a replan. The main loop runs at 50 Hz, so counts are in 20 ms frames (non-stuck frames subtract 2).

| Parameter | Type | Default | Description |
|---|---|---|---|
| `enable` | bool | true | Master switch. |
| `displacement_window_sec` | float | 1.0 | Sliding-window length (s) for displacement measurement. |
| `displacement_threshold_m` | float | 0.05 | Moving less than this (m) over the window counts as "not moving". Raise if a slow crawl should also count as stuck. |
| `v_eps` | float | 0.15 | Alternative trigger: last commanded \|v\| below this (m/s)... |
| `w_eps` | float | 0.30 | ...and \|w\| below this (rad/s) also counts as "not moving" (planner deadlock). |
| `trigger_frames` | int | 10 | Stuck-count needed to start the escape maneuver (~0.2 s at 50 Hz). Raise if escapes fire too eagerly. |
| `front_obs_threshold` | float | 1.0 | A frame only counts as stuck when the front-cone obstacle is closer than this (m). |
| `cone_half_width_rad` | float | 0.7 | Half-width (rad, ±40°) of the forward cone used for all front/side clearance checks. |
| `back_speed` | float | 0.2 | Reverse speed (m/s) during the escape maneuver. |
| `rotate_speed` | float | 0.27 | Rotation speed (rad/s) during escape; sign points toward the clearer side. |
| `duration_sec` | float | 2.0 | Hard time cap (s) on one escape maneuver. |
| `exit_obs_distance` | float | 1.0 | Escape may end early once the front cone clears beyond this (m) *and* `min_rotation_rad` is reached. |
| `side_search_radius_m` | float | 3.5 | Radius (m) of the left/right clearance comparison that picks the rotation direction. |
| `limit_cycle_consecutive_n` | int | 2 | If this many recent escapes chose the same direction... |
| `limit_cycle_window_sec` | float | 20.0 | ...within this window (s), the next escape flips direction to break A↔B loops. |
| `min_rotation_rad` | float | 0.35 | Minimum yaw change (rad, ~20°) before the "cleared" early exit is allowed (timeout still exits). Raise if the robot exits still facing the obstacle. |
| `early_replan_enable` | bool | true | Request an A\* replan via `/neupan/arrive` at a low stuck count, **before** the full escape. |
| `early_replan_trigger_frames` | int | 5 | Stuck-count that triggers the early replan. Keep below `trigger_frames`. |
| `early_replan_cooldown_sec` | float | 2.0 | Minimum interval (s) between two early replans. |

#### Post-escape grace (`stuck_escape.post_escape_grace_*`)

After every escape, `q_theta` and `q_s` are temporarily scaled down so the MPC can leave along the replanned path instead of snapping straight back onto the old one (which re-creates the stuck situation). Weights restore once the robot has moved away with a clear front, or after a hard timeout.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `post_escape_grace_q_theta_scale` | float | 0.25 | Multiplier applied to `q_theta` during grace. Lower = freer heading. |
| `post_escape_grace_q_s_scale` | float | 0.5 | Multiplier applied to `q_s` during grace. Lower = freer lateral deviation. |
| `post_escape_grace_min_distance_m` | float | 0.5 | Robot must move this far (m) from grace start before weights can restore. |
| `post_escape_grace_clear_threshold_m` | float | 1.0 | Front-cone distance (m) above which the front counts as "clear". |
| `post_escape_grace_clear_dwell_sec` | float | 0.5 | The front must stay clear continuously for this long (s) before restoring. |
| `post_escape_grace_max_sec` | float | 5.0 | Hard cap (s); weights restore regardless. Lower if the robot wanders off-path too long after escapes. |


### Global planner — `whole.launch`

#### A\* search (`map_planner` node)

| Argument | Type | Default | Description |
|---|---|---|---|
| `inflation_radius` | float | 0.25 | Static-map inflation radius (m). Paths cannot enter cells closer than this to a wall. |
| `obstacle_cost_weight` | float | 3.0 | "Hug-the-centre" multiplier. 0 = pure A\*. |
| `obstacle_cost_safe_distance` | float | 0.5 | Distance beyond which there is no extra cost (m). |
| `line_of_sight_safe_distance` | float | 0.25 | String-pull may shortcut corners that stay this far from walls (m). |
| `smoothing_iterations` | int | 20 | Laplacian smoothing passes. 0 disables. |
| `smoothing_max_deviation_m` | float | 0.20 | Cap on total drift from the original A\* path during smoothing (m). |
| `start_heading_penalty_weight` | float | 2.0 | Multiplier biasing the first segment toward the robot's current yaw. 0 disables. |
| `start_heading_penalty_radius_m` | float | 1.0 | Distance from start where the heading penalty applies (m); fades linearly to 1× outside. |

#### Dynamic obstacle memory (`fake360` node)

| Argument | Type | Default | Description |
|---|---|---|---|
| `use_fake360` | bool | true | Enable 360° accumulated obstacle memory. False reverts to live `/scan` only. |
| `fake360_cell_decay_sec` | float | 3.0 | Out-of-FOV cells decay back to unknown after this many seconds. Lower for more dynamic environments. |

#### Goal state machine

| Argument | Type | Default | Description |
|---|---|---|---|
| `goal_tolerance` | float | 0.6 | Replan trigger distance (m). |
| `enable_arrive_replan` | bool | true | If true, every NeuPAN arrive triggers a fresh A\* — required for continuous dynamic-obstacle bypass. |

#### Pointcloud to LaserScan (pc_to_scan)

Slices 3D pointcloud into a 2D LaserScan for fake360 and NeuPAN. Adjust these parameters when the Odin sensor is mounted at a different height.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `target_frame` | string | `odin1_base_link` | Reference frame for the laser scan; do not change |
| `transform_tolerance` | float | 0.1 | TF lookup tolerance (s) |
| `min_height` | float | 0.0 | Minimum slice height (m), relative to `target_frame` origin |
| `max_height` | float | 0.3 | Maximum slice height (m), relative to `target_frame` origin |
| `angle_min` | float | -0.7853 | Scan start angle (rad), -45° |
| `angle_max` | float | 0.7853 | Scan end angle (rad), +45° |
| `angle_increment` | float | 0.017 | Angular resolution (rad), ~1° |
| `scan_time` | float | 0.1 | Scan period (s) |
| `range_min` | float | 0.2 | Minimum forward range (m), filters self-body noise |
| `range_max` | float | 5.0 | Maximum forward range (m) |


## Tuning Procedure

Diagnose by symptom, fix the upstream cause first.

### The robot wobbles in straight corridors

1. Inspect `cmd_w` over a 10-s straight segment.
2. **Fast wobble (0.3-0.5 Hz)**: raise `p_w` (e.g. 0.5 → 0.8). If still oscillating, enable `p_w_cross: 1.0`.
3. **Slow drift (0.1-0.2 Hz)**: nudge `q_theta` toward 1.0. Going above 1.2 typically reintroduces fast wobble.
4. **Path itself is wavy near walls**: see "The robot hugs walls" below.

### The robot hugs walls

1. Set `obstacle_cost_weight` to ~3.0 (forces A\* to prefer central cells).
2. Increase `obstacle_cost_safe_distance` to ~0.5 m.
3. Set `line_of_sight_safe_distance` to ~0.25 m so straight corridors still simplify.

### The robot fails to bypass a person or moving obstacle

1. Confirm `enable_arrive_replan: true` (replan-on-arrive is required).
2. NeuPAN side: raise `eta` (12 → 18) and `d_max` (0.25 → 0.4).
3. Global side: raise `obstacle_cost_weight` (3 → 5) and `obstacle_cost_safe_distance` (0.5 → 0.8).
4. fake360 layer: lower `fake360_cell_decay_sec` (3.0 → 2.0) so the memory reacts faster to people leaving.
5. If the robot keeps replanning a path that aims **away** from its current heading, `start_heading_penalty_weight` is what biases the early portion of A\*; raise it (2.0 → 3.0).

### The robot oscillates in narrow doorways

1. Lower `d_max` (0.25 → 0.20).
2. Lower `inflation_radius` (0.25 → 0.20). Watch that paths still keep clearance from real walls.
3. If the robot re-enters the doorway from a poor angle, raise `prealign` rotation thresholds in `config.yaml` (see code comments — disabled by default).

### The robot won't stop near the goal

1. Raise `ipath.arrive_threshold` (0.2 → 0.3-0.4 m). The MPC has natural overshoot near `goal_tolerance`.
2. Raise `goal_tolerance` (0.6 → 0.8 m) if you accept a coarser stop position.
3. Confirm the published goal is in the `map` frame and TF chain `map → odom → base_link` is healthy.

### The robot is too slow

1. Raise `ref_speed` (0.4 → 0.6) up to platform `max_speed[0]`.
2. Raise `p_u` (0.5 → 1.0) so `v` tracks `ref_speed` more tightly.

## Switching the global guidance source

NeuPAN accepts three guidance inputs simultaneously. Priority is `path > waypoints > goal`.

| Input | Type | Topic key | Use when |
|---|---|---|---|
| Full path | `nav_msgs/Path` | `topic.path` | An upstream global planner produces the full reference. |
| Waypoints | `nav_msgs/Path` | `topic.waypoints` | You want NeuPAN to interpolate (line / Dubins / Reeds-Shepp) between key points. |
| Goal | `PoseStamped` | `topic.goal` | No global planner; NeuPAN goes straight from current pose. **Not recommended** when static obstacles are present. |

### Point NeuPAN at a different path topic

```yaml
topic:
  path: "/your_planner/path"
```

Requirements:

- Type must be `nav_msgs/Path`.
- Poses are assumed to be in the `map` frame.
- `pose.position.x/y` are required. `pose.orientation` is used only when `include_initial_path_direction: True`; otherwise yaw is inferred from neighbouring points.

When in doubt, change one parameter at a time and record a short bag with `cmd_vel`, `odometry`, and `initial_path` and any other topics you needed to verify the effect before stacking changes.
