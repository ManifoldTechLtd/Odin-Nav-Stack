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

#### Stuck-escape (`stuck_escape:`)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `enable` | bool | true | Master switch. |
| `disp_threshold_m` | float | 0.05 | Frame-to-frame displacement below which the robot is considered "not moving". |
| `front_clear_threshold_m` | float | 0.6 | Forward clearance below which "stuck" requires obstacle proximity. |
| `stuck_frames` | int | 15 | Consecutive frames matching the above before full backup-rotate kicks in. |
| `early_replan_enable` | bool | true | Trigger an early A\* replan via `/neupan/arrive` at low stuck count, **before** the full escape. |
| `early_replan_trigger_frames` | int | 5 | Stuck-frame count that triggers the early replan. |
| `early_replan_cooldown_sec` | float | 2.0 | Minimum seconds between two early replans. |

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
