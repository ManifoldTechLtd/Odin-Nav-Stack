# 调参指南

Odin 导航栈（全局规划、动态障碍物记忆、NeuPAN）调参的实用参考。使用本指南来查看参数的作用、调整范围，以及在机器人行为异常时首先应该调节哪个旋钮。

## 概述

本导航栈包含三个配置层面：

| 文件 | 所属模块 |
|---|---|
| `NeuPAN/neupan/ros/configs/planner.yaml` | NeuPAN MPC 权重、机器人运动学、优化器设置 |
| `NeuPAN/neupan/ros/configs/config.yaml` | NeuPAN ROS 话题、里程计反馈、预对齐、卡死脱困 |
| `ros_ws/src/map_planner/launch/whole.launch` | 全局 A\* 参数、fake360 参数、目标状态机 |

修改后的操作：

| 修改内容 | 需要执行的操作 |
|---|---|
| 任意 `*.yaml` | 重启 `neupan_ros.py` |
| `whole.launch` 中的 `<arg>` | 重启 `roslaunch map_planner whole.launch` |
| `map_planner/` 或 `fake360/` 下的 C++ 源码 | `catkin_make --pkg map_planner`（或 `--pkg fake360`），然后重启 |

## 参数

### NeuPAN — `planner.yaml`

#### MPC 框架

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `receding` | int | 8 | MPC 预测步长（步数）。每步为 `step_time` 秒。越大=视野越好，求解越慢。 |
| `step_time` | float | 0.1 | 预测步长时间（秒）。很少修改。 |
| `ref_speed` | float | 0.5 | 前进速度目标值（米/秒）。 |
| `collision_threshold` | float | 0.05 | 低于此距离的采样点被视为碰撞（触发逃逸）。 |

#### 机器人运动学（`robot:`）

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `kinematics` | string | `"diff"` | 差速驱动。 |
| `max_speed` | `[v, w]` | `[0.5, 0.7]` | 指令饱和值：线速度（米/秒）和角速度（弧度/秒）。 |
| `max_acce` | `[a, α]` | `[0.5, 0.7]` | 加速度饱和值。 |
| `length` / `width` | float | 0.7 / 0.35 | 机器人外形尺寸，NeuPAN 用于通行间隙检查。 |

> 长度和宽度反映机器人的物理能力；请将其视为平台常量，而非调参旋钮。
> 如果需要修改长度和宽度参数，还需要重新训练 DUNE 检查点。

#### 跟踪与避障权重（`adjust:`）

这些是日常调参的旋钮。每个条目都是 MPC 代价函数中的相对权重。

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `q_s` | float | 1.0 | 横向位置（x, y）跟踪。越大=越紧贴路径。 |
| `q_theta` | float | 1.0 | 朝向（yaw）跟踪。越大=航向越锁定路径方向。 |
| `p_u` | float | 1.0 | `v` 偏离 `ref_speed` 的惩罚。如果机器人达不到目标速度，请增大此值。 |
| `p_w` | float | 0.8 | 对绝对 `omega` 的惩罚。增大可抑制摆动。 |
| `p_w_cross` | float | 0.0 | 跨周期 `omega` 锚定。用于抗极限环，如果 `p_w` 单独不够，可从 1.0 开始尝试。 |
| `eta` | float | 12.0 | 绕行障碍物的奖励。越大=越主动绕行。 |
| `ro_obs` | float | 400 | 障碍物点的排斥强度。 |
| `d_max` | float | 0.25 | 横向障碍物膨胀距离（米）。越大=缓冲区越宽（在狭窄走廊中会吃力）。 |
| `d_min` | float | 0.02 | NRMP 内部边界。不要修改。 |
| `bk` | float | 0.4 | 近端代价权重（平滑性）。 |

#### 初始路径（`ipath:`）

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `interval` | float | 0.1 | 沿输入路径的重采样间隔（米）。 |
| `arrive_threshold` | float | 0.2 | 触发 `/neupan/arrive` 的到路径终点距离（米）。 |
| `close_threshold` | float | 0.1 | "到达路点"阈值（米）。 |
| `ind_range` | int | 30 | 定位最近路径点时的搜索窗口大小（采样点数）。 |
| `arrive_index_threshold` | int | 1 | 备用的到达判定条件（到终点的索引距离）。 |
| `curve_style` | string | `"line"` | 路点输入时使用：`"line"` / `"dubins"` / `"reeds"`。 |

#### PAN 优化器（`pan:`）

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `iter_num` | int | 2 | 每个周期内 PAN 内部迭代次数。 |
| `dune_max_num` | int | 100 | 输入 DUNE 的最大障碍物点数。 |
| `nrmp_max_num` | int | 10 | 输入 NRMP 的最大障碍物点数。 |
| `dune_checkpoint` | path | `model_5000.pth` | DUNE 神经网络权重。 |

### NeuPAN — `config.yaml`

#### 话题

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `topic.scan` | string | `/scan_360` | LaserScan 输入。如果 `use_fake360:=false`，则切换为 `/scan`。 |
| `topic.cmd_vel` | string | `/cmd_vel` | 指令输出。 |
| `topic.path` | string | `/initial_path` | 参考路径输入（`nav_msgs/Path`）。 |
| `topic.waypoints` | string | `/waypoints` | 路点输入（`nav_msgs/Path`）；NeuPAN 自行连接这些点。 |
| `topic.goal` | string | `/neupan/goal` | 单个目标输入（`PoseStamped`）；NeuPAN 向目标画一条直线。 |
| `topic.arrive` | string | `/neupan/arrive` | NeuPAN 发布的"已到达"事件。 |
| `topic.odom` | string | `/odin1/odometry` | 里程计输入，用于 `odom_feedback`。 |
| `refresh_initial_path` | bool | true | 每次收到新消息时重新加载路径；设为 false 则只加载一次。 |
| `include_initial_path_direction` | bool | false | 使用路径中的 `pose.orientation`；否则从相邻点推断偏航角。 |

#### Smith 预估器（`smith_predictor:`）

补偿指令到执行效果的延迟：MPC 从机器人*预测*在 `tau = 求解时间 + 固定延迟` 后的状态进行规划，而不是从当前位姿。当指令感觉有相位滞后时（弯道过冲、反应迟缓）进行调节。

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `enable` | bool | true | 总开关。禁用则从测量位姿进行规划（旧版行为）。 |
| `fixed_lag_sec` | float | 0.02 | 添加到 `tau` 的固定执行器延迟（秒）。如果底盘执行指令慢，请增大此值。 |
| `use_solve_time` | bool | true | 将上一次 MPC 求解时间加到 `tau` 中。除非求解时间波动很大，否则保持开启。 |
| `max_tau_sec` | float | 0.15 | `tau` 的上限（秒）。防止偶尔的慢求解导致过度预测。 |
| `velocity_source` | string | `"odom"` | 用于前向预测的速度来源：`odom`（里程计数据过期或缺失时回退到上次指令）或 `cmd`。 |

#### 激光扫描预处理

在点云输入到规划器之前应用的滤镜。

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `scan_range` | `[min, max]` | `[0.2, 4.5]` | 只保留该距离范围内的回波（米）。增大 `min` 可排除自身机身干扰；减小 `max` 可缩短障碍物视野。 |
| `scan_angle` | `[min, max]` | `[-3.14, 3.14]` | 只保留该角度范围内的波束（弧度）。缩小可忽略后方回波。 |
| `scan_downsample` | int | 1 | 每隔 N 个波束保留一个。增大可在密集扫描时降低 CPU 负载。 |
| `flip_angle` | bool | false | 反转波束角度顺序；仅当扫描镜像时设为 true。 |

#### 可视化标记

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `marker_size` | float | 0.05 | RViz 中 DUNE/NRMP 点标记的立方体边长（米）。仅影响显示效果。 |
| `marker_z` | float | 0.3 | 机器人足迹标记的高度（米）。仅影响显示效果。 |

#### 指令平滑

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `cmd_w_lpf_alpha` | float | 0.0 | 在限速前对 `omega` 进行一阶低通滤波：`w = a*prev + (1-a)*new`。0 表示禁用；尝试 0.3-0.7 可抑制急转抖动，代价是转向响应变慢。 |

#### 预对齐（`prealign:`）

可选的原地旋转覆盖：当路径朝向误差较大*且*前方障碍物较近时，机器人停下来朝路径方向旋转，然后将控制权交还给 NeuPAN（出口带滞回）。如果机器人以不良角度进入门口/走廊，可启用此功能。

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `enable` | bool | false | 总开关（默认禁用）。 |
| `heading_threshold` | float | 0.5 | 朝向误差（弧度，约30°）超过此值时，预对齐可能介入。 |
| `obs_distance_threshold` | float | 1.0 | 前方锥形区域的障碍物必须比此距离（米）更近才能介入。 |
| `exit_heading_threshold` | float | 0.2 | 误差低于此值时退出（弧度，约12°）。建议远低于 `heading_threshold` 以实现滞回。 |
| `exit_obs_distance_threshold` | float | 1.5 | 前方净空超过此值（米）时也退出。 |
| `cone_half_width_rad` | float | 0.524 | 用于障碍物检查的前方锥形区域的半宽度（弧度，±30°）。 |
| `angular_kp` | float | 1.5 | 旋转的 P 增益：`w = clip(kp * heading_err, ±max_w)`。 |
| `max_w` | float | 0.6 | 旋转速度上限（弧度/秒）。不得超过 `robot.max_speed[1]`。 |
| `ref_avg_count` | int | 5 | 对前 N 个路径偏航角求平均（向量平均）以获得稳定的目标朝向。 |

#### 自适应 d_max（`adaptive_d_max:`）

可选的运行时 `d_max` 覆盖：定期根据扫描点估计局部走廊半宽度，并设置 `d_max = clip(半宽度 - 机器人半宽 - 安全余量, min, max)`。当环境中混合有开阔区域（需要宽缓冲区）和狭窄门口（需要小缓冲区）时启用。

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `enable` | bool | false | 总开关（默认禁用）。开启后将覆盖 `planner.yaml` 中的静态 `d_max`。 |
| `min` | float | 0.05 | 生成的 `d_max` 下限（米）。 |
| `max` | float | 0.25 | 生成的 `d_max` 上限（米）。 |
| `safety_margin` | float | 0.05 | 从自由半宽度中减去的安全余量（米）；同时将采样窗口略微向后延伸至机器人后方。 |
| `forward_window_m` | float | 2.0 | 仅使用前方（机器人坐标系）此距离内的点进行估计。 |
| `lateral_window_m` | float | 2.0 | 仅使用横向 ± 此距离内的点；当一侧为空时，也用作假设的净空距离。 |
| `update_period_sec` | float | 0.5 | 每此秒数最多重新估计一次。 |

#### 卡死脱困（`stuck_escape:`）

恢复状态机：统计机器人不动（或指令接近零）*同时*前方有近距离障碍物的帧数；计数较低时触发早期 A\* 重规划，达到 `trigger_frames` 时机器人后退并朝较空旷侧旋转，直到超时或净空恢复，然后请求重规划。主循环运行在 50 Hz，因此计数以 20 毫秒为单位（非卡死帧会减 2）。

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `enable` | bool | true | 总开关。 |
| `displacement_window_sec` | float | 1.0 | 位移测量的滑动窗口时长（秒）。 |
| `displacement_threshold_m` | float | 0.05 | 在窗口内移动小于此值（米）视为"不动"。如果缓慢爬行也应算作卡死，请增大此值。 |
| `v_eps` | float | 0.15 | 备选触发条件：上次指令 \|v\| 低于此值（米/秒）... |
| `w_eps` | float | 0.30 | ...且 \|w\| 低于此值（弧度/秒），也视为"不动"（规划器死锁）。 |
| `trigger_frames` | int | 10 | 触发脱困动作所需的卡死计数（50 Hz 下约 0.2 秒）。如果脱困过于激进，请增大此值。 |
| `front_obs_threshold` | float | 1.0 | 仅当前方锥形区域障碍物比此距离（米）更近时，该帧才会计入卡死。 |
| `cone_half_width_rad` | float | 0.7 | 用于所有前方/侧方净空检查的前方锥形区域半宽度（弧度，±40°）。 |
| `back_speed` | float | 0.2 | 脱困动作期间的倒退速度（米/秒）。 |
| `rotate_speed` | float | 0.27 | 脱困期间的旋转速度（弧度/秒）；方向指向较空旷侧。 |
| `duration_sec` | float | 2.0 | 单次脱困动作的硬性时间上限（秒）。 |
| `exit_obs_distance` | float | 1.0 | 当前方锥形区域净空超过此值（米）*且*达到 `min_rotation_rad` 时，脱困可提前结束。 |
| `side_search_radius_m` | float | 3.5 | 左右净空对比的搜索半径（米），用于选择旋转方向。 |
| `limit_cycle_consecutive_n` | int | 2 | 如果近期多次脱困选择了同一方向... |
| `limit_cycle_window_sec` | float | 20.0 | ...在此时间窗口（秒）内，下次脱困将反转方向以打破 A↔B 循环。 |
| `min_rotation_rad` | float | 0.35 | "已清空"提前退出允许的最小偏航角变化（弧度，约20°）（超时仍会退出）。如果退出时机器人仍面朝障碍物，请增大此值。 |
| `early_replan_enable` | bool | true | 在卡死计数较低时，在**完全脱困之前**通过 `/neupan/arrive` 请求 A\* 重规划。 |
| `early_replan_trigger_frames` | int | 5 | 触发早期重规划的卡死计数。应低于 `trigger_frames`。 |
| `early_replan_cooldown_sec` | float | 2.0 | 两次早期重规划之间的最小间隔（秒）。 |

#### 脱困后宽限期（`stuck_escape.post_escape_grace_*`）

每次脱困后，`q_theta` 和 `q_s` 会被临时降低，以便 MPC 沿着重规划的路径离开，而不是强行回到旧路径（这会重新导致卡死）。当机器人已经离开且前方净空恢复，或硬性超时后，权重恢复。

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `post_escape_grace_q_theta_scale` | float | 0.25 | 宽限期内应用于 `q_theta` 的倍数。越小=朝向越自由。 |
| `post_escape_grace_q_s_scale` | float | 0.5 | 宽限期内应用于 `q_s` 的倍数。越小=横向偏离越自由。 |
| `post_escape_grace_min_distance_m` | float | 0.5 | 机器人从宽限期开始必须移动此距离（米）后权重才能恢复。 |
| `post_escape_grace_clear_threshold_m` | float | 1.0 | 前方锥形区域距离（米）超过此值视为"清空"。 |
| `post_escape_grace_clear_dwell_sec` | float | 0.5 | 前方必须保持清空状态此时间（秒）后才能恢复权重。 |
| `post_escape_grace_max_sec` | float | 5.0 | 硬性上限（秒）；超时后权重强制恢复。如果脱困后机器人长时间偏离路径，请减小此值。 |

### 全局规划器 — `whole.launch`

#### A\* 搜索（`map_planner` 节点）

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `inflation_radius` | float | 0.25 | 静态地图膨胀半径（米）。路径不能进入距离墙壁小于此值的栅格。 |
| `obstacle_cost_weight` | float | 3.0 | "贴中心"乘系数。0 = 纯 A\*。 |
| `obstacle_cost_safe_distance` | float | 0.5 | 超出此距离不再有额外代价（米）。 |
| `line_of_sight_safe_distance` | float | 0.25 | 线简化时可以切角，但保持此距离远离墙壁（米）。 |
| `smoothing_iterations` | int | 20 | Laplacian 平滑迭代次数。0 禁用。 |
| `smoothing_max_deviation_m` | float | 0.20 | 平滑过程中与原始 A\* 路径的总偏移上限（米）。 |
| `start_heading_penalty_weight` | float | 2.0 | 将路径第一段偏向机器人当前偏航角的乘系数。0 禁用。 |
| `start_heading_penalty_radius_m` | float | 1.0 | 起点附近应用朝向惩罚的距离（米）；在此之外线性衰减到 1 倍。 |

#### 动态障碍物记忆（`fake360` 节点）

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `use_fake360` | bool | true | 启用 360° 累积障碍物记忆。false 则回退到仅使用实时 `/scan`。 |
| `fake360_cell_decay_sec` | float | 3.0 | 视场外的栅格在此秒数后衰减回未知。对于更动态的环境，请减小此值。 |

#### 目标状态机

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `goal_tolerance` | float | 0.6 | 重规划触发距离（米）。 |
| `enable_arrive_replan` | bool | true | 如果为 true，每次 NeuPAN 到达都会触发新的 A\* — 连续动态障碍物绕行必需。 |

#### 点云转激光扫描（pc_to_scan）

将 3D 点云切片为 2D 激光扫描数据，供 fake360 和 NeuPAN 使用。当 Odin 传感器安装在不同高度时调整这些参数。

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `target_frame` | string | `odin1_base_link` | 激光扫描的参考坐标系；不要修改 |
| `transform_tolerance` | float | 0.1 | TF 查找容差（秒） |
| `min_height` | float | 0.0 | 最小切片高度（米），相对于 `target_frame` 原点 |
| `max_height` | float | 0.3 | 最大切片高度（米），相对于 `target_frame` 原点 |
| `angle_min` | float | -0.7853 | 扫描起始角度（弧度），-45° |
| `angle_max` | float | 0.7853 | 扫描结束角度（弧度），+45° |
| `angle_increment` | float | 0.017 | 角度分辨率（弧度），约1° |
| `scan_time` | float | 0.1 | 扫描周期（秒） |
| `range_min` | float | 0.2 | 最小前向距离（米），过滤机身噪声 |
| `range_max` | float | 5.0 | 最大前向距离（米） |

## 调参流程

根据症状进行诊断，先修复上游原因。

### 机器人在直线走廊中摆动

1. 观察 10 秒直线段上的 `cmd_w`。
2. **快速摆动（0.3-0.5 Hz）**：增大 `p_w`（例如 0.5 → 0.8）。如果仍然振荡，启用 `p_w_cross: 1.0`。
3. **缓慢漂移（0.1-0.2 Hz）**：将 `q_theta` 向 1.0 微调。超过 1.2 通常会重新引入快速摆动。
4. **路径本身在墙壁附近呈波浪状**：见下面的"机器人贴墙走"。

### 机器人贴墙走

1. 将 `obstacle_cost_weight` 设为 ~3.0（强制 A\* 偏好中心栅格）。
2. 增大 `obstacle_cost_safe_distance` 至 ~0.5 米。
3. 将 `line_of_sight_safe_distance` 设为 ~0.25 米，使直线走廊仍能被简化。

### 机器人无法绕过行人或移动障碍物

1. 确认 `enable_arrive_replan: true`（到达重规划必需）。
2. NeuPAN 侧：增大 `eta`（12 → 18）和 `d_max`（0.25 → 0.4）。
3. 全局侧：增大 `obstacle_cost_weight`（3 → 5）和 `obstacle_cost_safe_distance`（0.5 → 0.8）。
4. fake360 层：降低 `fake360_cell_decay_sec`（3.0 → 2.0），使记忆对人员离开反应更快。
5. 如果机器人持续重规划出一条**远离**当前朝向的路径，`start_heading_penalty_weight` 用于偏置 A\* 的前段；增大此值（2.0 → 3.0）。

### 机器人在狭窄门口处振荡

1. 降低 `d_max`（0.25 → 0.20）。
2. 降低 `inflation_radius`（0.25 → 0.20）。注意路径仍要与真实墙壁保持距离。
3. 如果机器人以不良角度重新进入门口，在 `config.yaml` 中提高 `prealign` 的旋转阈值（参见代码注释 — 默认禁用）。

### 机器人无法在目标附近停止

1. 增大 `ipath.arrive_threshold`（0.2 → 0.3-0.4 米）。MPC 在 `goal_tolerance` 附近有自然的过冲。
2. 如果接受较粗的停止精度，增大 `goal_tolerance`（0.6 → 0.8 米）。
3. 确认发布的目标在 `map` 坐标系下，且 TF 链 `map → odom → base_link` 正常。

### 机器人速度太慢

1. 增大 `ref_speed`（0.4 → 0.6），最高到平台 `max_speed[0]`。
2. 增大 `p_u`（0.5 → 1.0），使 `v` 更紧贴 `ref_speed`。

## 切换全局引导来源

NeuPAN 同时接受三种引导输入。优先级为 `path > waypoints > goal`。

| 输入 | 类型 | 话题键 | 使用场景 |
|---|---|---|---|
| 完整路径 | `nav_msgs/Path` | `topic.path` | 上游全局规划器提供完整参考路径。 |
| 路点 | `nav_msgs/Path` | `topic.waypoints` | 希望 NeuPAN 在关键点之间进行插值（直线 / Dubins / Reeds-Shepp）。 |
| 目标 | `PoseStamped` | `topic.goal` | 没有全局规划器；NeuPAN 从当前位置直线前往。**存在静态障碍物时不推荐**。 |

### 将 NeuPAN 指向不同的路径话题

```yaml
topic:
  path: "/your_planner/path"
```

要求：

- 类型须为 `nav_msgs/Path`。
- 位姿假定在 `map` 坐标系下。
- `pose.position.x/y` 为必填。`pose.orientation` 仅在 `include_initial_path_direction: True` 时使用；否则偏航角从相邻点推断。

有疑问时，一次只修改一个参数，并在叠加修改前录制一个包含 `cmd_vel`、`odometry` 和 `initial_path` 及其他需要的主题的短 bag 包，以验证效果。