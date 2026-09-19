# 跟踪模块 & 火控模块 —— 技术报告（初版）

---

## 第一部分：跟踪模块（predict/）

### 1. 模块概览

跟踪模块在 Stage2 线程中运行，位于 CornerRefine（角点精炼）之后、Planner（火控规划）之前。它接收检测模块输出的装甲板角点，通过 PnP 解算转换为 3D 空间位置，再用卡尔曼滤波器对目标运动状态进行跟踪和预测，最终输出目标的当前位置、速度和预测轨迹给火控模块。

```
CornerRefine（检测结果）→ MultiPolicyPredictor（跟踪+预测）→ Planner（火控）
                               │
                    ┌──────────┴──────────┐
                    │  3 个 Tracker 实例   │
                    │  每个独立跟踪1个目标  │
                    │                     │
                    │  ├── 装甲板模型(KF)  │
                    │  └── 整车模型(IESEKF)│
                    └─────────────────────┘
```

### 2. 输入输出

**入（数据流入 MultiPolicyPredictor）**

| 来源 | 载体 | 内容 | 频率 |
|---|---|---|---|
| CornerRefine（角点精炼模块） | ThreadDataPack（同一线程，直接读取） | 精炼后的装甲板四角点列表（世界坐标系下的 xyza） | 每帧 |
| Sensor（传感器模块） | ThreadDataPack（同一线程，直接读取） | 云台当前姿态、裁判系统状态（敌方颜色、弹速） | 每帧 |
| CoordTransformer（坐标变换单例） | 直接调用 | PnP 位姿解算服务 | 每帧每装甲板一次 |

**出（数据流出 MultiPolicyPredictor）**

| 去向 | 载体 | 内容 | 频率 |
|---|---|---|---|
| Planner（火控模块） | ThreadDataPack（同一线程，直接传递） | 选定的最优目标 Target（含跟踪状态、估计位置/速度、使用的模型类型） | 每帧 |

### 3. 各文件作用

#### 3.1 `MultiPolicyPredictor_submodule.hpp/cpp` —— 多策略预测器（SubModule）

- **类名**：`predict::MultiPolicyPredictorSubModule`，继承自 `pipeline::SubModule`
- **职责**：跟踪模块的"调度中心"——管理 3 个 Tracker，做装甲板匹配分发，选最优目标

**核心方法 `predict()` 的执行流程**：

```
① 从 ThreadDataPack 提取数据
   ├── 检测框列表 bboxes（来自 CornerRefine）
   ├── 云台姿态 attitude_yaw/pitch（来自 Sensor）
   └── 裁判系统状态 robot_status（来自 Sensor）

② 自瞄模式判断（连续 N 帧处于 AUTO_AIM 模式才确认进入）

③ 装甲板筛选（每个检测框逐一处理）
   ├── 颜色匹配（只跟踪敌方颜色的装甲板）
   ├── PnP 解算（调用 CoordTransformer，得到 3D 位置）
   ├── 距离/角度过滤（>4.5m 或偏航角过大则丢弃）
   └── 分类：已在跟踪的 → armors_in_tracking；新的 → new_armors

④ 新装甲板匹配到空闲 Tracker
   ├── 按距离+图像中心位置的加权得分排序
   └── 找到空闲 Tracker → 调用 reset_target() 初始化跟踪

⑤ 已有装甲板更新对应 Tracker
   └── 调用 track() 执行 KF/IESEKF 预测+更新

⑥ 选择最优目标
   从 3 个 Tracker 中选一个跟踪状态最好、距离最近的目标
   → 写入 data->target, data->has_fixed_target
```

#### 3.2 `Tracker.hpp/cpp` —— 目标跟踪器

- **类名**：`predict::Tracker`
- **职责**：单个目标的完整跟踪生命周期管理——状态机、双模型滤波、装甲板匹配

**状态机**：

```
IDLE（空闲）
  │ 收到该 tag_id 的检测
  ▼
DETECTING（检测中，需连续 4 帧命中才确认）
  │ 连续 4 帧成功
  ▼
TRACKING（跟踪中，正常状态）
  │ 某一帧丢失              │ 连续丢失 30 帧
  ▼                         ▼
TEMP_LOST（暂时丢失）→（超时）IDLE
  │ 30 帧内重新检测到
  ▼
TRACKING
```

**双模型滤波架构**：

```
Tracker 内部有两套数学模型、两类滤波器，根据目标旋转速度自动切换

┌─────────────────────────────────────────────────────────────┐
│  装甲板模型（ARMOR_MODEL）                                   │
│  ├── 思想：把装甲板当成独立运动的小物体                       │
│  ├── 状态：每个轴独立的位置+速度（共 4×2=8 维独立估计）       │
│  │     x 轴: [x, vx]     y 轴: [y, vy]                     │
│  │     z 轴: [z, vz]   yaw 角: [yaw, vyaw]                 │
│  ├── 滤波器：4 个独立的线性 KF（Kalman<1,2>）                │
│  │     每个 KF 用连续白噪声加速度运动模型                     │
│  │     predict: 匀速外推  update: 新观测修正                 │
│  ├── 优势：收敛快，延迟低                                    │
│  └── 劣势：无法预测旋转带来的装甲板轨迹变化                    │
│                                                              │
│  整车模型（VEHICLE_MODEL）                                    │
│  ├── 思想：把敌方机器人建模为绕中心旋转的刚体                   │
│  ├── 状态：11 维状态向量                                     │
│  │     [yc, vy, xc, vx, zc, vz, yaw, ω, r, Δr, Δz]        │
│  │      ↑车辆中心    ↑车辆中心   ↑旋转角  ↑转速 ↑半径 ↑另一对板│
│  ├── 滤波器：1 个 IESEKF<4,11>（迭代误差状态扩展卡尔曼）       │
│  │     观测方程: ya = yc - r·cos(yaw)（从车辆中心推算装甲板位置）│
│  │     IESEKF 在更新步迭代 5 次，反复重新线性化                 │
│  ├── 优势：能预测旋转轨迹，对高速旋转目标跟踪准                 │
│  └── 劣势：需要多帧数据才能收敛，初始化阶段不可靠               │
└─────────────────────────────────────────────────────────────┘
```

**模型切换逻辑**（`tracker_model_select()`）：

```
当前 ARMOR_MODEL:
    if |转速| >  0.7 rad/s → 切到 BOTH（两种模型都更新）
当前 BOTH:
    if |转速| >  2.7 rad/s → 切到 VEHICLE_MODEL（仅整车模型）
    if |转速| <  0.7 rad/s → 切回 ARMOR_MODEL（仅装甲板模型）
当前 VEHICLE_MODEL:
    if |转速| <  2.7 rad/s → 切回 BOTH
```

**装甲板 ID 匹配**（`match_armor_id()`）：

整车模型跟踪的是车辆中心，但观测到的是某一个装甲板。需要判断"当前观测到的是四个装甲板中的哪一个"。本系统使用**熵权法**：
- 用整车模型算出 4 个装甲板的预测位置
- 对每个候选装甲板，计算两个负向指标（空间距离差、角度差）
- 用熵权法自动确定两个指标的权重（数据离散度越大权重越小）
- 选加权得分最小的那个——即"最像当前观测"的装甲板编号

#### 3.3 `IESEKF.hpp` —— 迭代误差状态扩展卡尔曼滤波器

- **类名**：`mathutils::IESEKF<V_Z, V_X>`（模板类）
- **和普通 EKF 的区别**：

| | 普通 EKF | IESEKF |
|---|---|---|
| 更新步 | 在预测点处线性化一次，算完就结束 | 线性化→更新→在更新点重新线性化→再更新，反复 5 次 |
| 适用场景 | 弱非线性系统 | 强非线性系统（高速旋转、装甲板跳变） |
| 计算量 | 低 | 约 5 倍 |
| 本项目用在哪 | 不直接用普通 EKF | 用于整车模型跟踪 |

**"误差状态"是什么意思？** 不直接估计状态 x，而是估计"当前状态离名义状态的误差 δx"。好处是误差通常很小，线性近似更准；同时角度等周期性量可以在误差空间里正确处理。

**"流形运算"（x_add / x_minus）**：普通向量加减在角度上会出问题（比如 179° - (-179°) = 358°，实际只差 2°）。IESEKF 让用户自定义 `x_minus`，在里面做角度归一化——这对接近 ±180° 的角度跳变至关重要。

#### 3.4 `Kalman.hpp` —— 线性卡尔曼滤波器

- **类名**：`mathutils::Kalman<V_Z, V_X>`（模板类）
- **用途**：用于装甲板模型的 4 个独立 KF（x/y/z/yaw 各一个）
- **模型**：连续白噪声加速度（CWNA）运动模型
  - 状态：`[位置, 速度]`
  - 过程噪声 Q 的离散化形式：`dt⁴/4·q, dt³/2·q, dt³/2·q, dt²·q`

#### 3.5 `CoordTransformer.hpp/cpp` —— 坐标变换器（单例）

- **职责**：PnP 位姿解算 + 三大坐标系变换
- **关键方法**：

| 方法 | 功能 |
|---|---|
| `pnp_get_measurement()` | 输入 4 个图像角点 → PnP 解算 → 输出装甲板在相机坐标系下的 3D 位置和 yaw 角 |
| `pw_to_pi()` | 世界坐标系 → IMU 坐标系 |
| `camera2world()` | 相机坐标系 → 世界坐标系（PnP 结果 + 外参） |

**PnP 原理简述**：已知装甲板的物理尺寸（小装甲板 13.5cm×5.6cm，大装甲板 22.8cm×5.6cm）和相机内参，通过 4 对 3D-2D 点对应关系，求解装甲板相对于相机的旋转和平移。

#### 3.6 `math_tools.hpp` —— 数学工具函数

- `distance_2D()` / `distance_3D()`：2D/3D 距离计算
- `limit_rad()`：角度归一化到 [-π, π]
- `pw_to_yaw()`：从世界坐标计算目标相对于己方的偏航角
- `cal_fly_time()`：计算弹丸飞行时间（考虑或不考虑空气阻力）

---

## 第二部分：火控模块（planner/）

### 1. 模块概览

火控模块是自瞄系统的**最后一环**，在 Stage2 线程中位于 MultiPolicyPredictor 之后。它接收跟踪器输出的目标状态，负责：弹道补偿计算、云台轨迹优化（MPC）、射击决策、指令打包发送。

```
MultiPolicyPredictor → PlannerSubModule → PushBridge → TimedSerial → 串口 → MCU
                            │
                    ┌───────┴────────┐
                    │   Planner 核心  │
                    │  ├── 4 种射击策略│
                    │  ├── MPC 轨迹优化│
                    │  └── 射击决策   │
                    └────────────────┘
```

### 2. 输入输出

**入（数据流入 PlannerSubModule）**

| 来源 | 载体 | 内容 |
|---|---|---|
| MultiPolicyPredictor（跟踪模块） | ThreadDataPack（同一线程，直接读取） | 选定的目标 Target（含跟踪状态、估计位置/速度/转速） |
| Sensor（传感器模块） | ThreadDataPack（同一线程，直接读取） | 云台姿态、裁判系统状态（弹速、敌方颜色） |
| plannerParam/*.yml 文件 | 构造函数时读取 | 延迟参数、策略开关、补偿参数 |

**出（数据流出 PlannerSubModule）**

| 去向 | 载体 | 内容 | 频率 |
|---|---|---|---|
| TimedSerial（串口模块） | PlannerToSerialBridge（PushBridge） | 10 个前瞻 RobotCommand + 当前姿态 + 规划周期 2ms | 每帧推送一次 |
| TimedSerial → MCU（经插值后） | 串口线 | 1 个 RobotCommand（yaw/pitch 角度+角速度+距离+射击标志+目标ID） | 每 5ms |

### 3. 各文件作用

#### 3.1 `planner_submodule.hpp/cpp` —— 火控子模块（SubModule）

- **类名**：`plan::PlannerSubModule`，继承自 `pipeline::SubModule`
- **职责**：火控的"胶水层"——从 ThreadDataPack 取数据喂给 Planner，把 Planner 的结果翻译为 RobotCommand，再通过 PushBridge 推给 TimedSerial

**`process()` 执行流程**：

```
① 从 ThreadDataPack 提取：云台姿态、弹速、目标、R_world2imu

② 根据跟踪状态重置 Planner：
   IDLE → planner_reset()（清空所有状态）
   DETECTING → aim_target_init()（初始化无模型策略）

③ 如果不在 IDLE → 调用 planner.make_plan() 生成完整射击计划

④ 调用 update_information_to_send()
   把 Plan 里的 target_yaw/pitch（IMU 坐标系下的绝对角度）
   转换为 RobotCommand 里的相对增量：
     send.yaw_angle  = (target_yaw  - attitude_yaw)  / π × 180°
     send.pitch_angle = (target_pitch - attitude_pitch) / π × 180°

⑤ generate_command_array()
   把 1 个 RobotCommand 扩展为 10 个前瞻指令：
   第 i 个指令的 yaw = 当前 yaw + i×yaw_speed×2ms
   这样 TimedSerial 在 5ms 发送间隙可以做插值

⑥ 封装 PlannerToSerialMessage，通过 PushBridge 推送给 TimedSerial
```

#### 3.2 `Planner.hpp/cpp` —— 弹道规划器核心

- **类名**：`predict::Planner`
- **职责**：火控算法的全部核心逻辑——策略选择、弹道补偿、MPC 轨迹优化、射击决策

##### 3.2.1 四种射击策略

| 策略 | 枚举值 | 瞄准对象 | 适用场景 | 云台轨迹来源 |
|---|---|---|---|---|
| **策略1**：无模型跟踪装甲板 | `ARMOR_WITH_NO_MODEL` | 当前检测到的装甲板位置 | DETECTING 阶段、刚发现目标 | 直接使用观测值，不预测 |
| **策略2**：装甲板模型跟踪装甲板 | `ARMOR_WITH_ARMOR_MODEL` | 装甲板位置 + 用 KF 速度外推 | 低速目标（< 0.7 rad/s） | KF 预测位置 + 线性外推 |
| **策略3**：整车模型跟踪装甲板 | `ARMOR_WITH_VEHICLE_MODEL` | IESEKF 预测的最近装甲板 | 中高速目标（0.7~7.33 rad/s） | MPC 优化轨迹（平滑+满足物理约束） |
| **策略4**：整车模型瞄准车辆中心 | `VEHICLE_CENTER_WITH_VEHICLE_MODEL` | 车辆旋转中心 | 高速旋转目标（> 6.28 rad/s，反陀螺） | 预测发射窗口，等装甲板转过来 |

**策略自动切换逻辑**（基于转速 + 滞后区间防抖）：

```
转速范围：
  0 ────── 0.7 ────── 1.57 ────── 2.1 ────── 6.28 ────── 7.33 ────── ∞
  │ 策略2   │   策略3   │  策略3↔4 │   策略3   │   策略4    │  策略3   │
  │ 装甲板  │  整车模型  │ 滞后区间  │  整车模型  │  整车中心   │ 整车模型 │
  │ 模型    │  跟踪装甲板│ (维持原状) │  跟踪装甲板│  瞄准中心   │ 跟踪装甲板│
```

关键：转速在两个阈值之间时维持当前策略不变（防止来回切换抖动）。

##### 3.2.2 弹道补偿计算（`cal_gimbal_target()`）

```
① 已知装甲板在世界坐标系下的 3D 位置 pw

② 计算弹丸飞行时间
   fly_time = cal_fly_time(pw, bullet_speed)
   不考虑空气阻力时：fly_time = distance / bullet_speed

③ 重力下坠补偿
   pw.z -= 0.5 × g × fly_time²
   如果考虑空气阻力，fly_time 的计算会更复杂

④ 世界坐标系 → IMU 坐标系（减去云台旋转）
   pi = R_world2imu × pw

⑤ 计算云台应转到的角度
   target_yaw  = atan(pi.x / pi.z) + attitude_yaw  + yaw_comp
   target_pitch = atan(pi.y / pi.z) + attitude_pitch + pitch_comp
```

##### 3.2.3 MPC 轨迹优化（仅策略3使用）

**问题**：装甲板在空间中的轨迹是一条曲线（因为敌方机器人在旋转），云台如果直接逐点跟踪，会抖动、超调、不平滑。

**解决方案**：用 MPC（模型预测控制）提前规划一条**物理上可行**的平滑轨迹。

**TinyMPC 设置**：

```
状态空间模型（对 yaw 和 pitch 分别建模）：
  ┌ x₁(k+1) ┐   ┌ 1  dt ┐ ┌ x₁(k) ┐   ┌ 0  ┐
  │          │ = │       │ │       │ + │    │ u(k)
  └ x₂(k+1) ┘   └ 0   1 ┘ └ x₂(k) ┘   └ dt ┘

  x₁ = 角度（相对值）,  x₂ = 角速度,  u = 角加速度

代价函数：
  min Σ [Q·(x₁ - x₁_ref)² + R·u²]
  Q = 9×10⁶（极度重视跟踪精度）
  R = 1（适当惩罚剧烈加速）

约束（云台物理限制）：
  yaw 轴：|u| ≤ 50 rad/s²
  pitch 轴：|u| ≤ 100 rad/s²

求解器：ADMM（交替方向乘子法），最大 10 次迭代
规划视野：HORIZON = 100 步，DT = 0.01s → 前瞻 1 秒
```

**轨迹生成**（`get_trajectory()`）：
- 用整车模型预测未来 HORIZON 步的装甲板位置（`predict_closest_armor`）
- 对每个预测位置计算弹道补偿后的 yaw/pitch 角度
- 用中心差分法（`[x(k+1) - x(k-1)] / 2dt`）算角速度
- 把角度和角速度拼成 4×HORIZON 的参考轨迹矩阵
- 喂给 TinyMPC 求解器

##### 3.2.4 射击决策

**策略3（整车模型+MPC）的决策**：
```
射击条件 = MPC 优化轨迹 和 参考轨迹 的偏差 < same_trace_threshold（0.0035 rad ≈ 0.2°）
即：MPC 能"跟上"目标轨迹，说明瞄准稳定 → 可以开枪

装甲板跳变时强制禁止射击（armor_jump=true → fire_enable=0）
```

**策略4（整车模型瞄准中心）的决策**：
```
预测装甲板即将转到面向己方的时刻（发射窗口）
if 预测的装甲板偏航角 和 己方瞄准方向 的差 < fire_threshold（0.05 rad ≈ 2.86°）
   → 预估命中，fire_enable=3（单发模式）
   发射间隔最小 0.2s（防止连发浪费弹药）
```

#### 3.3 `tinympc/` —— 微型 MPC 求解库

- **算法**：ADMM（交替方向乘子法）
- **特点**：专为嵌入式实时控制设计的轻量 MPC 求解器，无外部依赖
- **关键文件**：
  - `tiny_api.hpp/cpp`：C 接口封装，`tiny_setup` / `tiny_solve` / `tiny_set_x0`
  - `admm.hpp/cpp`：ADMM 核心算法实现
  - `codegen.hpp/cpp`：矩阵运算代码生成（加速）

---

## 模块交互总结

```
┌────────────────────────────────────────────────────────────────────┐
│                        Stage2 线程内部                              │
│                                                                     │
│  CornerRefine（角点精炼）                                           │
│       │ 精炼后的 bboxes                                             │
│       ▼                                                             │
│  MultiPolicyPredictor（跟踪）                                       │
│       │                                                             │
│       ├── 对每个 bbox 做 PnP 解算 → 3D 位置                          │
│       ├── 匹配到 3 个 Tracker 之一                                   │
│       │     ├── Tracker 内部做 KF/IESEKF 预测+更新                   │
│       │     └── 根据转速自动切换装甲板模型/整车模型                    │
│       └── 选最优目标 → 写入 data->target                             │
│       │                                                             │
│       ▼                                                             │
│  PlannerSubModule（火控）                                           │
│       │                                                             │
│       ├── 读 target → 根据转速选 4 种策略之一                         │
│       ├── 弹道补偿：重力下坠 + 飞行时间                               │
│       ├── 策略3时：MPC 优化云台轨迹（100 步前瞻）                      │
│       ├── 射击决策：瞄准精度 + 发射窗口判断                           │
│       ├── 生成 10 个前瞻 RobotCommand                                │
│       └── PushBridge 推送给 TimedSerial                             │
│                                                                     │
└────────────────────────────────────────────────────────────────────┘

                              PushBridge
                                  │
                                  ▼
┌────────────────────────────────────────────────────────────────────┐
│  TimedSerial（独立线程）                                            │
│       ├── 接收指令数组 → 按时间戳插值 → 每 5ms 发送一条              │
│       └── 串口 → MCU → 云台电机转动 → 子弹发射                      │
└────────────────────────────────────────────────────────────────────┘
```

---

*报告初版，待补充：性能测试数据、参数调优建议。*
