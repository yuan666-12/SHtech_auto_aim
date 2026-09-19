# 通信模块 & 目标检测模块 —— 技术报告（初版）

---

## 第一部分：通信模块（timedserial/）

### 1. 模块概览

通信模块是自瞄系统与**下位机 MCU**（云台+发射控制器）之间的桥梁。它在系统中以**独立线程（BasicTask）** 的形式运行，负责：

- **下行（PC → MCU）**：将 Planner 规划出的云台控制指令（yaw/pitch 角度、角速度、射击信号），以 5ms 周期定时发送给下位机
- **上行（MCU → PC）**：实时接收下位机上报的 IMU 姿态数据（yaw/pitch/roll）和裁判系统赛场数据（血量、敌我识别等），提供给 Sensor 模块使用

```
Planner（规划器）                       Sensor（传感器）
     │                                      │
     │ PushBridge 推送指令                  │ PullBridge 拉取姿态/状态
     ▼                                      ▼
┌─────────────────────────────────────────────────┐
│                 TimedSerial                      │
│                                                  │
│  ┌──────────┐    ┌──────────────┐               │
│  │ 指令队列  │    │ SerialInterface│  ← 依赖注入  │
│  │ (10个元素)│    │ (UartDriver /  │              │
│  │ 时间戳插值│    │  MockDriver)   │              │
│  └──────────┘    └──────┬───────┘               │
│                         │                        │
└─────────────────────────┼────────────────────────┘
                          │ 串口 921600bps
                          ▼
              ┌─────────────────────┐
              │   下位机 MCU         │
              │   (云台 + 发射控制)    │
              └─────────────────────┘
```

### 2. 输入输出

**入（数据流入 TimedSerial）**

| 来源 | 载体 | 内容 | 频率 |
|---|---|---|---|
| Planner | PushBridge（软件管道） | 10 个云台控制指令 + 参考姿态 | 每 2ms |
| MCU（下位机） | 串口线（硬件，921600bps） | IMU 姿态（yaw / pitch / roll）+ 弹速 + 自瞄模式 | ~200Hz |
| MCU（下位机） | 串口线（硬件，921600bps） | 裁判系统数据（全场血量 + 己方 ID） | ~10Hz |

**出（数据流出 TimedSerial）**

| 去向 | 载体 | 内容 | 频率 |
|---|---|---|---|
| Sensor（传感器模块） | PullBridge（软件管道） | Attitude（yaw / pitch / roll） | Sensor 每帧按需拉取 |
| Sensor（传感器模块） | PullBridge（软件管道） | RobotStatus（敌方颜色 / 血量 / 弹速 / 模式） | Sensor 每帧按需拉取 |
| MCU（下位机） | 串口线（硬件，921600bps） | 云台指令（yaw / pitch 角度 + 角速度 + 距离 + 射击标志 + 目标ID） | 每 5ms 定时发送 |

### 3. 各文件作用

#### 3.1 `timed_serial.hpp / timed_serial.cpp` —— 核心任务类

- **类名**：`hardware::TimedSerial`，继承自 `pipeline::BasicTask`
- **职责**：通信模块的"大脑"，协调发送和接收两路数据流

**核心设计——"两把锁"隔离**：
```
command_mutex_（保护来自 Planner 的数据）
    ├── command_array_[10]   Planner 发来的 10 个指令
    ├── attitude_at_last_frame_  Planner 发来的参考姿态
    ├── plan_period_          Planner 的规划周期（2ms）
    └── command_start_time_   指令接收起始时间戳

sensor_mutex_（保护来自驱动层的数据）
    ├── latest_attitude_      最新 IMU 姿态
    └── latest_robot_status_ 最新裁判系统状态
```

**关键方法**：

| 方法 | 作用 |
|---|---|
| `handle_planner_message(msg)` | PushBridge 回调：收到 Planner 指令时，将指令数组和参考姿态存入 `command_mutex_` 保护区 |
| `handle_attitude_update(att)` | 驱动层回调：收到 IMU 数据时，更新 `latest_attitude_` |
| `handle_status_update(sts)` | 驱动层回调：收到裁判系统数据时，**智能合并**更新 `latest_robot_status_`（保留已有的弹速值，避免被默认值覆盖） |
| `read_latest_command_and_attitude()` | 发送前调用：基于时间戳从指令数组中**线性插值**出当前时刻应该发送的指令 |
| `operator()()` | 主循环：每 5ms 取插值后的指令 → 加上当前 IMU 姿态做补偿 → 调用 `driver_->transmit_cmd()` 发送 |

#### 3.2 `serial_interface.hpp` —— 串口驱动抽象接口

- **设计模式**：依赖注入（Dependency Injection）
- **作用**：定义串口驱动的抽象接口，让 `TimedSerial` 不依赖具体硬件

```cpp
class SerialInterface {
    // 生命周期
    virtual bool init() = 0;
    virtual void start() = 0;
    virtual void close() = 0;

    // 事件回调注册（独立注册，姿态和状态分开）
    virtual void set_attitude_callback(AttitudeCallback cb) = 0;
    virtual void set_robot_status_callback(RobotStatusCallback cb) = 0;

    // 发送指令
    virtual void transmit_cmd(yaw, pitch, yaw_spd, pitch_spd, ...) = 0;
};
```

**为什么要有这个接口？**
- 真机部署时注入 `UartDriver`（真实串口）
- 无硬件调试时注入 `MockDriver`（模拟数据）
- 未来更换硬件平台时，只需实现新的 Driver，不改 `TimedSerial` 一行代码

#### 3.3 `UartIMU/uart_driver.hpp / uart_driver.cpp` —— 真实串口驱动

- **底层库**：`RMCVSerial`（上科大自研串口库），基于 Linux 串口通信
- **波特率**：921600 bps
- **设计原则**：**无状态驱动**——不保存任何 Attitude/RobotStatus 数据，收到数据立即通过回调传给业务层

**接收处理流程**：

```
串口中断
  │
  ├── 命令码 0x1021 (CMD_MCU_DATA)
  │     └── on_receive_imu()
  │           ├── 解析 pc_mcu_data_t 结构体
  │           ├── 构造 Attitude{yaw, pitch, roll} → 触发 attitude_cb_
  │           └── 提取 shoot_speed + autoaim_mode → 触发 status_cb_
  │
  └── 命令码 0x1022 (CMD_ROBOT_DATA)
        └── on_receive_sts()
              ├── 解析 robot_data_t 结构体
              ├── 根据 robot_id 判断敌我颜色（<20=红方, ≥100=蓝方）
              ├── 映射敌方各兵种血量 → 触发 status_cb_
              └── robot_id 不在范围 → enemy_color = GRAY（无效）
```

**发送处理流程**：

```
transmit_cmd(yaw, pitch, yaw_spd, pitch_spd, ...)
  │
  └── 封装 advv_detection_t 结构体
        └── m_serial.send(命令码 0x0503, 数据, 长度)
```

#### 3.4 `UartIMU/packet.hpp` —— 通信协议定义

定义了三类数据包的二进制结构体（使用 `__attribute__((packed))` 确保内存对齐一致）：

| 结构体 | 命令码 | 方向 | 内容 |
|---|---|---|---|
| `advv_detection_t` | `0x0503` | PC→MCU | yaw/pitch 角度、角速度、角加速度、距离、射击标志、目标 ID |
| `pc_mcu_data_t` | `0x1021` | MCU→PC | 当前 yaw/pitch/roll 角度、弹速、自瞄模式标志 |
| `robot_data_t` | `0x1022` | MCU→PC | 全场 16 个实体的血量、己方 robot_id |

#### 3.5 `UartIMU/mock_driver.hpp` —— 模拟驱动

- **用途**：没有真实硬件时的开发和测试
- **行为**：启动时发送一次固定的初始状态（弹速 28m/s，敌方红色，全场满血），不响应发送指令
- **切换方式**：在 `launch.cfg` 中设置 `port=None`

### 4. 实现方法与底层逻辑

#### 4.1 指令插值机制

Planner 以 2ms 为周期规划，生成 10 个前瞻指令。但串口以 5ms 周期发送，两个周期不同步。解决方法：

```
Planner 指令数组: [cmd0][cmd1][cmd2][cmd3]...[cmd9]
                  0ms   2ms   4ms   6ms      18ms

串口发送时刻:         2.5ms (取 cmd1 和 cmd2 之间)
                   ↓
         command_linear_interpolation(cmd1, cmd2, 0.5/2.0)
```

- 根据 `elapsed_time / plan_period` 算出当前落在哪两个指令之间
- 对 yaw/pitch/角速度等连续量做**线性插值**
- 对 `fire_enable` 和 `target_id` 等离散量采用"时间上更接近的值"

#### 4.2 弹速数据合并策略

弹速来自两个渠道，可能冲突：
- IMU 包（0x1021）：包含实际弹速，**实时且准确**
- 裁判系统包（0x1022）：弹速字段为默认值 30m/s，**不准确**

**合并策略**（在 `handle_status_update` 中）：
```
if 新数据弹速 == 默认值(30.0):
    保留已有弹速值，只更新其他字段
else:
    用新数据覆盖弹速
```

#### 4.3 事件驱动架构

```
驱动层（UartDriver）          业务层（TimedSerial）
     │                              │
     │ 收到数据 → 立即回调           │ 回调中更新本地缓存
     │ 不保存状态                   │ 用 mutex 保护共享数据
     │ 不关心数据用途               │ 提供 PullBridge 给下游
```

- 驱动层**无状态**：收到数据立即通过回调抛出，不保存
- 业务层**有状态**：用两把锁分别保护命令数据和传感器数据
- 好处：驱动可替换、可单独测试、职责清晰

---

## 第二部分：目标检测模块（detect/）

### 1. 模块概览

检测模块横跨三条流水线线程，由三个 SubModule 和一个独立的角点优化器组成：

```
Stage0 线程                Stage1 线程              Stage2 线程
┌──────────────┐    ┌──────────────────┐    ┌──────────────────┐
│ Preprocess   │    │     Detect       │    │  CornerRefine    │
│ 图像预处理    │───▶│  神经网络推理     │───▶│  角点亚像素精炼   │
│              │    │                  │    │                  │
│ 640×512 RGB  │    │  AXCL/ONNX/TRT  │    │ 传统视觉优化      │
└──────────────┘    └──────────────────┘    └──────────────────┘
```

### 2. 输入输出

#### Preprocess（预处理）

| | 类型 | 说明 |
|---|---|---|
| **输入** | `ThreadDataPack.frame`（原始图像 BGR） | 来自 Sensor 采集的原始帧 |
| **输出** | `ThreadDataPack.detect_input`（RGB 图 640×512） | 缩放到模型输入尺寸 |
| **输出** | `ThreadDataPack.detect_input_map`（坐标映射） | 记录 ROI 与原始图像的映射关系 |

#### Detect（推理检测）

| | 类型 | 说明 |
|---|---|---|
| **输入** | `ThreadDataPack.detect_input` + `detect_input_map` | 预处理后的图像及映射 |
| **输出** | `ThreadDataPack.bboxes`（`vector<bbox_t>`） | 检测框列表，每个框含 4 个角点坐标、颜色 ID、标签 ID、置信度 |

#### CornerRefine（角点精炼）

| | 类型 | 说明 |
|---|---|---|
| **输入** | `ThreadDataPack.frame`（原始图像）+ `bboxes`（推理结果） | 在原图上对每个检测框进行精炼 |
| **输出** | `ThreadDataPack.bboxes`（角点已更新） | 每个框的 4 个角点被传统视觉方法修正为亚像素精度 |

### 3. 各文件作用

#### 3.1 `preprocess_submodule.hpp/cpp` —— 预处理子模块（Stage0）

- **输入尺寸**：640×512（`INPUT_W=640, INPUT_H=512`）
- **处理步骤**：
  1. 将原始图像缩放到 640×512
  2. 色彩空间转换：BGR → RGB（模型训练时用 RGB）
  3. 记录坐标映射关系 `DetectInputMap`（用于后续把检测结果映射回原图坐标）

```
原始图像 (e.g. 1280×1024 BGR)
     │  cv::resize
     ▼
640×512 BGR
     │  cv::cvtColor(BGR→RGB)
     ▼
640×512 RGB  → 送入神经网络
```

#### 3.2 `detect_submodule.hpp/cpp` —— 推理检测子模块（Stage1）

- **核心逻辑**：
  1. 根据编译宏选择推理后端：`INFERENCE_BACKEND_TYPE==1` → ONNX，`==2` → TensorRT，`==3` → AXCL
  2. 构造函数中加载模型文件，初始化推理引擎
  3. `process()` 中调用 `(*model)(detect_input, output_bboxes)` 执行推理
  4. 利用 `map_bbox_to_source()` 将检测框从模型坐标（640×512）映射回原始图像坐标

```
detect_input (640×512 RGB)
     │  model->forward()
     ▼
output_bboxes（模型坐标系下的四角点）
     │  map_bbox_to_source(detect_input_map, bbox)
     ▼
bboxes（原始图像坐标系下的四角点）
```

#### 3.3 `backend.hpp` —— 推理后端抽象基类

```cpp
class BackEnd {
    virtual void operator()(const cv::Mat &src, std::vector<bbox_t> &det) = 0;
};
```

- 统一的推理接口，所有后端（AXCL/ONNX/TensorRT）都实现此接口
- 定义了公共常量：`TOPK_NUM=128`（最多保留 128 个候选框）、`KEEP_THRES=0.1`（置信度阈值）

#### 3.4 `AXCL/AXCL.hpp / AXCL.cpp` —— 爱芯元智 AX650N 推理后端

- **推理硬件**：AX650N 内置 NPU（神经网络处理器）
- **模型格式**：`.axmodel`（AXCL 专用格式，由 ONNX 转换而来）
- **核心流程**：
  1. `AX_SYS_Init()` 初始化 AX 系统层
  2. `AX_ENGINE_Create()` 从 axmodel 文件创建推理引擎
  3. `prepare_io()` 在 NPU 共享内存中分配输入/输出 buffer（物理地址 + 虚拟地址映射）
  4. 推理时将图像数据拷贝到输入 buffer 的虚拟地址
  5. `AX_ENGINE_RunSync()` 同步执行推理
  6. 从输出 buffer 读取检测结果，解析为 `bbox_t` 格式

- **特点**：直接操作物理内存地址，零拷贝数据交互，适合低延迟场景

#### 3.5 `ONNX/ONNX.hpp / ONNX.cpp` —— ONNX Runtime 推理后端

- **推理框架**：AMD MIGraphX（ROCm 平台）或 ONNX Runtime
- **模型格式**：`.onnx` 通用格式
- **核心流程**：
  1. 解析 ONNX 模型，编译为 MIGraphX 可执行程序
  2. 支持模型缓存（`build_engine_from_cache` / `cache_engine`），避免每次启动重复编译
  3. 推理阈值：`CONF_THRESH=0.65`（置信度）、`NMS_THRESH=0.45`（NMS 重叠阈值）

#### 3.6 `TensorRT/TRTModule.hpp / TRTModule.cpp` —— NVIDIA TensorRT 推理后端

- **推理框架**：NVIDIA TensorRT 8（CUDA 10.2）
- **模型格式**：`.onnx` → TensorRT 引擎（`.cache`）
- **核心流程**：
  1. 从 ONNX 构建 TensorRT 引擎（`build_engine_from_onnx`）
  2. 支持引擎缓存，首次构建耗时 ~30 分钟，之后加载缓存秒级启动
  3. 推理使用 CUDA Stream 异步执行
  4. 自定义 `Logger` 类对接 TensorRT 日志系统

#### 3.7 `corner_refine_submodule.hpp/cpp` —— 角点精炼子模块（Stage2）

- **核心逻辑**：
  1. 遍历每个检测框（`bboxes`）
  2. 根据敌方颜色选择二值化阈值（红色=60, 蓝色=70）
  3. 调用 `ArmorCornerOptimizer::optimizeCorners()` 进行角点精炼
  4. 精炼成功 → 标记 `source=TRADITIONAL`（传统方法修正）
  5. 精炼失败 → 保留原始 YOLO 输出，标记 `source=NEURAL_NETWORK`（纯网络结果）

#### 3.8 `armor_corner_optimizer.hpp/cpp` —— 装甲板角点优化器

这是整个检测模块中算法最复杂的部分，采用**传统计算机视觉方法**对 YOLO 粗检测结果进行亚像素精度修正。

**算法流程**：

```
YOLO 输出的四个角点 (粗精度)
     │
     ├── Step 1: 计算左右灯条的中心、高度、宽度、倾斜角
     │
     ├── Step 2: 自适应 ROI 裁剪（calculateLightRoi）
     │     根据灯条高度和宽度动态计算搜索区域
     │
     ├── Step 3: 图像预处理（preprocessImage）
     │     灰度化 → 二值化（阈值 60~94，按敌方颜色选择）
     │
     ├── Step 4: 灯条搜索（findLightBars）
     │     在 ROI 内查找轮廓 → 筛选符合灯条几何特征的目标
     │     筛选条件：宽高比(0.07~0.6)、倾斜角(<30°)、填充率(>50%)
     │
     ├── Step 5: 最优灯条匹配（selectBestLightBar）
     │     在候选灯条中找最接近 YOLO 原始位置和角度的那个
     │
     └── Step 6: 更新角点坐标
           用精炼后的灯条端点替换 YOLO 输出的角点
```

**灯条筛选参数**（`LightParams`）：
| 参数 | 含义 | 默认值 |
|---|---|---|
| `min_ratio` | 最小宽高比 | 0.07 |
| `max_ratio` | 最大宽高比 | 0.6 |
| `max_angle` | 最大倾斜角 | 30° |
| `min_fill_ratio` | 最小填充率 | 0.5 |

**YOLO 模型特性参数**（`YoloModelCharacteristics`）：
| 参数 | 含义 |
|---|---|
| `image_width / image_height` | 模型输入尺寸 |
| `min_light_height` | 最小灯条像素高度 |
| `max_light_height` | 最大灯条像素高度 |

### 4. 实现方法与底层逻辑

#### 4.1 神经网络检测原理

本系统使用的是**四点模型**（直接回归装甲板四个角点），而非传统的"中心点+宽高"模型。输出格式：

```
YOLO 输出: [四角点 x8坐标, 置信度, 颜色分类, 标签分类]
                ↓
bbox_t {
    pts[4]      ← 装甲板四个角点 (左上、左下、右下、右上)
    color_id    ← 0=红色, 1=蓝色
    tag_id      ← 装甲板标签 ID
    conf        ← 置信度
}
```

四点模型的优势：直接输出装甲板的精确角点，省去传统方法中"检测框 → 寻找灯条 → 确定角点"的中间步骤。

#### 4.2 为什么还需要 CornerRefine（传统视觉精炼）？

神经网络虽然能直接输出四角点，但：
- YOLO 输出的角点精度有限（像素级），在远距离小目标场景下误差较大
- PnP 位姿解算对角点精度极其敏感，角点误差会放大约 10~20 倍到 3D 位置误差

因此用传统视觉方法（二值化+灯条搜索）对角点进行亚像素级修正，神经网络提供"粗定位"，传统方法提供"精定位"。

#### 4.3 三后端切换机制

通过 CMake 编译选项 `-DINFERENCE_BACKEND=AXCL/ONNX/TRT` 控制：

```cpp
// CMakeLists.txt 中生成宏定义
#if INFERENCE_BACKEND_TYPE == 1    // ONNX
    model.reset(new ONNX(OnnxFileName));
#elif INFERENCE_BACKEND_TYPE == 2  // TensorRT
    model.reset(new TRTModule(OnnxFileName));
#elif INFERENCE_BACKEND_TYPE == 3  // AXCL
    model.reset(new AXCL(OnnxFileName));
#endif
```

三个后端实现同一个 `BackEnd` 接口（`operator()(src, det)`），上层 `DetectSubModule` 完全不感知用的是哪个后端。

#### 4.4 坐标映射（map_bbox_to_source）

神经网络在 640×512 的输入上推理，但上游 Preprocess 可能只对 ROI 做了缩放（当前实现是全图缩放），因此需要将检测结果从模型坐标系映射回原始图像坐标系：

```
模型坐标: (x_model, y_model)
     │
     │  x_orig = roi.x + x_model × (roi.width / 640)
     │  y_orig = roi.y + y_model × (roi.height / 512)
     ▼
原始坐标: (x_orig, y_orig)
```

---

## 模块交互关系总结

```
┌─────────────────────────────────────────────────────────────────┐
│                        通信模块 (timedserial)                     │
│                                                                  │
│  上行：MCU → 串口 → UartDriver → 回调 → TimedSerial             │
│         → PullBridge → Sensor                                   │
│                                                                  │
│  下行：Planner → PushBridge → TimedSerial → 插值 →              │
│        UartDriver → 串口 → MCU                                  │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                      检测模块 (detect)                            │
│                                                                  │
│  Preprocess(Stage0) → Detect(Stage1) → CornerRefine(Stage2)     │
│       │                    │                    │                │
│   BGR→RGB             神经网络推理          传统视觉精炼          │
│   缩放 640×512        AXCL/ONNX/TRT        二值化+灯条搜索        │
│                          │                    │                │
│                    四点角点输出          亚像素角点修正            │
└─────────────────────────────────────────────────────────────────┘
```

---

*报告初版，待补充：性能测试数据、参数调优建议、常见问题排查。*
