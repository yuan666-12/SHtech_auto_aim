# Hero 自瞄

英雄机器人自瞄项目：相机图像 → 装甲板检测 → 灯条精修 → PnP 定位 → 运动估计 → 弹道规划 → XUC 串口。

当前配置使用 `SKD250526.onnx`、英雄相机参数和中心预瞄策略。默认只跟踪；显示一个完整缩放的检测窗口。

## 编译和运行

运行环境为 Linux。依赖 C++20 编译器、CMake、OpenCV 4、Eigen3、ONNX Runtime；使用真实相机时还需海康 MVS SDK。

```bash
# MVS_PATH 指向当前机器实际安装的 SDK 库目录，例如 /opt/MVS/lib/64。
export MVS_PATH=/opt/MVS/lib/64
./build.sh
./build_real/auto-aim
```

在项目根目录运行。默认 `AUTOAIM_TRACK_ONLY=1`，收到下位机自瞄模式许可后可下发云台跟踪角度，开火建议为 0。终端按 Ctrl+C 正常退出。

`INFERENCE_BACKEND` 可选 ONNX、TRT、AXCL；默认 ONNX。`USE_HIKCAM=OFF` 可编译视频输入版本。旧 RMCVSerial 驱动仅在 `ENABLE_LEGACY_UART=ON` 时需要，Hero 的 `xuc:` 链路不依赖它。

Windows 可编辑源码并通过 Git 同步；编译和硬件运行仍在 Linux 完成。文本文件统一 UTF-8/LF，构建目录、日志和采集数据不提交。

项目目录可以改名。`build.sh`、离线检查和标定采集入口会根据脚本位置定位当前项目；外参修正工具输出到当前项目的 `asset/`。改名或移动后应重新编译，旧可执行文件可能仍包含编译时的资源绝对路径。若随项目移动了 `build_real/`、`build_checks/`，先移走旧构建目录再编译，避免复用旧 CMake 缓存。小电脑上的桌面快捷方式或服务若使用绝对路径，也需随实际部署路径更新。

## 离线验证

```bash
python3 tools/run_offline_checks.py
# 或只执行一个测试，例如：
python3 tools/run_offline_checks.py --case refactor
```

测试从当前源码编译，使用合成观测、内存串口和相机 SDK 替身，不打开真实设备。测试二进制位于 `build_checks/`，最新摘要位于 `log/checks/latest/`。

## 目录

| 目录 | 内容 |
|---|---|
| `common/` | 流水线、数据结构、消息桥、火控和日志 |
| `detect/` | 图像预处理、推理后端、角点精修 |
| `predict/` | 装甲板关联、目标锁定、卡尔曼运动估计 |
| `planner/` | 弹道、中心预瞄、可选 MPC |
| `sensor/` | 相机、视频和标定采集 |
| `timedserial/` | XUC 收发、角度限速、发送时火控检查 |
| `asset/` | 生效相机/规划参数、当前模型 |
| `tools/` | 回归检查、标定工具和测试输入 |
| `thirdparty/`、`planner/tinympc/` | 保留的第三方代码 |

最新参数结论见 [启动参数.md](启动参数.md)，坐标约定见 [坐标系定义.md](坐标系定义.md)。历史报告、旧候选参数、重复源码和旧运行数据已从工作目录移除，整理前完整备份保留在项目目录外。

源码长度检查可选安装 `requirements-dev.txt` 后运行 `python3 tools/check_function_size.py`；第三方库和海康 SDK 头文件不参与应用函数拆分。
