# 英雄外参 A/B DRY_RUN 测试

A = R-only 鲁棒 R＋机械 IMU 到光心 T。
B = 联合 R＋联合 T 换算到相同的真实 IMU 原点。
运行坐标分量顺序为左、上、前，单位米。

机械交点到光心为 [89.5,99.9,60.68] mm。
交点到 IMU = 交点到光心 − IMU 到光心 = [-64.28,67.87,57.89] mm。
B 的真实 IMU 到光心 T = [166.566384,78.331758,18.808781] mm。

在机器人桌面终端执行，先停掉已有 auto-aim 进程：

    cd /home/rcs/rm_project/SHtech_auto_aim
    bash tools/start_hero_extrinsic_ab.sh A near

近处固定一个可被自瞄稳定识别的目标。A、B 两次运行之间保持底盘和目标位置不变。
按低 pitch 的左、中、右，再按高 pitch 的左、中、右采 6 个停稳姿态。
pitch 使用机械允许范围内的低/高档即可，不需要小于零。
每次停稳 1–2 秒，确认有效识别，再点击 sensor_submodule 窗口按 P，保持 1 秒，然后换姿态。
P 可保存任意目标画面，不要求棋盘。确认终端 TX/DRY_RUN 的 distance 为正，避免记录无有效目标的 TX。
保存成功提示为 Saved pair 编号。RX missing/stale 时不会保存。

Ctrl+C 退出后运行：

    bash tools/start_hero_extrinsic_ab.sh B near

按近处 A 的顺序和大致相同姿态重复 6 次。
随后把目标移远，固定好，依次执行：

    bash tools/start_hero_extrinsic_ab.sh A far
    bash tools/start_hero_extrinsic_ab.sh B far

总计四批，每批建议 6 对图像/RX。A/B 不要求姿态完全一致，但同距离比较时目标和底盘不能移动。
近远只需保持目标能稳定检测并形成明显距离差，不预设未知场地距离。

启动脚本打印 Output 完整路径。每批新目录位于项目根目录，名称 extrinsic_ab_候选_距离标签_日期时间_进程号。
保存：frame_编号.png、euler_编号.txt、frame_编号.json（采图时间、RX、缓存年龄）、
xuc.log（带毫秒时间戳的 RX/TX）、runtime.log、session.json、该批配置快照。

脚本强制 AUTOAIM_XUC_DRY_RUN=1，阻断 XUC UART 下发，并采用独立测试配置：
geometry_only=1 完整跳过弹道飞行时间和下坠补偿；yaw_comp=pitch_comp=0。
geometry_only 在未启用 DRY_RUN 时会拒绝初始化。
程序仍包含检测、滤波、预测和 Planner；DRY_RUN 不代表下位机自身逻辑已关闭。
日常相机配置仍为 A；日常规划器的弹道与固定补偿参数没有更改。

仅静态检查、不启动硬件：

    bash tools/start_hero_extrinsic_ab.sh --check

采集完成后可由助手分析四个目录，或执行：

    python3 -B tools/analyze_hero_extrinsic_ab_logs.py 目录A近 目录B近 目录A远 目录B远 --output-dir 一个新结果目录

分析程序用采图后 0.05–0.75 秒内、有目标的 TX 估计停稳姿态的目标角；不是曝光/串口硬件同步。
TX−RX 是算法要求的修正量，不是电机执行误差。
要判断绝对瞄准精度，还需可靠炮口方向基准下手动对准目标的参考 RX；缺少此参考时只报告一致性，不自动选赢家。

已用现有 34 对图像做静态几何公式对照。它不运行检测/预测，不是实际串口 TX。
共同交点原点下，固定棋盘位置 RMS：A=9.843899 mm，B=1.456762 mm。
当前静态公式的 A/B pitch 差中位数为3.783293度。
B 在这34张图上拟合过，训练误差较小不等于独立验证通过。
逐样本数据和三维方向对照：
/home/rcs/rm_project/SHtech_auto_aim/log/extrinsic-ab-prepare-20261012-224014/geometry_comparison

备份与构建记录：/home/rcs/rm_project/SHtech_auto_aim/log/extrinsic-ab-prepare-20261012-224014
