# 哨兵相机标定报告

## 标定时间
2026-08-05

## 标定设备
- 相机: Hikrobot MV-CS016-10UC (1440x1080, BayerRG)
- IMU: HI91 (CH340, 115200 baud)
- 棋盘格: 12x9 squares / 11x8 inner corners / 15mm

## 标定流程

### 1. 内参标定 (K/D)
- 数据: 93帧棋盘格图像
- 方法: OpenCV calibrateCamera + CALIB_FIX_K3
- 结果: RMS = 0.2466 px
- K = [2347.55, 2359.40, 620.00, 615.40]
- D = [-0.064, 0.235, 0.007, -0.008, 0]

### 2. 手眼标定 (R_c2i)
- 数据: 60帧 (过滤: delta<=20ms + reproj<=1.5px)
- 方法: SVD + 鲁棒离群值剔除 (4次迭代)
- 轴映射: yaw=col1, pitch=col2, roll=col0
- 旋转顺序: ZXY (Rz(yaw) @ Rx(pitch) @ Ry(roll))
- 结果:
  - g1: 4.88 deg (26帧)
  - g3: 3.12 deg (34帧)
  - ALL: 3.88 deg (RMS)
  - R_c2i 角度: 96.25 deg (相机与IMU轴夹角)

### 3. 平移标定 (T_c2i)
- 来源: 2022年原始团队物理测量值 (未优化)
- T_c2i = [0.045, -0.055, 0.116] m
- 说明: 手眼标定无法求解T_c2i (尺度模糊), 需要CAD尺寸或多距离标定

## 文件说明
- cam_param_sentry.yml: 完整参数 (K/D + R_c2i + T_c2i)
- baseline_R_c2i_SVD.npy: R_c2i矩阵 (numpy格式)
- cam_param_handeye_final.txt: R_c2i矩阵 (文本格式)

## 已知限制
1. T_c2i 为手工测量值, 精度未知
2. 时间同步误差 ~10ms, 运动中会放大
3. 3.88 deg 是数据质量的地板, 无法进一步优化

## 标定脚本
位于本机: final_svd.py
