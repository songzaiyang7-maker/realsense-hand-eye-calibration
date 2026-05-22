# RealSense 手眼标定

**基于 Intel RealSense + 机械臂的 eye-in-hand 手眼标定工具包，支持多算法对比、异常点排除、自动诊断。**

---

## 创新点

- **多算法自动对比选优** — 同时使用 4 种算法（Tsai-Lenz、Park、Horaud、Daniilidis）求解，自动选误差最小的，并报告跨算法一致性。
- **异常点检测与排除** — 从验证输出中人工识别误差偏大的样本，用 `--exclude` 参数排除，避免个别坏数据影响整体精度。
- **角度变化诊断** — 自动分析 rx/ry/rz 变化范围，不足时给出具体建议（如"ry 变化只有 5°，需至少 20~30°"）。
- **正确的验证公式** — 使用 `gripper2base @ cam2gripper @ target2cam = 常量`（目标在基座坐标系下的一致性验证），区别于常见但错误的 `||AX - XB||` 验证。
- **平移标准差分析** — 输出三轴平移标准差（mm），直观评估精度。
- **实时角度反馈** — 采集时相机画面实时显示机械臂 rx/ry/rz 角度及累计变化范围，不足时红色警告，辅助采集高质量数据。

---

## 架构

```
Windows (RealSense)                          WSL2 (ROS2)
┌──────────────────────┐                    ┌────────────────────────────┐
│ collect_camera.py    │──ZMQ 触发信号──────▶│ record_arm.py              │
│  ├─ 棋盘格检测       │   (端口 5558)       │  └─ 订阅机械臂位姿          │
│  ├─ solvePnP → 位姿  │                    │     ToolVectorActual       │
│  └─ 保存 camera.json │                    │  └─ 保存 arm.json          │
└──────────────────────┘                    └────────────────────────────┘
                       ┌────────────────────┐
                       │ handeye_solver.py  │
                       │  ├─ 读取两个 JSON   │
                       │  ├─ 4 种算法求解    │
                       │  ├─ 自动选最优      │
                       │  └─ 验证 + 保存     │
                       └────────────────────┘
```

---

## 脚本说明

| 脚本 | 运行环境 | 用途 |
|------|----------|------|
| `collect_camera.py` | **Windows**（需要 RealSense + pyrealsense2） | 棋盘格检测、相机位姿记录、实时显示臂角度 |
| `record_arm.py` | **WSL2 / Linux**（需要 ROS2 + 机械臂驱动） | 订阅臂位姿、触发记录、回传角度数据 |
| `handeye_solver.py` | **任意**（纯计算） | 四种算法解算 AX=XB、验证、输出结果 |

---

## 快速开始

### 1. 采集相机数据（Windows 端）

```bash
pip install pyrealsense2 opencv-python pyzmq numpy
python collect_camera.py --pattern 4 6 --square 15 --zmq-port 5558
```

检测到棋盘格（绿色叠加）时按 **空格** 记录。建议采集 10~15 组，手腕角度尽量多样化。

### 2. 记录臂数据（ROS2 / WSL2 端）

```bash
# 查看 Windows IP
ip route show default
# → default via 172.23.224.1

# 启动记录器
ros2 run <你的包名> record_arm --ros-args \
    -p windows_host:=172.23.224.1 -p zmq_port:=5558
```

### 3. 解算标定

将两个 JSON 文件放到同一目录，运行：

```bash
python handeye_solver.py --camera camera_data.json --arm arm_data.json
```

输出：`handeye_result.json`，包含 4x4 相机到末端变换矩阵。

排除坏样本（从验证输出中人工识别误差明显偏大的 idx）：
```bash
python handeye_solver.py --camera camera_data.json --arm arm_data.json --exclude 4 7
```

---

## 输入数据格式

### camera_data.json（由 collect_camera.py 生成）

```json
{
  "pattern": [4, 6],
  "square_mm": 15.0,
  "camera_matrix": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
  "dist_coeffs": [...],
  "poses": [
    {"idx": 1, "rvec": [...], "tvec": [...], "timestamp": ...},
    ...
  ]
}
```

### arm_data.json（由 record_arm.py 生成）

```json
{
  "arm_type": "dobot",
  "poses": [
    {"idx": 1, "x": 350.5, "y": -120.3, "z": 400.1, "rx": 180.0, "ry": 5.2, "rz": 90.0},
    ...
  ]
}
```

单位：位置 mm，角度度。欧拉角约定可通过 `--euler` 配置（默认 `zyx` = Rz@Ry@Rx）。

---

## 参数说明

### collect_camera.py

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--pattern` | 4 6 | 棋盘格内角点数（列, 行）— 根据你的标定板调整 |
| `--square` | 15.0 | 格子边长（mm）— 根据你的标定板调整 |
| `--zmq-port` | 无 | ZMQ 触发端口，用于同步记录臂数据 |

### handeye_solver.py

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--camera` | camera_data.json | 相机数据文件路径 |
| `--arm` | arm_data.json | 臂数据文件路径 |
| `--out` | handeye_result.json | 输出文件路径 |
| `--exclude` | 无 | 排除的样本索引，如 `--exclude 4 7` |
| `--euler` | zyx | 欧拉角约定：`zyx`、`xyz`、`zyz` |

---

## 算法说明

| 方法 | 说明 |
|------|------|
| **Tsai-Lenz** | 经典两阶段法（先旋转后平移） |
| **Park** | 基于 Lie 群，数据分布好时精度最高 |
| **Horaud** | 基于特征值分解 |
| **Daniilidis** | 基于对偶四元数，抗噪声能力强 |

解算器会同时尝试四种方法，报告各方法的误差，自动选择最优（验证误差最小）。

---

## 坐标系约定

```
AX = XB  其中:
  A = target_to_camera   (solvePnP 结果)
  B = gripper_to_base    (机械臂控制器)
  X = camera_to_gripper  (标定结果)
```

验证公式：
```
T_target_in_base = gripper2base @ cam2gripper @ target2cam
```
该值在所有样本中应恒定 — 偏差越大，标定误差越大。

---

## 示例输出

```json
{
  "method": "Park",
  "euler_convention": "zyx",
  "matrix_4x4": [[...4x4...]],
  "translation_mm": [58.54, 71.35, 117.35],
  "translation_std_mm": [0.83, 0.57, 0.87],
  "verification_errors": [0.0, 0.012, 0.008, ...]
}
```

---

## 依赖

- Python 3.8+
- OpenCV（含 contrib，需要 `calibrateHandEye`）
- NumPy
- pyzmq（同步采集时需要）
- pyrealsense2（`collect_camera.py` 需要）
- ROS2 Humble（`record_arm.py` 需要）

---

## 许可证

MIT

---
---

# RealSense Hand-Eye Calibration

**Eye-in-hand calibration toolkit for Intel RealSense + robot arm, with multi-method comparison, outlier exclusion, and automatic diagnostics.**

---

## Highlights

- **Multi-method auto-comparison** — Solves with 4 algorithms (Tsai-Lenz, Park, Horaud, Daniilidis) simultaneously, auto-selects the best, and reports cross-method consistency.
- **Outlier exclusion** — `--exclude` flag to manually drop bad samples identified from the verification output, preventing them from degrading overall accuracy.
- **Arm angle diagnostics** — Automatically analyzes rx/ry/rz variation range; warns when rotation diversity is insufficient with specific suggestions.
- **Correct verification** — Uses `gripper2base @ cam2gripper @ target2cam = constant` (target-in-base-frame consistency), not the common but incorrect `||AX - XB||`.
- **Translation std analysis** — Reports per-axis translation standard deviation in mm for intuitive accuracy assessment.
- **Real-time angle feedback** — Displays arm rx/ry/rz and accumulated rotation range on the camera feed during data collection, with red warnings when diversity is insufficient.

---

## Architecture

```
Windows (RealSense)                          WSL2 (ROS2)
┌──────────────────────┐                    ┌────────────────────────────┐
│ collect_camera.py    │──ZMQ trigger──────▶│ record_arm.py              │
│  ├─ Chessboard detect│   (port 5558)      │  └─ Subscribes to          │
│  ├─ solvePnP → pose  │                    │     ToolVectorActual       │
│  └─ Save camera.json │                    │  └─ Saves arm.json         │
└──────────────────────┘                    └────────────────────────────┘
                       ┌────────────────────┐
                       │ handeye_solver.py  │
                       │  ├─ Loads both JSON│
                       │  ├─ 4 methods solve│
                       │  ├─ Auto-select best│
                       │  └─ Verify + save  │
                       └────────────────────┘
```

---

## Script Overview

| Script | Runs on | Purpose |
|--------|---------|---------|
| `collect_camera.py` | **Windows** (needs RealSense + pyrealsense2) | Chessboard detection, camera pose recording, arm angle display |
| `record_arm.py` | **WSL2 / Linux** (needs ROS2 + robot driver) | Subscribe to arm pose, record on trigger, broadcast angles back |
| `handeye_solver.py` | **Either** (pure computation) | Solve AX=XB with 4 methods, verify, output result |

---

## Quick Start

### 1. Collect camera data (Windows)

```bash
pip install pyrealsense2 opencv-python pyzmq numpy
python collect_camera.py --pattern 4 6 --square 15 --zmq-port 5558
```

Press **SPACE** when chessboard is detected (green overlay). Aim for 10-15 samples with diverse wrist orientations.

### 2. Record arm data (ROS2 / WSL2)

```bash
# Check Windows IP
ip route show default
# → default via 172.23.224.1

# Run recorder
ros2 run <your_pkg> record_arm --ros-args \
    -p windows_host:=172.23.224.1 -p zmq_port:=5558
```

### 3. Solve calibration

Copy both JSON files to the same directory, then:

```bash
python handeye_solver.py --camera camera_data.json --arm arm_data.json
```

Output: `handeye_result.json` with the 4x4 camera-to-gripper transform.

To exclude bad samples (identified manually from the verification output — look for outlier indices with significantly larger errors):
```bash
python handeye_solver.py --camera camera_data.json --arm arm_data.json --exclude 4 7
```

---

## Input Data Format

### camera_data.json (from collect_camera.py)

```json
{
  "pattern": [4, 6],
  "square_mm": 15.0,
  "camera_matrix": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
  "dist_coeffs": [...],
  "poses": [
    {"idx": 1, "rvec": [...], "tvec": [...], "timestamp": ...},
    ...
  ]
}
```

### arm_data.json (from record_arm.py)

```json
{
  "arm_type": "dobot",
  "poses": [
    {"idx": 1, "x": 350.5, "y": -120.3, "z": 400.1, "rx": 180.0, "ry": 5.2, "rz": 90.0},
    ...
  ]
}
```

Units: position in mm, angles in degrees. The Euler convention is configurable via `--euler` (default: `zyx` = Rz@Ry@Rx).

---

## Parameters

### collect_camera.py

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--pattern` | 4 6 | Chessboard inner corners (cols, rows) — adjust to match your board |
| `--square` | 15.0 | Square side length in mm — adjust to match your board |
| `--zmq-port` | None | ZMQ trigger port for synchronized arm recording |

### handeye_solver.py

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--camera` | camera_data.json | Camera data file path |
| `--arm` | arm_data.json | Arm data file path |
| `--out` | handeye_result.json | Output file path |
| `--exclude` | (none) | Sample indices to exclude, e.g. `--exclude 4 7` |
| `--euler` | zyx | Euler angle convention: `zyx`, `xyz`, or `zyz` |

---

## Algorithms

| Method | Description |
|--------|-------------|
| **Tsai-Lenz** | Classic two-stage method (rotation then translation) |
| **Park** | Lie group based; often most accurate for well-distributed data |
| **Horaud** | Eigenvalue decomposition approach |
| **Daniilidis** | Dual quaternion based; handles noise well |

The solver tries all four and reports each method's error. The best (lowest verification error) is selected automatically.

---

## Coordinate Convention

```
AX = XB  where:
  A = target_to_camera   (from solvePnP)
  B = gripper_to_base    (from robot controller)
  X = camera_to_gripper  (the calibration result)
```

Verification formula:
```
T_target_in_base = gripper2base @ cam2gripper @ target2cam
```
This should be constant across all samples — any variation indicates calibration error.

---

## Example Output

```json
{
  "method": "Park",
  "euler_convention": "zyx",
  "matrix_4x4": [[...4x4...]],
  "translation_mm": [58.54, 71.35, 117.35],
  "translation_std_mm": [0.83, 0.57, 0.87],
  "verification_errors": [0.0, 0.012, 0.008, ...]
}
```

---

## Requirements

- Python 3.8+
- OpenCV (with contrib, for `calibrateHandEye`)
- NumPy
- pyzmq (for synchronized data collection)
- pyrealsense2 (for `collect_camera.py`)
- ROS2 Humble (for `record_arm.py`)

---

## License

MIT
