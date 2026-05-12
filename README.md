# Robot Kinematic Viewer 👋
### 🤖 Interactive Kinematic Debug Viewer · IK · Playback · Safety Monitor

[English](#-english) | [中文](#-中文)

---

## 📑 Contents

- [🌍 English](#-english)
- [✨ Key Features](#-key-features)
- [🖼️ Preview](#️-preview)
- [🚀 Quick Start](#-quick-start)
- [🎮 RViz IK Demo](#-rviz-ik-demo)
- [中文](#-中文)
- [✨ 主要能力](#-主要能力)
- [🖼️ 预览](#️-预览)
- [🚀 快速开始](#-快速开始)
- [🎮 RViz 联调示例](#-rviz-联调示例)

## 🌍 English

`robot_kinematic_viewer` is a practical desktop tool for robot kinematic debugging.
It provides a 3D interactive viewer with IK target manipulation, trajectory playback, and near-collision monitoring.

### ✨ Key Features

- Interactive 3D robot view (OpenGL + ImGui)
- Single-chain IK and full-body IK backend switching
- Keyframe recording and trajectory playback
- Safety panel with proxy-sphere distance monitoring
- User-defined obstacle editing and visualization
- Optional ROS bridge for external IK target input

### 🖼️ Preview

![Robot Kinematic Viewer — interactive 3D view, IK, and playback](assets/gui_viewer.png)

Additional screenshots or short clips can live under `docs/assets/` if you want to split demos by feature (for example `overview.png`, `ik_demo.gif`, `playback_demo.gif`).

### 🧠 Core Workflow

```text
Load URDF + config
  -> Interactive marker / UI target edits
  -> IK solve (single_chain or full_body)
  -> Scene update
  -> Collision monitor (proxy spheres)
  -> UI + line overlay feedback
```

### 🗂️ Repository Layout

```text
robot_kinematic_viewer/
  include/                 # public headers
  src/                     # core implementation
  config/                  # YAML runtime configs
  scripts/                 # build/deploy and RViz integration scripts
  docs/                    # design docs
  deps/                    # vendored imgui/imguizmo/glad sources
```

### 🧰 Dependencies

- CMake >= 3.16
- C++17 compiler
- OpenGL, GLFW, GLEW, Assimp
- pinocchio, trac_ik, roscpp (ROS Noetic)
- yaml-cpp
- qpOASES (recommended for full-body backend)

### 🚀 Quick Start

Build only:

```bash
./build.sh
```

Full rebuild:

```bash
./all_rebuild.sh
```

Build + package runtime release bundle:

```bash
./auto_build.sh
```

Run with YAML config:

```bash
./bin/robot_kinematic_viewer config/robot_kinematic_viewer.yaml
```

Run directly with URDF path:

```bash
./bin/robot_kinematic_viewer /absolute/path/to/robot.urdf
```

### 🎮 RViz IK Demo

Start ROS + interactive marker + viewer stack:

```bash
./scripts/start_rviz_ik_stack.sh
```

Useful options:

```bash
./scripts/start_rviz_ik_stack.sh --ik-mode full_body --backend wbc_chain_ik --chain-index 0
./scripts/start_rviz_ik_stack.sh --pose-ik
```

### ⚙️ Configuration Notes

- Default startup config: `config/robot_kinematic_viewer.yaml`
- First CLI argument is treated as config path unless it ends with `.urdf`
- Update `robot.urdf_path` in YAML before first run on a new machine
- Keep top-level YAML keys limited to:
  - `window`, `robot`, `camera`, `ui`, `ik`, `ros`, `initial_pose`

### 🛣️ Roadmap

- Replace visual-proxy collision with URDF collision geometry
- Add trajectory import/export (YAML/JSON)
- Improve pair-filter customization for collision checking
- Add more automated regression tests

---

## 中文

`robot_kinematic_viewer` 是一个面向机器人日常调试的桌面可视化工具。
它提供了 3D 交互视图、IK 目标操控、轨迹回放和近碰撞监控能力。

### ✨ 主要能力

- OpenGL + ImGui 交互式机器人 3D 视图
- `single_chain` / `full_body` IK 模式切换
- 关键帧录制与轨迹回放
- 基于代理球（proxy sphere）的最小距离安全监控
- 用户障碍物编辑与可视化
- 可选 ROS 外部目标位姿接入

### 🖼️ 预览

![Robot Kinematic Viewer — 交互 3D、IK 与回放](assets/robot_kinematic_viewer.gif)

如需按功能拆分展示，可把更多截图或短视频放在 `docs/assets/`（例如 `overview.png`、`ik_demo.gif`、`playback_demo.gif`）。

### 🧠 核心流程

```text
加载 URDF 与配置
  -> UI/外部目标输入
  -> IK 求解
  -> 场景状态更新
  -> 距离监控与风险分级
  -> 侧边栏与3D连线反馈
```

### 🗂️ 仓库结构

```text
robot_kinematic_viewer/
  include/                 # 对外头文件
  src/                     # 核心实现
  config/                  # 运行配置
  scripts/                 # 构建、打包、RViz 联调脚本
  docs/                    # 设计文档
  deps/                    # 内置三方源码（imgui/imguizmo/glad）
```

### 🧰 依赖环境

- CMake 3.16+
- C++17 编译器
- OpenGL / GLFW / GLEW / Assimp
- pinocchio / trac_ik / roscpp（ROS Noetic）
- yaml-cpp
- qpOASES（full-body 后端推荐）

### 🚀 快速开始

仅编译：

```bash
./build.sh
```

全量重编译：

```bash
./all_rebuild.sh
```

编译并生成可发布目录（含依赖收集）：

```bash
./auto_build.sh
```

使用配置文件启动：

```bash
./bin/robot_kinematic_viewer config/robot_kinematic_viewer.yaml
```

直接指定 URDF 启动：

```bash
./bin/robot_kinematic_viewer /绝对路径/robot.urdf
```

### 🎮 RViz 联调示例

一键拉起 `roscore + interactive marker + viewer`：

```bash
./scripts/start_rviz_ik_stack.sh
```

常见参数：

```bash
./scripts/start_rviz_ik_stack.sh --ik-mode full_body --backend wbc_chain_ik --chain-index 0
./scripts/start_rviz_ik_stack.sh --pose-ik
```

### ⚙️ 配置说明

- 默认入口配置：`config/robot_kinematic_viewer.yaml`
- 程序首个参数若以 `.urdf` 结尾则按 URDF 直接启动，否则按 YAML 配置加载
- 在新机器上请先修改 YAML 中 `robot.urdf_path`
- 顶层配置项需保持为：
  - `window`、`robot`、`camera`、`ui`、`ik`、`ros`、`initial_pose`

### 📚 文档

- 设计文档：`docs/ROBOT_KINEMATIC_VIEWER_DESIGN.md`

### 🛣️ 后续规划

- 用 URDF collision mesh 提升碰撞检测精度
- 增加轨迹导入导出（YAML/JSON）
- 完善碰撞对过滤策略可配置能力
- 补齐自动化回归测试

---

## 📄 License

TBD
