# 依赖安装指南

本文档说明 `robot_kinematic_viewer` 所需的系统依赖及其安装方式。

---

## 基础编译环境

```bash
sudo apt update
sudo apt install -y build-essential cmake git pkg-config
```

---

## 图形与窗口

```bash
sudo apt install -y libgl1-mesa-dev libglu1-mesa-dev libglew-dev libglfw3-dev
```

---

## 模型加载

```bash
sudo apt install -y libassimp-dev
```

---

## ROS Noetic

参考官方安装文档：http://wiki.ros.org/noetic/Installation/Ubuntu

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full
sudo apt install -y ros-noetic-roscpp
```

---

## pinocchio

```bash
sudo apt install -y robotpkg-pinocchio
```

或从源码构建（推荐与项目使用的版本一致）：
https://github.com/stack-of-tasks/pinocchio

---

## trac_ik

```bash
sudo apt install -y ros-noetic-trac-ik
```

---

## yaml-cpp

```bash
sudo apt install -y libyaml-cpp-dev
```

---

## Eigen3

```bash
sudo apt install -y libeigen3-dev
```

---

## qpOASES（可选，full-body IK 后端推荐）

```bash
sudo apt install -y libqpoases-dev
```

---

## FFmpeg（MP4 视频录制）

```bash
sudo apt install -y libavcodec-dev libavformat-dev libavutil-dev libswscale-dev ffmpeg
```

---

## libgif（GIF 导出）

```bash
sudo apt install -y libgif-dev
```

---

## 一键安装（Ubuntu 20.04）

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake git pkg-config \
    libgl1-mesa-dev libglu1-mesa-dev libglew-dev libglfw3-dev \
    libassimp-dev \
    libyaml-cpp-dev libeigen3-dev \
    libavcodec-dev libavformat-dev libavutil-dev libswscale-dev ffmpeg \
    libgif-dev \
    ros-noetic-roscpp ros-noetic-trac-ik
```

> 注：pinocchio 与 qpOASES 若通过 apt 安装版本不匹配，可能需要从源码构建。

---

## 内置依赖

以下库已随仓库内置，无需额外安装：

- `deps/imgui` —  immediate-mode GUI
- `deps/imguizmo` — 3D gizmo manipulator
- `deps/glad` — OpenGL loader
- `deps/vp` — 速度规划库（trapezoidal / double-S）
