# Robot Kinematic Viewer 功能设计（MVP v1）

## 1. 目标
面向机器人日常开发调试，优先解决三类高频需求：
- 可重复的轨迹录制与回放。
- 快速发现姿态下的潜在自碰风险。
- 保持现有 IK / 关节拖动流程不被破坏。

本版本强调“好用、稳定、可扩展”，不追求花哨特效。

## 2. MVP 范围
### 2.1 轨迹回放（高优先级）
- 关键帧录制（支持设置关键帧间隔 dt）。
- 播放/暂停/停止/循环。
- 时间轴拖拽（scrub）并立即应用到场景。
- 关键帧列表查看、删除指定帧。
- 当前播放段索引、总时长显示。

### 2.2 碰撞预警与距离显示（高优先级）
- 基于每个 link visual 的包围球（proxy sphere）做实时近似距离检测。
- 输出最小表面距离、最近 link 对、对应空间连线。
- 支持 Warning / Danger 双阈值分级。
- 默认忽略同一 link 内部 proxy 与父子 link 对，降低噪声。

### 2.3 可快速做好的附加功能
- 新增“安全”页集中展示碰撞监控。
- 在 3D 视窗绘制最近 link 对连线（颜色分级）。

## 3. 架构与封装
### 3.1 模块划分
- `kinematic_playback`：轨迹状态机与插值采样。
- `kinematic_collision_monitor`：碰撞代理构建后的距离评估。
- `kinematic_sidebar_panels`：仅负责 UI 展示与交互输入，不做核心算法。
- `rkv::RobotScene`：提供只读几何代理接口（link collision proxies）。

### 3.2 设计模式
- Strategy
  - 轨迹插值策略：`TrajectoryInterpolator`（当前实现 `LinearTrajectoryInterpolator`）。
  - 碰撞对过滤策略：`CollisionPairFilterStrategy`（当前实现 `DefaultCollisionPairFilterStrategy`）。
- State
  - 轨迹播放模式：`Stopped / Playing / Paused`。
- Facade
  - 主循环通过简洁接口调用 `TrajectoryPlayer` 与 `CollisionMonitor`，避免逻辑堆在 `main`。

## 4. 数据流
1. 用户在 UI 记录关键帧 -> `TrajectoryPlayer` 更新 `DebugPlaybackState`。
2. 每帧主循环调用 `TrajectoryPlayer::advanceAndApply` -> 将采样关节值写回 `RobotScene`。
3. `RobotScene::updateTransforms` 后，`CollisionMonitor` 从 scene 读取 link proxies。
4. `CollisionMonitor` 计算最小距离并返回 `CollisionMonitorResult`。
5. UI 面板展示数值，3D 视窗绘制最近对连线。

## 5. 计算模型说明
### 5.1 轨迹插值
- 按关键帧时间戳做分段线性插值。
- 对缺失关节字段采取“跳过不写入”，兼容不同录制版本。

### 5.2 碰撞距离
- 每个 visual 预计算局部包围球（center/radius）。
- 运行时映射到世界系并做球-球表面距离：
  - `surface_distance = ||c1-c2|| - (r1+r2)`
- 小于 0 代表代理已穿透（高风险）。

## 6. 扩展路线（后续）
- 用 URDF collision mesh 代替 visual mesh，提升准确性。
- 增加邻接白名单/黑名单配置。
- 增加轨迹导入导出（YAML/JSON）与回归测试脚本化接口。
