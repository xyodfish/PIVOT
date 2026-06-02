# 调试会话与 Log 回放设计

## 现状（已实现）

### 场景化启动 — `scripts/run_viewer.sh`

注册表方式启动 Viewer，避免每次手改 YAML 或记 CSV 路径：

```bash
./scripts/run_viewer.sh --list
./scripts/run_viewer.sh dance
RKV_URDF=/path/to/robot.urdf ./scripts/run_viewer.sh galbot
```

场景通过 `scripts/rkv_prepare_config.py` 生成临时 YAML（可覆盖 URDF、预装轨迹、窗口标题）。

### 会话快照 — `scripts/rkv_session.sh`

对标 `ros2_humble_docker` 里 `galbot_vis_*/logs` 的**可复现目录**，但不解析 Galbot 业务日志：

```bash
# 保存当前 config + user_obstacles + 配置里引用的轨迹 CSV
./scripts/rkv_session.sh save my_debug --config config/robot_kinematic_viewer.yaml --note "碰撞 case"

./scripts/rkv_session.sh list
./scripts/run_viewer.sh --session sessions/rkv_20260602_143000_my_debug
```

目录结构：

```text
sessions/rkv_YYYYMMDD_HHMMSS[_name]/
  manifest.env      # 元数据
  config.yaml       # 启动配置快照
  user_obstacles.yaml   # 若存在则复制
  trajectories/     # 配置中 playback.trajectory_files 引用的 CSV
  notes.txt         # 可选备注
```

**限制**：快照不会自动捕获 Viewer 运行中未写回磁盘的 UI 状态（例如仅存在于内存的障碍物编辑、未保存的轨迹）。退出前应在 Viewer 内保存轨迹/障碍物，或把 `config/robot_kinematic_viewer.yaml` 设为已持久化状态后再 `save`。

---

## 与 ros2_humble_docker `log_gui` 的差异

| 能力 | log_gui (Galbot) | robot_kinematic_viewer |
|------|------------------|-------------------------|
| 数据源 | `galbot_store` 等文本 log，解析 `--program shelf` 参数字符串 | 会话目录 + 已有 CSV/YAML |
| 回放动作 | 调 `run_vis.sh` 重跑 Python 可视化任务 | 启动 C++ Viewer + 轨迹回放 |
| 参数编辑 | Tk 表格改 perception/place 参数 | 需在 Viewer 或 YAML 中改 |

本仓库**没有** Galbot `galbot_store` 日志格式，因此短期内不做同款 log 解析器。

---

## 后续可做（按优先级）

### Phase A — 加强会话（低成本）

- [ ] `save` 时可选 `--include-video` / 复制最近录制的 MP4
- [ ] `save` 从 Viewer 进程内触发（写 `sessions/` 钩子，需 C++ 侧写配置回调）
- [ ] `replay` 自动 `cp` 快照内 `user_obstacles.yaml` 到 `config/`（若 Viewer 支持启动时加载路径配置）

### Phase B — 结构化运行记录（不依赖 Galbot log）

Viewer 或脚本在退出时写 `sessions/.../run_record.json`：

```json
{
  "scenario": "dance",
  "urdf": "/path/to/robot.urdf",
  "trajectory_files": ["config/trajectories/..."],
  "playback_duration_sec": 10.0,
  "ik_mode": "full_body"
}
```

便于 `list` 展示摘要，无需解析自由文本 log。

### Phase C — 导入外部 log（按需）

若需对齐历史 Galbot 日志：

1. 定义 **最小子集** 解析规则（例如只提取 `JointState` 序列或 `--program` 行里的 `object_position`）
2. 提供 `scripts/import_galbot_vis_log.py` → 输出 CSV 或 `run_record.json`
3. 用 `run_viewer.sh --trajectory` 回放

若无稳定 log 格式样本，Phase C 应等有真实文件再实现。

### Phase D — 轻量 GUI（可选）

类似 `log_gui.py` 的 Tk 工具只负责：

- 浏览 `sessions/rkv_*`
- 显示 `manifest.env` / `notes.txt`
- 按钮调用 `run_viewer.sh --session`

与 Viewer 本体解耦，可后做。

---

## 建议工作流

1. 日常调试：`./scripts/run_viewer.sh <场景>`
2. 复现问题：调参并保存 CSV/YAML 后 → `./scripts/rkv_session.sh save bug_123 --note "..."`
3. 给他人：`tar czf bug_123.tgz -C sessions rkv_...` 对方解压后 `--session` 启动
