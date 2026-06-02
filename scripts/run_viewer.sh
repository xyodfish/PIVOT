#!/usr/bin/env bash
# Scenario launcher for robot_kinematic_viewer (plugin-style registry).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PREPARE_PY="$SCRIPT_DIR/rkv_prepare_config.py"
SESSION_SH="$SCRIPT_DIR/rkv_session.sh"

VIEWER_BIN="${RKV_VIEWER_BIN:-$REPO_ROOT/bin/robot_kinematic_viewer}"
DEFAULT_BASE_CONFIG="config/robot_kinematic_viewer.yaml"

# shellcheck disable=SC2034
declare -A scenario_desc=()
declare -A scenario_base=()
declare -A scenario_trajectory=()
declare -A scenario_title=()
declare -A scenario_config=()
declare -A scenario_point_cloud_enable=()
declare -A scenario_point_cloud_file=()
declare -A scenario_point_cloud_build_esdf=()

_register() {
  local name="$1" desc="$2"
  scenario_desc["$name"]="$desc"
  shift 2
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --base) scenario_base["$name"]="$2"; shift 2 ;;
      --trajectory) scenario_trajectory["$name"]="$2"; shift 2 ;;
      --title) scenario_title["$name"]="$2"; shift 2 ;;
      --config) scenario_config["$name"]="$2"; shift 2 ;;
      --point-cloud-enable) scenario_point_cloud_enable["$name"]=1; shift 1 ;;
      --point-cloud) scenario_point_cloud_file["$name"]="$2"; shift 2 ;;
      --point-cloud-build-esdf) scenario_point_cloud_build_esdf["$name"]=1; shift 1 ;;
      *) echo "internal: unknown key $1" >&2; exit 2 ;;
    esac
  done
}

_register default "Galbot 默认 IK 调试（无预装轨迹）" \
  --base "$DEFAULT_BASE_CONFIG"

_register galbot "同 default" \
  --base "$DEFAULT_BASE_CONFIG"

_register unitree "Unitree G1（固定基座）" \
  --config "config/robot_kinematic_viewer_unitree_g1.yaml"

_register dance "Galbot 跳舞：底盘滑入 10s" \
  --base "$DEFAULT_BASE_CONFIG" \
  --trajectory "config/trajectories/galbot_g1_dance_slide_in.csv" \
  --title "跳舞滑入"

_register dance_full "Galbot 跳舞：完整 ~56s" \
  --base "$DEFAULT_BASE_CONFIG" \
  --trajectory "config/trajectories/galbot_g1_dance_full.csv" \
  --title "跳舞完整"

_register dance_arm "Galbot 跳舞：手臂段" \
  --base "$DEFAULT_BASE_CONFIG" \
  --trajectory "config/trajectories/galbot_g1_dance_left_action1.csv" \
  --title "跳舞手臂"

_register playback "Galbot 常规回放示例" \
  --base "$DEFAULT_BASE_CONFIG" \
  --trajectory "config/trajectories/galbot_g1_playback.csv" \
  --title "回放示例"

_register collision_debug "移动底盘 WBC 碰撞任务调试" \
  --base "$DEFAULT_BASE_CONFIG" \
  --trajectory "config/trajectories/galbot_mobile_wbc_ik_collision_task_debug_collision_task.csv" \
  --title "碰撞调试"

_register teach "示教程序 demo（侧边栏 Teach）" \
  --base "$DEFAULT_BASE_CONFIG" \
  --title "示教 demo"

_register map "全局地图点云 global_cloud.pcd" \
  --base "$DEFAULT_BASE_CONFIG" \
  --title "地图点云" \
  --point-cloud-enable \
  --point-cloud "global_cloud.pcd"

_register map_esdf "PCD 构建 ESDF 并可视化 (global_cloud.pcd)" \
  --base "$DEFAULT_BASE_CONFIG" \
  --title "地图 ESDF" \
  --point-cloud-enable \
  --point-cloud "global_cloud.pcd" \
  --point-cloud-build-esdf

DO_BUILD=false
SCENARIO=""
EXTRA_URDF=""
EXTRA_TRAJ=()
SESSION_DIR=""
LIST_ONLY=false
DRY_RUN=false

show_help() {
  cat <<EOF
用法: $(basename "$0") [选项] [场景名]

场景化启动 robot_kinematic_viewer。在仓库根目录执行，或设置 RKV_VIEWER_BIN。

选项:
  -h, --help           显示帮助
  -l, --list           列出已注册场景
  -s, --scenario NAME  场景名（也可作为第一个位置参数）
  --urdf PATH          覆盖 robot.urdf_path
  --trajectory PATH    额外预装轨迹（可重复；覆盖场景默认轨迹列表）
  --session DIR        使用 rkv_session 保存的快照目录启动
  --build              启动前先执行 ./build.sh
  --dry-run            只打印将执行的命令
  -n, --dry-run        同 --dry-run

环境变量:
  RKV_URDF             同 --urdf
  RKV_VIEWER_BIN       可执行文件路径（默认: 仓库 bin/robot_kinematic_viewer）

示例:
  ./scripts/run_viewer.sh dance
  ./scripts/run_viewer.sh --scenario unitree --urdf /path/to/g1.urdf
  ./scripts/run_viewer.sh --session sessions/rkv_20260101_120000
  RKV_URDF=/path/to/robot.urdf ./scripts/run_viewer.sh galbot

EOF
  echo "已注册场景:"
  for name in $(list_scenarios); do
    printf "  %-16s %s\n" "$name" "${scenario_desc[$name]}"
  done
}

list_scenarios() {
  printf '%s\n' "${!scenario_desc[@]}" | LC_ALL=C sort
}

resolve_path() {
  local p="$1"
  if [[ "$p" = /* ]]; then
    echo "$p"
  else
    echo "$REPO_ROOT/$p"
  fi
}

prepare_from_scenario() {
  local name="$1"
  local urdf="${EXTRA_URDF:-${RKV_URDF:-}}"
  local has_traj=false
  [[ ${#EXTRA_TRAJ[@]} -gt 0 ]] && has_traj=true
  [[ -n "${scenario_trajectory[$name]:-}" ]] && has_traj=true

  if [[ -n "${scenario_config[$name]:-}" && -z "$urdf" && "$has_traj" == false \
        && -z "${scenario_title[$name]:-}" ]]; then
    echo "$(resolve_path "${scenario_config[$name]}")"
    return
  fi

  local base
  if [[ -n "${scenario_config[$name]:-}" ]]; then
    base="$(resolve_path "${scenario_config[$name]}")"
  else
    base="$(resolve_path "${scenario_base[$name]:-$DEFAULT_BASE_CONFIG}")"
  fi
  if [[ ! -f "$base" ]]; then
    echo "❌ 场景 $name 的 base 配置不存在: $base" >&2
    exit 1
  fi

  local -a prep_args=(--base "$base" --repo-root "$REPO_ROOT")
  if [[ -n "$urdf" ]]; then
    prep_args+=(--urdf "$urdf")
  fi

  if [[ ${#EXTRA_TRAJ[@]} -gt 0 ]]; then
    for t in "${EXTRA_TRAJ[@]}"; do
      prep_args+=(--trajectory "$(resolve_path "$t")")
    done
  elif [[ -n "${scenario_trajectory[$name]:-}" ]]; then
    prep_args+=(--trajectory "$(resolve_path "${scenario_trajectory[$name]}")")
  fi

  if [[ -n "${scenario_title[$name]:-}" ]]; then
    prep_args+=(--title-suffix "${scenario_title[$name]}")
  fi
  if [[ -n "${scenario_point_cloud_enable[$name]:-}" ]]; then
    prep_args+=(--point-cloud-enable)
    if [[ -n "${scenario_point_cloud_file[$name]:-}" ]]; then
      prep_args+=(--point-cloud-file "${scenario_point_cloud_file[$name]}")
    fi
    if [[ -n "${scenario_point_cloud_build_esdf[$name]:-}" ]]; then
      prep_args+=(--point-cloud-build-esdf)
    fi
  fi

  if [[ ! -f "$PREPARE_PY" ]]; then
    echo "❌ 缺少 $PREPARE_PY" >&2
    exit 1
  fi
  python3 "$PREPARE_PY" "${prep_args[@]}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) show_help; exit 0 ;;
    -l|--list) LIST_ONLY=true; shift ;;
    -s|--scenario) SCENARIO="$2"; shift 2 ;;
    --urdf) EXTRA_URDF="$2"; shift 2 ;;
    --trajectory) EXTRA_TRAJ+=("$2"); shift 2 ;;
    --session) SESSION_DIR="$2"; shift 2 ;;
    --build) DO_BUILD=true; shift ;;
    --dry-run|-n) DRY_RUN=true; shift ;;
    --) shift; break ;;
    -*) echo "未知选项: $1" >&2; show_help >&2; exit 1 ;;
    *)
      if [[ -z "$SCENARIO" ]]; then
        SCENARIO="$1"
      else
        echo "多余参数: $1" >&2
        exit 1
      fi
      shift
      ;;
  esac
done

if [[ "$LIST_ONLY" == true ]]; then
  list_scenarios | while read -r n; do
    printf "%-16s %s\n" "$n" "${scenario_desc[$n]}"
  done
  exit 0
fi

if [[ "$DO_BUILD" == true ]]; then
  (cd "$REPO_ROOT" && ./build.sh)
fi

if [[ -n "$SESSION_DIR" ]]; then
  if [[ ! -x "$SESSION_SH" ]]; then
    chmod +x "$SESSION_SH" 2>/dev/null || true
  fi
  exec "$SESSION_SH" replay "$SESSION_DIR" ${DRY_RUN:+--dry-run}
fi

if [[ -z "$SCENARIO" ]]; then
  SCENARIO="default"
fi

if [[ -z "${scenario_desc[$SCENARIO]:-}" ]]; then
  echo "❌ 未知场景: $SCENARIO（可用 --list 查看）" >&2
  exit 1
fi

if [[ ! -x "$VIEWER_BIN" ]]; then
  echo "❌ 找不到可执行文件: $VIEWER_BIN" >&2
  echo "   请先 ./build.sh 或设置 RKV_VIEWER_BIN" >&2
  exit 1
fi

CONFIG_PATH="$(prepare_from_scenario "$SCENARIO")"
if [[ ! -f "$CONFIG_PATH" ]]; then
  echo "❌ 配置不存在: $CONFIG_PATH" >&2
  exit 1
fi

cd "$REPO_ROOT"
CMD=("$VIEWER_BIN" "$CONFIG_PATH")

echo "场景: $SCENARIO — ${scenario_desc[$SCENARIO]}"
echo "配置: $CONFIG_PATH"
echo "命令: ${CMD[*]}"

if [[ "$DRY_RUN" == true ]]; then
  exit 0
fi

exec "${CMD[@]}"
