#!/usr/bin/env bash
# Save / list / replay debugging sessions (config + trajectories + obstacles snapshot).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
RUN_VIEWER="$SCRIPT_DIR/run_viewer.sh"
SESSIONS_ROOT="${RKV_SESSIONS_DIR:-$REPO_ROOT/sessions}"

usage() {
  cat <<EOF
用法:
  $(basename "$0") save [名称] [--config PATH] [--note TEXT]
  $(basename "$0") list
  $(basename "$0") replay <session_dir> [--dry-run]

save   将当前调试状态快照到 sessions/rkv_YYYYMMDD_HHMMSS[_name]/
list   列出已有会话
replay 用快照中的 config.yaml 启动 Viewer（走 run_viewer.sh）

环境变量:
  RKV_SESSIONS_DIR   会话根目录（默认: 仓库 sessions/）

EOF
}

timestamp_id() {
  date +"%Y%m%d_%H%M%S"
}

cmd_save() {
  local name=""
  local config="${RKV_CONFIG:-$REPO_ROOT/config/robot_kinematic_viewer.yaml}"
  local note=""

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --config) config="$2"; shift 2 ;;
      --note) note="$2"; shift 2 ;;
      -h|--help) usage; exit 0 ;;
      *)
        if [[ -z "$name" ]]; then
          name="$1"
        else
          echo "未知参数: $1" >&2
          exit 1
        fi
        shift
        ;;
    esac
  done

  config="$(cd "$REPO_ROOT" && realpath -m "$config" 2>/dev/null || echo "$config")"
  if [[ ! -f "$config" ]]; then
    echo "❌ 配置不存在: $config" >&2
    exit 1
  fi

  local id
  id="$(timestamp_id)"
  if [[ -n "$name" ]]; then
    name="$(echo "$name" | tr ' /' '__')"
    id="${id}_${name}"
  fi

  local dest="$SESSIONS_ROOT/rkv_${id}"
  mkdir -p "$dest/trajectories"

  cp "$config" "$dest/config.yaml"

  local obstacles="$REPO_ROOT/config/user_obstacles.yaml"
  if [[ -f "$obstacles" ]]; then
    cp "$obstacles" "$dest/user_obstacles.yaml"
  fi

  # Copy trajectories referenced in config (best-effort parse).
  if command -v python3 >/dev/null 2>&1; then
    python3 - <<'PY' "$config" "$dest" "$REPO_ROOT"
import shutil
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.exit(0)

config_path = Path(sys.argv[1])
dest = Path(sys.argv[2])
repo = Path(sys.argv[3])

with config_path.open(encoding="utf-8") as f:
    cfg = yaml.safe_load(f) or {}

files = (cfg.get("playback") or {}).get("trajectory_files") or []
for rel in files:
    src = Path(rel)
    if not src.is_absolute():
        src = repo / src
    if src.is_file():
        shutil.copy2(src, dest / "trajectories" / src.name)
PY
  fi

  {
    echo "id=rkv_${id}"
    echo "created=$(date -Iseconds)"
    echo "source_config=$config"
    [[ -n "$note" ]] && echo "note=$note"
  } >"$dest/manifest.env"

  if [[ -n "$note" ]]; then
    echo "$note" >"$dest/notes.txt"
  fi

  echo "✅ 已保存会话: $dest"
  echo "   回放: ./scripts/run_viewer.sh --session $dest"
}

cmd_list() {
  if [[ ! -d "$SESSIONS_ROOT" ]]; then
    echo "(无 sessions 目录)"
    exit 0
  fi
  local found=0
  while IFS= read -r d; do
    found=1
    local note=""
    [[ -f "$d/notes.txt" ]] && note="$(head -n1 "$d/notes.txt")"
    local created=""
    [[ -f "$d/manifest.env" ]] && created="$(grep '^created=' "$d/manifest.env" | cut -d= -f2-)"
    printf "%s  %s  %s\n" "$(basename "$d")" "${created:-?}" "${note:-}"
  done < <(find "$SESSIONS_ROOT" -maxdepth 1 -type d -name 'rkv_*' | LC_ALL=C sort -r)
  [[ "$found" -eq 1 ]] || echo "(暂无 rkv_* 会话)"
}

cmd_replay() {
  local session_dir="${1:-}"
  local dry=false
  shift || true
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --dry-run|-n) dry=true; shift ;;
      *) echo "未知选项: $1" >&2; exit 1 ;;
    esac
  done

  if [[ -z "$session_dir" ]]; then
    echo "❌ 请指定会话目录" >&2
    usage >&2
    exit 1
  fi

  if [[ ! -d "$session_dir" ]]; then
    # Allow bare id without path
    if [[ -d "$SESSIONS_ROOT/$session_dir" ]]; then
      session_dir="$SESSIONS_ROOT/$session_dir"
    elif [[ -d "$SESSIONS_ROOT/rkv_${session_dir}" ]]; then
      session_dir="$SESSIONS_ROOT/rkv_${session_dir}"
    else
      echo "❌ 会话目录不存在: $session_dir" >&2
      exit 1
    fi
  fi

  session_dir="$(cd "$session_dir" && pwd)"
  local cfg="$session_dir/config.yaml"
  if [[ ! -f "$cfg" ]]; then
    echo "❌ 会话内缺少 config.yaml: $session_dir" >&2
    exit 1
  fi

  # Launch from repo root; config paths inside snapshot may be relative to repo.
  local -a args=()
  [[ "$dry" == true ]] && args+=(--dry-run)

  # Direct launch with saved config (bypass scenario registry).
  local viewer="${RKV_VIEWER_BIN:-$REPO_ROOT/bin/robot_kinematic_viewer}"
  if [[ ! -x "$viewer" ]]; then
    echo "❌ 找不到 $viewer" >&2
    exit 1
  fi

  echo "回放会话: $(basename "$session_dir")"
  echo "配置: $cfg"
  if [[ "$dry" == true ]]; then
    echo "命令: $viewer $cfg"
    exit 0
  fi

  cd "$REPO_ROOT"
  exec "$viewer" "$cfg"
}

main() {
  local cmd="${1:-}"
  shift || true
  case "$cmd" in
    save) cmd_save "$@" ;;
    list|ls) cmd_list ;;
    replay) cmd_replay "$@" ;;
    -h|--help|"") usage ;;
    *)
      echo "未知子命令: $cmd" >&2
      usage >&2
      exit 1
      ;;
  esac
}

main "$@"
