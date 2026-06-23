#!/usr/bin/env bash
set -euo pipefail

if [ -t 1 ]; then
  GREEN='\033[1;32m'
  CYAN='\033[1;36m'
  BLUE='\033[1;34m'
  YELLOW='\033[1;33m'
  MAGENTA='\033[1;35m'
  WHITE='\033[1;37m'
  NC='\033[0m'
else
  GREEN='' ; CYAN='' ; BLUE='' ; YELLOW='' ; MAGENTA='' ; WHITE='' ; NC=''
fi

render_line() {
  local line="$1"
  line="${line//\{G\}/$GREEN}"
  line="${line//\{C\}/$CYAN}"
  line="${line//\{B\}/$BLUE}"
  line="${line//\{Y\}/$YELLOW}"
  line="${line//\{M\}/$MAGENTA}"
  line="${line//\{W\}/$WHITE}"
  line="${line//\{N\}/$NC}"
  printf '%b\n' "$line"
}

render_box_line() {
  local color="$1"
  local text="$2"
  local padded
  printf -v padded '%-72s' "$text"
  printf '%b║ %s ║%b\n' "$color" "$padded" "$NC"
}

render_line "{G}╔══════════════════════════════════════════════════════════════════════════╗{N}"
render_box_line "$GREEN" "[ ROBOT MODEL ] => [ IK / PLAYBACK ] => [ COLLISION / UI ] => [ VIEWER ]"
render_line "{G}╠══════════════════════════════════════════════════════════════════════════╣{N}"
render_box_line "$GREEN" ""
render_box_line "$CYAN"  "                   ____   ___  __      __   ___    _____ "
render_box_line "$BLUE"  "                  |  _ \\ |_ _| \\ \\    / /  / _ \\  |_   _|"
render_box_line "$MAGENTA" "                  | |_) | | |   \\ \\  / /  | | | |   | |  "
render_box_line "$YELLOW" "                  |  __/  | |    \\ \\/ /   | |_| |   | |  "
render_box_line "$GREEN" "                  |_|    |___|    \\__/     \\___/    |_|  "
render_box_line "$GREEN" ""
render_box_line "$WHITE" "   PIVOT : Pose and IK Visualization for Obstacles and Trajectories"
render_box_line "$CYAN"  "             [ visualize / inspect / teach / playback ]"
render_box_line "$GREEN" ""
render_line "{G}╚══════════════════════════════════════════════════════════════════════════╝{N}"
