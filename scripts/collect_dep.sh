#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)
PROJECT_DIR="$SCRIPT_DIR/.."

SRC_BIN_PATH="$PROJECT_DIR/bin/robot_kinematic_viewer"
SRC_CFG_PATH="$PROJECT_DIR/config"
SRC_DOC_PATH="$PROJECT_DIR/docs"
SRC_SHADER_PATH="$PROJECT_DIR/shader"

STAGE_DIR="$PROJECT_DIR/robot_kinematic_viewer_release"
TARGET_BIN_DIR="$STAGE_DIR/bin"
TARGET_LIB_DIR="$STAGE_DIR/lib"
TARGET_CFG_DIR="$STAGE_DIR/config"
TARGET_FONT_DIR="$STAGE_DIR/fonts"

RUN_SCRIPT_PATH="$STAGE_DIR/run.sh"
TEMP_FILE="/tmp/robot_kinematic_viewer_deps.txt"

ensure_dir_exists() {
    local dir="$1"
    if [ ! -d "$dir" ]; then
        mkdir -p "$dir" || {
            echo "❌ 无法创建目录: $dir"
            exit 1
        }
    fi
}

is_system_lib() {
    local lib_path="$1"
    case "$lib_path" in
        /lib/*|/lib64/*|/usr/lib/*|/usr/lib64/*|/usr/local/lib/*|/lib/x86_64-linux-gnu/*|/usr/lib/x86_64-linux-gnu/*)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

# 这些库视为“系统基础库”，不打包（目标机默认应有）
is_core_system_lib() {
    local lib_name="$1"
    local core_patterns=(
        '^ld-linux.*\.so(\..*)?$'
        '^libc\.so(\..*)?$'
        '^libc-[0-9].*\.so$'
        '^libm\.so(\..*)?$'
        '^libm-[0-9].*\.so$'
        '^libdl\.so(\..*)?$'
        '^libdl-[0-9].*\.so$'
        '^libpthread\.so(\..*)?$'
        '^libpthread-[0-9].*\.so$'
        '^librt\.so(\..*)?$'
        '^librt-[0-9].*\.so$'
        '^libgcc_s\.so(\..*)?$'
        '^libstdc\+\+\.so(\..*)?$'
        '^libresolv\.so(\..*)?$'
        '^libresolv-[0-9].*\.so$'
        '^libutil\.so(\..*)?$'
        '^libutil-[0-9].*\.so$'
        '^libnsl\.so(\..*)?$'
        '^libnss_.*\.so(\..*)?$'
        '^linux-vdso\.so(\..*)?$'
    )
    for pattern in "${core_patterns[@]}"; do
        if [[ "$lib_name" =~ $pattern ]]; then
            return 0
        fi
    done
    return 1
}

list_deps_from_ldd() {
    local file="$1"
    ldd "$file" | awk '
        /=> not found/ { print "MISSING:" $1; next }
        /=>/ {
            if ($3 ~ /^\//) print $3;
            next
        }
        /^\// { print $1; next }
    '
}

copy_one_lib() {
    local dep_path="$1"
    local dep_name real_path real_name

    dep_name=$(basename "$dep_path")
    real_path=$(readlink -f "$dep_path")
    real_name=$(basename "$real_path")

    if is_core_system_lib "$real_name"; then
        return 0
    fi

    if [ ! -f "$TARGET_LIB_DIR/$real_name" ]; then
        cp -L "$real_path" "$TARGET_LIB_DIR/"
        COPIED_COUNT=$((COPIED_COUNT + 1))
    fi

    if [ "$dep_name" != "$real_name" ] && [ ! -e "$TARGET_LIB_DIR/$dep_name" ] && [ ! -L "$TARGET_LIB_DIR/$dep_name" ]; then
        ln -s "$real_name" "$TARGET_LIB_DIR/$dep_name"
    fi

    # 补充常见 so 链接：libxxx.so / libxxx.so.major
    if [[ "$real_name" =~ ^(lib.+)\.so\.([0-9]+)(\..*)?$ ]]; then
        local base_so="${BASH_REMATCH[1]}.so"
        local major_so="${BASH_REMATCH[1]}.so.${BASH_REMATCH[2]}"
        [ -e "$TARGET_LIB_DIR/$base_so" ] || [ -L "$TARGET_LIB_DIR/$base_so" ] || ln -s "$real_name" "$TARGET_LIB_DIR/$base_so"
        [ -e "$TARGET_LIB_DIR/$major_so" ] || [ -L "$TARGET_LIB_DIR/$major_so" ] || ln -s "$real_name" "$TARGET_LIB_DIR/$major_so"
    fi

    # 记录待递归分析项
    if [ -z "${SEEN_REAL_LIBS[$real_path]:-}" ]; then
        SEEN_REAL_LIBS["$real_path"]=1
        LIB_QUEUE+=("$TARGET_LIB_DIR/$real_name")
    fi
}

is_allowed_runtime_path() {
    local p="$1"
    case "$p" in
        "$TARGET_LIB_DIR"/*|/lib/*|/lib64/*|/usr/lib/*|/usr/lib64/*|/usr/local/lib/*|/lib/x86_64-linux-gnu/*|/usr/lib/x86_64-linux-gnu/*)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

if [ ! -f "$SRC_BIN_PATH" ]; then
    echo "❌ 错误: 可执行文件不存在: $SRC_BIN_PATH"
    echo "请先执行编译脚本。"
    exit 1
fi

echo "========== 准备发布目录 =========="
rm -rf "$STAGE_DIR"
ensure_dir_exists "$TARGET_BIN_DIR"
ensure_dir_exists "$TARGET_LIB_DIR"
ensure_dir_exists "$TARGET_CFG_DIR"

cp -L "$SRC_BIN_PATH" "$TARGET_BIN_DIR/"
SRC_PLUGIN_PATH="$PROJECT_DIR/lib/librkv_point_cloud_pcl.so"
if [ -f "$SRC_PLUGIN_PATH" ]; then
    cp -L "$SRC_PLUGIN_PATH" "$TARGET_LIB_DIR/"
fi
cp -R "$SRC_CFG_PATH"/. "$TARGET_CFG_DIR/"

if [ -d "$SRC_DOC_PATH" ]; then
    cp -R "$SRC_DOC_PATH" "$STAGE_DIR/"
fi
if [ -d "$SRC_SHADER_PATH" ]; then
    cp -R "$SRC_SHADER_PATH" "$STAGE_DIR/"
fi

echo "✅ 二进制与配置拷贝完成"

echo "========== 递归收集动态库依赖 =========="
declare -A SEEN_REAL_LIBS=()
declare -a LIB_QUEUE=("$TARGET_BIN_DIR/robot_kinematic_viewer")
if [ -f "$TARGET_LIB_DIR/librkv_point_cloud_pcl.so" ]; then
    LIB_QUEUE+=("$TARGET_LIB_DIR/librkv_point_cloud_pcl.so")
fi
COPIED_COUNT=0
SKIPPED_CORE_COUNT=0
MISSING_COUNT=0

idx=0
while [ $idx -lt ${#LIB_QUEUE[@]} ]; do
    cur="${LIB_QUEUE[$idx]}"
    idx=$((idx + 1))
    [ -e "$cur" ] || continue

    while IFS= read -r dep; do
        [ -n "$dep" ] || continue
        if [[ "$dep" == MISSING:* ]]; then
            echo "❌ 缺失依赖: ${dep#MISSING:} (from $cur)"
            MISSING_COUNT=$((MISSING_COUNT + 1))
            continue
        fi

        if [ ! -e "$dep" ]; then
            echo "❌ 依赖路径不存在: $dep (from $cur)"
            MISSING_COUNT=$((MISSING_COUNT + 1))
            continue
        fi

        dep_real_name=$(basename "$(readlink -f "$dep")")
        if is_core_system_lib "$dep_real_name"; then
            SKIPPED_CORE_COUNT=$((SKIPPED_CORE_COUNT + 1))
            continue
        fi

        copy_one_lib "$dep"
    done < <(list_deps_from_ldd "$cur")
done

if [ $MISSING_COUNT -gt 0 ]; then
    echo "❌ 依赖收集失败：存在 $MISSING_COUNT 个缺失依赖"
    exit 1
fi

echo "✅ 依赖收集完成: 复制 $COPIED_COUNT 个，跳过核心系统库 $SKIPPED_CORE_COUNT 个"

rm -f "$TEMP_FILE"

echo "========== 处理中文字体（可选） =========="
CFG_FILE="$TARGET_CFG_DIR/robot_kinematic_viewer.yaml"
font_from_cfg=""
if [ -f "$CFG_FILE" ]; then
    font_from_cfg=$(awk -F: '/^[[:space:]]*cjk_font_path:/ {
        val=$0
        sub(/^[^:]*:[[:space:]]*/, "", val)
        gsub(/"/, "", val)
        gsub(/'\''/, "", val)
        print val
        exit
    }' "$CFG_FILE")
fi

font_candidate=""
if [ -n "$font_from_cfg" ] && [ -f "$font_from_cfg" ]; then
    font_candidate="$font_from_cfg"
else
    for f in \
        /usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc \
        /usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc \
        /usr/share/fonts/truetype/wqy/wqy-zenhei.ttc \
        /usr/share/fonts/truetype/wqy/wqy-microhei.ttc
    do
        if [ -f "$f" ]; then
            font_candidate="$f"
            break
        fi
    done
fi

if [ -n "$font_candidate" ] && [ -f "$font_candidate" ] && [ -f "$CFG_FILE" ]; then
    ensure_dir_exists "$TARGET_FONT_DIR"
    font_name=$(basename "$font_candidate")
    cp -L "$font_candidate" "$TARGET_FONT_DIR/$font_name"

    sed -i -E 's|^([[:space:]]*cjk_font_path:[[:space:]]*).*$|\1"fonts/'"$font_name"'"|' "$CFG_FILE"
    echo "✅ 已写入打包字体: fonts/$font_name"
else
    echo "⚠️ 未找到可打包的中文字体，将使用目标机器系统字体"
fi

echo "========== 生成 run.sh =========="
cat > "$RUN_SCRIPT_PATH" << 'EOF'
#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)
export LD_LIBRARY_PATH="$SCRIPT_DIR/lib:${LD_LIBRARY_PATH:-}"

usage() {
    echo "Usage: $0 [config_path] [--self-test]"
    echo "  --self-test  仅执行依赖自检，不启动 robot_kinematic_viewer"
}

is_allowed_path() {
    local p="$1"
    case "$p" in
        "$SCRIPT_DIR/lib"/*|/lib/*|/lib64/*|/usr/lib/*|/usr/lib64/*|/usr/local/lib/*|/lib/x86_64-linux-gnu/*|/usr/lib/x86_64-linux-gnu/*)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

self_test() {
    echo "========== robot_kinematic_viewer_release 自检 =========="
    local missing=0
    local escaped=0
    local items=("$SCRIPT_DIR/bin/robot_kinematic_viewer")

    while IFS= read -r f; do
        items+=("$f")
    done < <(find "$SCRIPT_DIR/lib" -maxdepth 1 -type f -name "*.so*")

    for item in "${items[@]}"; do
        [ -e "$item" ] || continue
        while IFS= read -r line; do
            [ -n "$line" ] || continue
            if [[ "$line" == *"not found"* ]]; then
                echo "❌ 缺失依赖: $line (from $item)"
                missing=$((missing + 1))
                continue
            fi

            dep_path=""
            if [[ "$line" == *"=>"* ]]; then
                dep_path=$(echo "$line" | awk '{print $3}')
            elif [[ "$line" =~ ^/ ]]; then
                dep_path=$(echo "$line" | awk '{print $1}')
            fi

            if [ -n "$dep_path" ] && ! is_allowed_path "$dep_path"; then
                echo "❌ 发现打包外非系统依赖: $dep_path (from $item)"
                escaped=$((escaped + 1))
            fi
        done < <(LD_LIBRARY_PATH="$SCRIPT_DIR/lib" ldd "$item")
    done

    if [ "$missing" -gt 0 ] || [ "$escaped" -gt 0 ]; then
        echo "❌ 自检失败: missing=$missing escaped=$escaped"
        return 1
    fi

    echo "✅ 自检通过：依赖完整且无打包外第三方库"
    return 0
}

if [ "${1:-}" == "-h" ] || [ "${1:-}" == "--help" ]; then
    usage
    exit 0
fi

if [ "${1:-}" == "--self-test" ]; then
    self_test
    exit $?
fi

for arg in "$@"; do
    if [ "$arg" == "--self-test" ]; then
        self_test
        exit $?
    fi
done

CONFIG_PATH="${1:-$SCRIPT_DIR/config/robot_kinematic_viewer.yaml}"
if [ "${CONFIG_PATH:0:2}" != "--" ] && [ ! -f "$CONFIG_PATH" ]; then
    echo "⚠️ 指定配置文件不存在，将回退到默认配置: $SCRIPT_DIR/config/robot_kinematic_viewer.yaml"
    CONFIG_PATH="$SCRIPT_DIR/config/robot_kinematic_viewer.yaml"
fi

cd "$SCRIPT_DIR"
exec "$SCRIPT_DIR/bin/robot_kinematic_viewer" "$CONFIG_PATH"
EOF
chmod +x "$RUN_SCRIPT_PATH"

echo "========== 运行时依赖完整性检查 =========="
check_missing=0
check_escaped=0
CHECK_ITEMS=("$TARGET_BIN_DIR/robot_kinematic_viewer")
while IFS= read -r f; do
    CHECK_ITEMS+=("$f")
done < <(find "$TARGET_LIB_DIR" -maxdepth 1 -type f -name "*.so*")

for item in "${CHECK_ITEMS[@]}"; do
    while IFS= read -r dep; do
        [ -n "$dep" ] || continue
        if [[ "$dep" == MISSING:* ]]; then
            echo "❌ 打包后仍缺失依赖: ${dep#MISSING:} (from $item)"
            check_missing=$((check_missing + 1))
            continue
        fi
        if ! is_allowed_runtime_path "$dep"; then
            echo "❌ 发现打包外非系统依赖: $dep (from $item)"
            check_escaped=$((check_escaped + 1))
        fi
    done < <(LD_LIBRARY_PATH="$TARGET_LIB_DIR" list_deps_from_ldd "$item")
done

if [ $check_missing -gt 0 ] || [ $check_escaped -gt 0 ]; then
    echo "❌ 运行时依赖检查失败: missing=$check_missing escaped=$check_escaped"
    exit 1
fi

echo "✅ 运行时依赖检查通过"

echo "✅ 发布目录已生成: $STAGE_DIR"
