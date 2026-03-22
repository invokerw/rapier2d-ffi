#!/usr/bin/env bash
# =============================================================================
# rapier2d-ffi 可视化测试编译脚本 (跨平台)
#
# 用法:
#   ./test/build_visual_test.sh          # 编译并运行
#   ./test/build_visual_test.sh build    # 只编译
#   ./test/build_visual_test.sh run      # 只运行 (需要先编译)
#   ./test/build_visual_test.sh setup    # 安装依赖 (raylib)
#
# 前置条件:
#   1. 已编译 rapier2d-ffi:  cargo build --release
#   2. 已安装 raylib:        ./test/build_visual_test.sh setup
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

SRC="$SCRIPT_DIR/visual_test.c"
OUT="$SCRIPT_DIR/visual_test"
LIB_DIR="$PROJECT_DIR/target/release"
HEADER_DIR="$PROJECT_DIR"

# 颜色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# -------------------------------------------------------
# 检测操作系统
# -------------------------------------------------------
detect_os() {
    case "$(uname -s)" in
        Darwin*)  echo "macos" ;;
        Linux*)   echo "linux" ;;
        MINGW*|MSYS*|CYGWIN*) echo "windows" ;;
        *)        echo "unknown" ;;
    esac
}

OS=$(detect_os)

# -------------------------------------------------------
# 安装 raylib
# -------------------------------------------------------
do_setup() {
    info "安装 raylib..."
    case "$OS" in
        macos)
            if command -v brew &>/dev/null; then
                brew install raylib
            else
                error "需要 Homebrew。请先安装: https://brew.sh"
            fi
            ;;
        linux)
            if command -v apt &>/dev/null; then
                info "使用 apt 安装..."
                sudo apt update && sudo apt install -y libraylib-dev
            elif command -v dnf &>/dev/null; then
                info "使用 dnf 安装..."
                sudo dnf install -y raylib-devel
            elif command -v pacman &>/dev/null; then
                info "使用 pacman 安装..."
                sudo pacman -S --noconfirm raylib
            else
                warn "未检测到包管理器，请手动安装 raylib:"
                echo "  https://github.com/raysan5/raylib/releases"
            fi
            ;;
        windows)
            if command -v pacman &>/dev/null; then
                info "使用 MSYS2 pacman 安装..."
                pacman -S --noconfirm mingw-w64-x86_64-raylib
            else
                warn "请通过以下方式安装 raylib:"
                echo "  MSYS2:  pacman -S mingw-w64-x86_64-raylib"
                echo "  vcpkg:  vcpkg install raylib"
                echo "  或从源码编译: https://github.com/raysan5/raylib"
            fi
            ;;
    esac
    info "raylib 安装完成。"
}

# -------------------------------------------------------
# 编译
# -------------------------------------------------------
do_build() {
    # 检查 rapier2d-ffi 是否已编译
    if [ ! -d "$LIB_DIR" ]; then
        error "请先编译 rapier2d-ffi: cargo build --release"
    fi

    info "编译可视化测试 ($OS)..."

    case "$OS" in
        macos)
            # 检查 raylib
            if ! pkg-config --exists raylib 2>/dev/null; then
                # 尝试 brew 路径
                RAYLIB_PREFIX="$(brew --prefix raylib 2>/dev/null || true)"
                if [ -z "$RAYLIB_PREFIX" ] || [ ! -d "$RAYLIB_PREFIX" ]; then
                    error "找不到 raylib。请运行: ./test/build_visual_test.sh setup"
                fi
                RAYLIB_CFLAGS="-I${RAYLIB_PREFIX}/include"
                RAYLIB_LIBS="-L${RAYLIB_PREFIX}/lib -lraylib"
            else
                RAYLIB_CFLAGS="$(pkg-config --cflags raylib)"
                RAYLIB_LIBS="$(pkg-config --libs raylib)"
            fi

            cc -o "$OUT" "$SRC" \
                -I"$HEADER_DIR" $RAYLIB_CFLAGS \
                -L"$LIB_DIR" -lrapier2d_ffi \
                $RAYLIB_LIBS \
                -framework IOKit -framework Cocoa -framework OpenGL \
                -framework Security -framework CoreFoundation \
                -lm -O2
            ;;
        linux)
            if ! pkg-config --exists raylib 2>/dev/null; then
                error "找不到 raylib。请运行: ./test/build_visual_test.sh setup"
            fi

            cc -o "$OUT" "$SRC" \
                -I"$HEADER_DIR" $(pkg-config --cflags raylib) \
                -L"$LIB_DIR" -lrapier2d_ffi \
                $(pkg-config --libs raylib) \
                -lm -lpthread -ldl -O2
            ;;
        windows)
            # MSYS2/MinGW 环境
            cc -o "$OUT.exe" "$SRC" \
                -I"$HEADER_DIR" \
                -L"$LIB_DIR" -lrapier2d_ffi \
                -lraylib -lopengl32 -lgdi32 -lwinmm \
                -lm -O2
            ;;
        *)
            error "不支持的操作系统: $(uname -s)"
            ;;
    esac

    info "编译成功: $OUT"
}

# -------------------------------------------------------
# 运行
# -------------------------------------------------------
do_run() {
    case "$OS" in
        macos)
            if [ ! -f "$OUT" ]; then
                error "可执行文件不存在，请先编译: ./test/build_visual_test.sh build"
            fi
            DYLD_LIBRARY_PATH="$LIB_DIR" "$OUT"
            ;;
        linux)
            if [ ! -f "$OUT" ]; then
                error "可执行文件不存在，请先编译: ./test/build_visual_test.sh build"
            fi
            LD_LIBRARY_PATH="$LIB_DIR" "$OUT"
            ;;
        windows)
            if [ ! -f "$OUT.exe" ]; then
                error "可执行文件不存在，请先编译: ./test/build_visual_test.sh build"
            fi
            PATH="$LIB_DIR:$PATH" "$OUT.exe"
            ;;
    esac
}

# -------------------------------------------------------
# 入口
# -------------------------------------------------------
case "${1:-default}" in
    setup)   do_setup ;;
    build)   do_build ;;
    run)     do_run ;;
    default) do_build && do_run ;;
    *)
        echo "用法: $0 {setup|build|run}"
        echo "  (不带参数 = 编译并运行)"
        exit 1
        ;;
esac
