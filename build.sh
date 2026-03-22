#!/usr/bin/env bash
# =============================================================================
# rapier2d-ffi 跨平台编译脚本
#
# 用法:
#   ./build.sh              # 编译所有平台（需要对应的工具链）
#   ./build.sh mac          # 只编译 macOS (arm64 + x86_64 通用二进制)
#   ./build.sh linux        # 只编译 Linux (x86_64, 需要交叉编译工具链)
#   ./build.sh windows      # 只编译 Windows (x86_64, 需要交叉编译工具链)
#   ./build.sh setup        # 安装所有需要的 Rust target 和工具链
#
# 输出目录: dist/<平台>/
# =============================================================================

set -euo pipefail

# 项目根目录（脚本所在目录）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# 库名（与 Cargo.toml 中的 name 对应，连字符会被转为下划线）
LIB_NAME="rapier2d_ffi"
# 头文件名（build.rs 生成的）
HEADER_NAME="rapier2d_ffi.h"
# 输出根目录
DIST_DIR="$SCRIPT_DIR/dist"

# 检测宿主系统
HOST_OS="$(uname -s)"   # Darwin | Linux
HOST_ARCH="$(uname -m)" # x86_64 | arm64 | aarch64

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# -------------------------------------------------------
# 安装所需的 Rust target 和工具链
# -------------------------------------------------------
do_setup() {
    info "安装所需的 Rust 编译目标 (targets)..."

    # macOS
    rustup target add aarch64-apple-darwin
    rustup target add x86_64-apple-darwin

    # Linux (需要交叉编译器 - 见下方提示)
    rustup target add x86_64-unknown-linux-gnu
    rustup target add aarch64-unknown-linux-gnu

    # Windows (需要 mingw 交叉编译器)
    rustup target add x86_64-pc-windows-gnu

    info "Rust targets 安装完成。"
    echo ""
    warn "交叉编译还需要安装以下系统工具链："
    echo ""
    if [[ "$HOST_OS" == "Darwin" ]]; then
        echo "  Linux 交叉编译 (在 macOS 上):"
        echo "    brew tap messense/macos-cross-toolchains"
        echo "    brew install x86_64-unknown-linux-gnu"
        echo "    brew install aarch64-unknown-linux-gnu"
        echo ""
        echo "  Windows 交叉编译 (在 macOS 上):"
        echo "    brew install mingw-w64"
    else
        echo "  Windows 交叉编译 (在 Linux 上):"
        echo "    sudo apt install mingw-w64        # Debian/Ubuntu"
        echo "    sudo dnf install mingw64-gcc      # Fedora/RHEL"
        echo ""
        echo "  macOS 交叉编译需在 macOS 上执行（依赖 lipo）"
    fi
    echo ""
}

# -------------------------------------------------------
# 通用编译函数
# -------------------------------------------------------
build_target() {
    local target="$1"
    local profile="${2:-release}"

    info "编译 target: $target ..."
    cargo build --release --target "$target"
    info "编译完成: $target"
}

# -------------------------------------------------------
# macOS: arm64 + x86_64 -> 通用二进制 (Universal Binary)
# -------------------------------------------------------
do_build_mac() {
    if [[ "$HOST_OS" != "Darwin" ]]; then
        error "macOS Universal Binary 需要 lipo 工具，只能在 macOS 上编译。当前系统: $HOST_OS"
    fi

    local out_dir="$DIST_DIR/mac"
    mkdir -p "$out_dir"

    info "===== 编译 macOS ====="

    # 编译 arm64 (Apple Silicon)
    build_target "aarch64-apple-darwin"

    # 编译 x86_64 (Intel)
    build_target "x86_64-apple-darwin"

    # 用 lipo 合并为 Universal Binary（动态库）
    info "合并为 Universal Binary (dylib)..."
    lipo -create \
        "target/aarch64-apple-darwin/release/lib${LIB_NAME}.dylib" \
        "target/x86_64-apple-darwin/release/lib${LIB_NAME}.dylib" \
        -output "$out_dir/lib${LIB_NAME}.dylib"

    # 用 lipo 合并为 Universal Binary（静态库）
    info "合并为 Universal Binary (staticlib)..."
    lipo -create \
        "target/aarch64-apple-darwin/release/lib${LIB_NAME}.a" \
        "target/x86_64-apple-darwin/release/lib${LIB_NAME}.a" \
        -output "$out_dir/lib${LIB_NAME}.a"

    # 复制头文件
    cp "$HEADER_NAME" "$out_dir/" 2>/dev/null || warn "头文件 $HEADER_NAME 未找到，请先运行 cargo build 生成"

    info "macOS 构建完成 -> $out_dir/"
    ls -lh "$out_dir/"
    echo ""
}

# -------------------------------------------------------
# Linux: x86_64 静态库 + 动态库
# -------------------------------------------------------
do_build_linux() {
    local out_dir="$DIST_DIR/linux-x86_64"
    mkdir -p "$out_dir"

    info "===== 编译 Linux (x86_64) ====="

    # 判断是否需要交叉编译
    if [[ "$HOST_OS" == "Linux" && ( "$HOST_ARCH" == "x86_64" || "$HOST_ARCH" == "amd64" ) ]]; then
        # 本机就是 Linux x86_64，直接原生编译，无需交叉编译器
        info "检测到本机 Linux x86_64，使用原生编译..."
    else
        # 需要交叉编译（macOS 上或其他架构）
        info "检测到非本机编译，配置交叉编译器..."
        if command -v x86_64-unknown-linux-gnu-gcc &>/dev/null; then
            export CC_x86_64_unknown_linux_gnu=x86_64-unknown-linux-gnu-gcc
            export AR_x86_64_unknown_linux_gnu=x86_64-unknown-linux-gnu-ar
            export CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_LINKER=x86_64-unknown-linux-gnu-gcc
        elif command -v x86_64-linux-gnu-gcc &>/dev/null; then
            # Debian/Ubuntu 交叉编译器命名方式
            export CC_x86_64_unknown_linux_gnu=x86_64-linux-gnu-gcc
            export AR_x86_64_unknown_linux_gnu=x86_64-linux-gnu-ar
            export CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_LINKER=x86_64-linux-gnu-gcc
        else
            if [[ "$HOST_OS" == "Darwin" ]]; then
                error "找不到 Linux 交叉编译器。请运行:\n  brew tap messense/macos-cross-toolchains\n  brew install x86_64-unknown-linux-gnu"
            else
                error "找不到 Linux 交叉编译器。请运行:\n  sudo apt install gcc-x86-64-linux-gnu   # Debian/Ubuntu\n  sudo dnf install gcc-x86_64-linux-gnu   # Fedora/RHEL"
            fi
        fi
    fi

    build_target "x86_64-unknown-linux-gnu"

    cp "target/x86_64-unknown-linux-gnu/release/lib${LIB_NAME}.so" "$out_dir/"
    cp "target/x86_64-unknown-linux-gnu/release/lib${LIB_NAME}.a"  "$out_dir/"
    cp "$HEADER_NAME" "$out_dir/" 2>/dev/null || true

    info "Linux (x86_64) 构建完成 -> $out_dir/"
    ls -lh "$out_dir/"
    echo ""
}

# -------------------------------------------------------
# Windows: x86_64 (使用 mingw-w64 交叉编译)
# -------------------------------------------------------
do_build_windows() {
    local out_dir="$DIST_DIR/windows-x86_64"
    mkdir -p "$out_dir"

    info "===== 编译 Windows (x86_64) ====="

    # 检查 mingw-w64 是否安装
    if ! command -v x86_64-w64-mingw32-gcc &>/dev/null; then
        if [[ "$HOST_OS" == "Darwin" ]]; then
            error "找不到 mingw-w64 交叉编译器。请运行: brew install mingw-w64"
        else
            error "找不到 mingw-w64 交叉编译器。请运行:\n  sudo apt install mingw-w64    # Debian/Ubuntu\n  sudo dnf install mingw64-gcc  # Fedora/RHEL"
        fi
    fi

    export CC_x86_64_pc_windows_gnu=x86_64-w64-mingw32-gcc
    export AR_x86_64_pc_windows_gnu=x86_64-w64-mingw32-ar
    export CARGO_TARGET_X86_64_PC_WINDOWS_GNU_LINKER=x86_64-w64-mingw32-gcc

    build_target "x86_64-pc-windows-gnu"

    cp "target/x86_64-pc-windows-gnu/release/${LIB_NAME}.dll"  "$out_dir/" 2>/dev/null || true
    cp "target/x86_64-pc-windows-gnu/release/lib${LIB_NAME}.a" "$out_dir/" 2>/dev/null || true
    # Windows 有时生成 .dll.a 导入库
    cp "target/x86_64-pc-windows-gnu/release/${LIB_NAME}.dll.a" "$out_dir/" 2>/dev/null || true
    cp "$HEADER_NAME" "$out_dir/" 2>/dev/null || true

    info "Windows (x86_64) 构建完成 -> $out_dir/"
    ls -lh "$out_dir/"
    echo ""
}

# -------------------------------------------------------
# 编译所有平台
# -------------------------------------------------------
do_build_all() {
    if [[ "$HOST_OS" == "Darwin" ]]; then
        do_build_mac
    else
        warn "跳过 macOS 编译（需要在 macOS 上运行，依赖 lipo）"
    fi
    do_build_linux
    do_build_windows

    info "===== 全部完成 ====="
    echo ""
    echo "输出目录结构:"
    echo "  dist/"
    echo "  ├── mac/"
    echo "  │   ├── lib${LIB_NAME}.dylib   (macOS 动态库, Universal Binary)"
    echo "  │   ├── lib${LIB_NAME}.a        (macOS 静态库, Universal Binary)"
    echo "  │   └── ${HEADER_NAME}"
    echo "  ├── linux-x86_64/"
    echo "  │   ├── lib${LIB_NAME}.so       (Linux 动态库)"
    echo "  │   ├── lib${LIB_NAME}.a        (Linux 静态库)"
    echo "  │   └── ${HEADER_NAME}"
    echo "  └── windows-x86_64/"
    echo "      ├── ${LIB_NAME}.dll         (Windows 动态库)"
    echo "      ├── lib${LIB_NAME}.a        (Windows 静态库)"
    echo "      └── ${HEADER_NAME}"
}

# -------------------------------------------------------
# 入口
# -------------------------------------------------------
case "${1:-all}" in
    setup)   do_setup ;;
    mac)     do_build_mac ;;
    linux)   do_build_linux ;;
    windows) do_build_windows ;;
    all)     do_build_all ;;
    *)
        echo "用法: $0 {setup|mac|linux|windows|all}"
        exit 1
        ;;
esac
