#!/bin/sh
# =============================================================================
#  publish_linux.sh —— 编译 modbusmech(+netmech) 静态库并发布到 /usr/3rdlibsIes
# =============================================================================
#  用法：  sh publish_linux.sh [build|debug]
#            build/debug 仅影响是否带调试信息（当前只发 Release 一套 .a）
#
#  产出：  /usr/3rdlibsIes/3rdlibs/include/modbusmech/*.hpp
#          /usr/3rdlibsIes/3rdlibs/include/netmech/*.hpp
#          /usr/3rdlibsIes/3rdlibs/lib/libmodbusmech.a
#          /usr/3rdlibsIes/3rdlibs/lib/libNetMechanism.a
#
#  注意：
#    * netmech 用 thirdparty/netmech 内嵌副本 —— E:/git/netmech 是"去串口"后的
#      新架构（没有 netmech/serial.hpp），与本库不兼容。
#    * .a 用 g++ 编，与板子交叉编译链不一致；板上要用时需用交叉链重编。
# =============================================================================
set -e

ROOT=$(cd "$(dirname "$0")" && pwd)
NMR="$ROOT/thirdparty/netmech"
BUILD="$ROOT/build/publish_linux"
DST=/usr/3rdlibsIes/3rdlibs

echo "===== [1/3] configure ====="
cmake -S "$ROOT" -B "$BUILD" \
      -DMODBUSMECH_NETMECH_DIR="$NMR" \
      -DMODBUSMECH_BUILD_TESTS=OFF \
      -DCMAKE_BUILD_TYPE=Release

echo "===== [2/3] build ====="
cmake --build "$BUILD" -j"$(nproc)"

echo "===== [3/3] publish to $DST ====="
mkdir -p "$DST/include/modbusmech" "$DST/include/netmech" "$DST/lib"

cp -f "$ROOT"/include/modbusmech/*.hpp "$DST/include/modbusmech/"
cp -f "$NMR"/include/netmech/*.hpp      "$DST/include/netmech/"

MM_A=$(find "$BUILD" -name 'libmodbusmech.a' | head -1)
NM_A=$(find "$BUILD" -name 'libNetMechanism.a' | head -1)
[ -n "$MM_A" ] && [ -n "$NM_A" ] || { echo "FAILED: .a not found"; exit 1; }
cp -f "$MM_A" "$DST/lib/libmodbusmech.a"
cp -f "$NM_A" "$DST/lib/libNetMechanism.a"

echo
echo "DONE."
ls -la "$DST/lib/libmodbusmech.a" "$DST/lib/libNetMechanism.a"
ls "$DST/include/modbusmech/"
