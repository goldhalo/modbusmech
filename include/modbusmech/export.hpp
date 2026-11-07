// =============================================================================
//  modbusmech - 共享库导出宏
// =============================================================================
//  构建时加 `-DBUILD_SHARED_LIBS=ON` 即可产出动态库；CMake 会自动：
//    · 给库本身定义 `MODBUSMECH_BUILDING_LIBRARY`
//    · 给使用方定义 `MODBUSMECH_SHARED`（Windows 下切换 dllimport）
//  静态库下宏展开为空，不引入 ABI/性能负担。
// =============================================================================
#pragma once

#if defined(_WIN32) && defined(MODBUSMECH_SHARED)
#  if defined(MODBUSMECH_BUILDING_LIBRARY)
#    define MODBUSMECH_API __declspec(dllexport)
#  else
#    define MODBUSMECH_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__) && defined(MODBUSMECH_SHARED)
// Linux/macOS：配合 CXX_VISIBILITY_PRESET=hidden 收窄导出面。
#  define MODBUSMECH_API __attribute__((visibility("default")))
#else
#  define MODBUSMECH_API
#endif
