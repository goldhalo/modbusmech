cmake_minimum_required(VERSION 3.16)
project(modbusmech VERSION 0.9.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# 安装目录规范。必须早于 target 定义：$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
# 里的变量是立即展开的，晚定义会展开成空串。
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 静态库统一放 build/lib —— 与 netmech / rtspmech / rtpmech / mpegmech 一致。
# 之前本仓的 .a 落在构建目录根上、netmech 的落在 build/lib/，写链接命令时
# 得在两个目录之间来回切，很容易漏。
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 默认**优先用 git submodule**（thirdparty/netmech），没有再退回同级目录。
#
# 原先只认 ../netmech：本地两个仓并排时能用，但 CI 里 submodules: recursive
# 把 netmech 检出到 thirdparty/netmech，配置阶段就会报"找不到 netmech"。
# 两种布局都照顾到之后，本地和 CI 都不需要额外传参。
set(MODBUSMECH_NETMECH_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/netmech"
    CACHE PATH "netmech 所在目录（默认使用 git submodule）")
if(NOT EXISTS "${MODBUSMECH_NETMECH_DIR}/CMakeLists.txt"
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../netmech/CMakeLists.txt")
    set(MODBUSMECH_NETMECH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../netmech")
endif()

option(MODBUSMECH_BUILD_TESTS "构建单元测试" ON)

# 位置无关代码。
#
# 本库只出静态库，但**静态库经常被链进 .so**（公司项目里 modbusmech + netmech
# 一起链进 moc 的 .so）。静态库的 .o 若不含 -fPIC，链接共享库时会报：
#     relocation R_X86_64_PC32 against symbol `__libc_single_threaded'
#     can not be used when making a shared object; recompile with -fPIC
# 而且这个错误只在**最终链 .so 时**才出现，编静态库、跑单测都看不出来。
#
# 所以默认开。纯静态链接、且在意那一点点 GOT 间接寻址开销的嵌入式场景，
# 可以 -DMODBUSMECH_PIC=OFF 关掉。
option(MODBUSMECH_PIC "编译为位置无关代码（静态库要链进 .so 时必须开）" ON)

add_library(modbusmech_compile_options INTERFACE)
# 多个选项用 ';' 分隔，不要靠空格 —— 空格写法只有 Makefile/Ninja
# "把选项拼成命令行"时才恰好正确，VS 生成器会原样塞进 AdditionalOptions。
target_compile_options(modbusmech_compile_options INTERFACE
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8;/W4;/permissive-;/Zc:__cplusplus>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall;-Wextra>)

option(MODBUSMECH_WARNINGS_AS_ERRORS "把编译告警当错误（CI 用）" OFF)
if(MODBUSMECH_WARNINGS_AS_ERRORS)
    target_compile_options(modbusmech_compile_options INTERFACE
        $<$<CXX_COMPILER_ID:MSVC>:/WX>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Werror>)
endif()

# netmech：串口 + TCP 的底层
if(NOT EXISTS "${MODBUSMECH_NETMECH_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "找不到 netmech：${MODBUSMECH_NETMECH_DIR}")
endif()
set(NETMECH_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(NETMECH_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
# 动态库构建要求 netmech 也以共享库提供：否则 netmech 的 io 运行时/线程池
# 会被静态链进本 DLL，与其它 mech 库各自的副本彼此独立（同进程多份状态）。
if(BUILD_SHARED_LIBS)
    set(NETMECH_LIBRARY_TYPE "SHARED" CACHE STRING "netmech 库类型" FORCE)
endif()
add_subdirectory("${MODBUSMECH_NETMECH_DIR}" "${CMAKE_BINARY_DIR}/netmech" EXCLUDE_FROM_ALL)

file(GLOB MODBUSMECH_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_library(modbusmech ${MODBUSMECH_SOURCES})
add_library(modbusmech::modbusmech ALIAS modbusmech)

# 共享库：导出/导入宏 + hidden 可见性（只导出标了 MODBUSMECH_API 的符号）
if(BUILD_SHARED_LIBS)
    target_compile_definitions(modbusmech PUBLIC MODBUSMECH_SHARED)
    target_compile_definitions(modbusmech PRIVATE MODBUSMECH_BUILDING_LIBRARY)
    set_target_properties(modbusmech PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN ON)
    if(MSVC)
        target_compile_options(modbusmech PRIVATE /wd4251)
    endif()
endif()

# ---------------------------------------------------------------------------
# 发布成动态库（-DBUILD_SHARED_LIBS=ON）时的符号卫生
#
# 共享形态下 netmech 被强制为 SHARED（见上方）：它是独立的动态库，
# 发布时需随附其二进制与 LICENSE。--exclude-libs 隐藏的是**静态**链入
# 本 .so 的依赖符号（避免污染 ABI / 被同进程其它模块的符号解析撞上）；
# 对共享形态的 netmech 不适用（它不在静态归档里）。
#
# Windows 走 MODBUSMECH_API（export.hpp）：库内 dllexport、使用方 dllimport，
# 使用方需要定义 MODBUSMECH_SHARED（CMake target 会传递）。
# ---------------------------------------------------------------------------
if(BUILD_SHARED_LIBS AND (UNIX AND NOT APPLE))
    target_link_options(modbusmech PRIVATE "-Wl,--exclude-libs,ALL")
elseif(BUILD_SHARED_LIBS AND APPLE)
    target_link_options(modbusmech PRIVATE "-Wl,-hidden-lsymbols")
endif()
set_target_properties(modbusmech PROPERTIES
    POSITION_INDEPENDENT_CODE ${MODBUSMECH_PIC})
target_include_directories(modbusmech PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(modbusmech
    PUBLIC  netmech
    PRIVATE modbusmech_compile_options)

if(MODBUSMECH_BUILD_TESTS)
    enable_testing()
    file(GLOB MODBUSMECH_TEST_SOURCES CONFIGURE_DEPENDS
         "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")
    if(MODBUSMECH_TEST_SOURCES)
        add_executable(modbusmech_ut ${MODBUSMECH_TEST_SOURCES})
        target_include_directories(modbusmech_ut PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/test"
            "${CMAKE_CURRENT_SOURCE_DIR}/src"
            "${MODBUSMECH_NETMECH_DIR}/test")
        target_link_libraries(modbusmech_ut PRIVATE modbusmech modbusmech_compile_options)
        add_test(NAME modbusmech_ut COMMAND modbusmech_ut)
    endif()
endif()

# ---------------------------------------------------------------------------
# 示例程序（主站 / 从站）
# ---------------------------------------------------------------------------
#  没有对接真实设备时，可以把两个示例互相当对端跑起来：
#     ./bin/modbusmech_slave  tcp 1502
#     ./bin/modbusmech_master tcp 127.0.0.1 1502 rh 0 10
#  也可以和第三方实现（libmodbus / mthings）互测，见 docs/interop.md。
option(MODBUSMECH_BUILD_EXAMPLES "构建示例程序（master / slave）" ON)
if(MODBUSMECH_BUILD_EXAMPLES)
    add_executable(modbusmech_master example/master_main.cpp)
    target_link_libraries(modbusmech_master
        PRIVATE modbusmech::modbusmech modbusmech_compile_options)

    add_executable(modbusmech_slave example/slave_main.cpp)
    target_link_libraries(modbusmech_slave
        PRIVATE modbusmech::modbusmech modbusmech_compile_options)
endif()

# ---------------------------------------------------------------------------
# 安装与导出
# ---------------------------------------------------------------------------
#  装完之后使用者只需要：
#      find_package(modbusmech REQUIRED)
#      target_link_libraries(app PRIVATE modbusmech::modbusmech)
#  netmech 会由 Config 里的 find_dependency 自动找出来，不用手写
#  libmodbusmech.a + libnetmech.a 那一串。
#
#  默认前缀：Linux -> /usr/local，Windows -> C:/Program Files/modbusmech
# ---------------------------------------------------------------------------
install(TARGETS modbusmech
        EXPORT modbusmechTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# 编译选项载体（PRIVATE 依赖在静态库场景下仍会进 interface，必须一起导出）
install(TARGETS modbusmech_compile_options EXPORT modbusmechTargets)

install(DIRECTORY include/modbusmech
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.hpp")

install(EXPORT modbusmechTargets
        FILE modbusmechTargets.cmake
        NAMESPACE modbusmech::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/modbusmech)

configure_package_config_file(cmake/modbusmechConfig.cmake.in
        "${CMAKE_CURRENT_BINARY_DIR}/modbusmechConfig.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/modbusmech)

write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/modbusmechConfigVersion.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMinorVersion)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/modbusmechConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/modbusmechConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/modbusmech)

install(FILES LICENSE NOTICE
        DESTINATION ${CMAKE_INSTALL_DATADIR}/doc/modbusmech)
