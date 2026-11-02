cmake_minimum_required(VERSION 3.16)
project(modbusmech VERSION 0.9.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(MODBUSMECH_NETMECH_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../netmech"
    CACHE PATH "netmech 所在目录")

option(MODBUSMECH_BUILD_TESTS "构建单元测试" ON)

add_library(modbusmech_compile_options INTERFACE)
target_compile_options(modbusmech_compile_options INTERFACE
    $<$<CXX_COMPILER_ID:MSVC>:/utf-8 /W4 /permissive- /Zc:__cplusplus>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra>)

# netmech：串口 + TCP 的底层
if(NOT EXISTS "${MODBUSMECH_NETMECH_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "找不到 netmech：${MODBUSMECH_NETMECH_DIR}")
endif()
set(NETMECH_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(NETMECH_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory("${MODBUSMECH_NETMECH_DIR}" "${CMAKE_BINARY_DIR}/netmech" EXCLUDE_FROM_ALL)

file(GLOB MODBUSMECH_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
add_library(modbusmech ${MODBUSMECH_SOURCES})
add_library(modbusmech::modbusmech ALIAS modbusmech)
target_include_directories(modbusmech PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
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
