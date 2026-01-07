cmake_minimum_required(VERSION 3.0.0)
project(HYYRobotMain VERSION 10.0.0)

include(CTest)
enable_testing()

# ===================== Performance / Build-type settings (minimal change) =====================

# 如果你用的是单配置生成器（Make/Ninja），未指定时给一个更合理的默认类型
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# 用标准方式声明 C++11（替代 -std=c++11 拼 flags）
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 统一 warnings（对所有 target 生效）
add_compile_options(-Wall)

# 运行性能关键：按构建类型给优化级别与 NDEBUG
set(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g3")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O2 -g -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_MINSIZEREL     "${CMAKE_CXX_FLAGS_MINSIZEREL} -Os -DNDEBUG")

# 可选：裁剪无用代码/数据段（更小体积；常改善启动/内存，有时也改善 cache）
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE} -ffunction-sections -fdata-sections")
set(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO} -Wl,--gc-sections -Wl,--as-needed")
set(CMAKE_SHARED_LINKER_FLAGS_RELEASE        "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} -Wl,--gc-sections -Wl,--as-needed")
set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO    "${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO} -Wl,--gc-sections -Wl,--as-needed")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE           "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -Wl,--gc-sections -Wl,--as-needed")

# 可选：LTO（跨目标链接时整体优化；通常对性能收益可观）
option(HYY_ENABLE_LTO "Enable -flto for Release/RelWithDebInfo" OFF)
if(HYY_ENABLE_LTO)
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -flto")
    set(CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE} -flto")
    set(CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO "${CMAKE_SHARED_LINKER_FLAGS_RELWITHDEBINFO} -flto")
    set(CMAKE_SHARED_LINKER_FLAGS_RELEASE        "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} -flto")
    set(CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO    "${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO} -flto")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE           "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -flto")
endif()

# 可选：CPU 指令集本机优化（仅当部署机器 CPU 型号一致时开启）
option(HYY_ENABLE_NATIVE "Enable -march=native/-mtune=native for Release/RelWithDebInfo" OFF)
if(HYY_ENABLE_NATIVE)
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -march=native -mtune=native")
    set(CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native")
endif()

# ==============================================================================================

include_directories(
    include
    include/user
    include/DeviceDriver
)

aux_source_directory(src DIR_SRCS)

link_directories(
        lib
)

list(REMOVE_ITEM DIR_SRCS
    "src/PluginTemplate.cpp"
    "src/PokaePlugin.cpp"
    "src/SUPCONPlugin.cpp"
    "src/SubwayPlugin.cpp"
    "src/JEServer.cpp"
    "src/JETest.cpp"
)

# add_executable(${PROJECT_NAME} ${DIR_SRCS})
# target_link_libraries(${PROJECT_NAME} RobotControl modbus nlopt HYYRobotTechnology m pthread rt dl xml2)

# add_library(PluginTemplate SHARED src/PluginTemplate.cpp)
# target_link_libraries(PluginTemplate m pthread rt dl xml2)

# add_library(RokaePlugin SHARED src/RokaePlugin.cpp)
# target_link_libraries(RokaePlugin RobotControl m pthread rt dl xml2)

# add_library(SUPCONPlugin SHARED src/SUPCONPlugin.cpp src/inspire.cpp )
# target_link_libraries(SUPCONPlugin RobotControl m pthread rt dl xml2)

# add_library(JSMovePlugin SHARED src/JSMovePlugin.cpp src/inspire.cpp )
# target_link_libraries(JSMovePlugin RobotControl m pthread rt dl xml2)

# add_library(SubwayPlugin SHARED src/SubwayPlugin.cpp)
# target_link_libraries(SubwayPlugin RobotControl m pthread rt dl xml2)

# add_library(ExternalDemo SHARED src/ExternalPlugin.cpp)
# target_link_libraries(ExternalDemo RobotControl m pthread rt dl xml2)

add_library(JEServer SHARED src/JEServer.cpp)
target_link_libraries(JEServer RobotControl m pthread zmq rt dl xml2)

# add_executable(JETest src/JETest.cpp)
# target_link_libraries(JETest zmq rt dl xml2)

# add_executable(JETestV2 src/JETestV2.cpp)
# target_link_libraries(JETestV2 zmq rt dl xml2)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)