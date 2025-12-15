cmake_minimum_required(VERSION 3.0.0)
project(HYYRobotMain VERSION 10.0.0)

include(CTest)
enable_testing()

include_directories(
    include
    include/user
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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
