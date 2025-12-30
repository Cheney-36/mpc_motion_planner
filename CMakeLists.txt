cmake_minimum_required(VERSION 3.13)
project(mpc_motion_planner_examples VERSION 0.1.0)

# 允许跨目录链接目标 CMP0079
cmake_policy(SET CMP0079 NEW)

set(CMAKE_BUILD_TYPE Release)

# 添加子模块 library
add_subdirectory(mpc_solver)

# backward compatibility for coal replacing hpp-fcl
set(COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL ON CACHE BOOL "Allow hpp-fcl compatibility")
find_package(coal QUIET)
if(TARGET coal::coal AND NOT TARGET hpp-fcl::hpp-fcl)
  add_library(hpp-fcl::hpp-fcl ALIAS coal::coal)
endif()

# 查找 ZeroMQ
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZeroMQ REQUIRED libzmq)

# 添加包含目录和库路径
include_directories(${ZeroMQ_INCLUDE_DIRS})
link_directories(${ZeroMQ_LIBRARY_DIRS})

find_package(Eigen3 REQUIRED)


# find header-only msgpack-c
find_path(MSGPACK_INCLUDE_DIR msgpack.hpp REQUIRED)
include_directories(${MSGPACK_INCLUDE_DIR})

# 主程序 target
add_executable(mpc_benchmark examples/benchmark.cpp)
add_executable(offline_trajectory examples/offline_trajectory.cpp)
add_executable(online_tracking examples/online_tracking.cpp)
add_executable(online_traj_gener examples/online_traj_gener.cpp)


add_executable(hw_zmq examples/hw_zmq.cpp)
add_executable(comm_zmq examples/comm_zmq.cpp)

# 链接 motionPlanner 库
target_link_libraries(mpc_benchmark PRIVATE motionPlanner)
target_link_libraries(offline_trajectory PRIVATE motionPlanner)

# link libraries
# remove msgpackc, link with ZeroMQ via pkg-config
target_link_libraries(online_tracking PUBLIC motionPlanner ${ZeroMQ_LIBRARIES})
target_link_libraries(online_traj_gener PUBLIC motionPlanner ${ZeroMQ_LIBRARIES})
target_link_libraries(hw_zmq PUBLIC motionPlanner ${ZeroMQ_LIBRARIES})
target_link_libraries(comm_zmq PUBLIC motionPlanner ${ZeroMQ_LIBRARIES})

# ZeroMQ 头文件目录导入到目标
target_include_directories(online_tracking PRIVATE ${ZeroMQ_INCLUDE_DIRS})
target_include_directories(online_traj_gener PRIVATE ${ZeroMQ_INCLUDE_DIRS})
target_link_libraries(hw_zmq PRIVATE ${ZeroMQ_INCLUDE_DIRS})
target_link_libraries(comm_zmq PRIVATE ${ZeroMQ_INCLUDE_DIRS})

