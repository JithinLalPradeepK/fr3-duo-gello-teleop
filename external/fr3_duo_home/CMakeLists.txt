# Standalone build: needs only libfranka (no ROS, no GELLO, no sourcing).
#   mkdir build && cd build && cmake .. && make
# If libfranka lives elsewhere:  cmake .. -DLIBFRANKA_DIR=/path/to/libfranka/install
cmake_minimum_required(VERSION 3.10)
project(fr3_duo_home CXX)
set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Default: the libfranka you already built for the FR3s (must match the robots' system version).
set(LIBFRANKA_DIR "$ENV{HOME}/fr3_tools/libfranka-install" CACHE PATH "libfranka install prefix")
# libfranka itself depends on pinocchio, which on this machine is installed under /opt/ros/humble.
# Only the library is used; nothing from ROS runs.
list(APPEND CMAKE_PREFIX_PATH "${LIBFRANKA_DIR}" "/opt/ros/humble")

find_package(Franka REQUIRED)
find_package(Threads REQUIRED)
add_executable(fr3_duo_home fr3_duo_home.cpp)
target_link_libraries(fr3_duo_home Franka::Franka Threads::Threads)
# Bake library paths into the binary so it runs without `source` or LD_LIBRARY_PATH.
set_target_properties(fr3_duo_home PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
