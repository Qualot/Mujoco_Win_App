cmake_minimum_required(VERSION 3.15)
project(Mujoco_Win_App CXX)

set(CMAKE_CXX_STANDARD 17)

# ========= MuJoCo =========
set(MUJOCO_ROOT "/path/to/mujoco")  # Update this path to your MuJoCo installation
include_directories(${MUJOCO_ROOT}/include)
link_directories(${MUJOCO_ROOT}/lib)

# ========= GLFW (via vcpkg) =========
find_package(glfw3 CONFIG REQUIRED)

# ========= nlohmann/json =========
#find_package(nlohmann_json CONFIG REQUIRED)

# Viewer library
add_library(Viewer viewer.cc viewer.h)
target_include_directories(Viewer PRIVATE ${MUJOCO_ROOT}/include)
target_link_libraries(Viewer PUBLIC mujoco glfw)

# UDP libraries
add_library(UDPSender udp_sender.cc udp_sender.h)
add_library(UDPReceiver udp_receiver.cc udp_receiver.h)

# SimulationManager library
add_library(SimulationManager simulation_manager.cc simulation_manager.h)
target_include_directories(SimulationManager PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(SimulationManager PUBLIC Viewer UDPSender UDPReceiver)

# Main executable
add_executable(Mujoco_Win_App main.cc)
target_link_libraries(Mujoco_Win_App PRIVATE SimulationManager)
