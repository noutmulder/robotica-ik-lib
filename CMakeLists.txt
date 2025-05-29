cmake_minimum_required(VERSION 3.10)
project(IKProject)

set(CMAKE_CXX_STANDARD 17)

# --------------------------------------
# Zoek naar Eigen3
# --------------------------------------
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

if (NOT Eigen3_FOUND)
    message(FATAL_ERROR "Eigen3 is not found. Please install it with your package manager (e.g. sudo apt install libeigen3-dev)")
endif()

# --------------------------------------
# Voeg de subdirectories toe
# --------------------------------------
add_subdirectory(ik_solver)
add_subdirectory(robot_arm)

# --------------------------------------
# Main executable
# --------------------------------------
add_executable(ik_project src/main.cpp)

# Link de benodigde libraries
target_link_libraries(ik_project PRIVATE ik_solver robot_arm Eigen3::Eigen)

# Include directories
target_include_directories(ik_project PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/ik_solver/include
    ${CMAKE_CURRENT_SOURCE_DIR}/robot_arm/include
)

# --------------------------------------
# Executable voor FK test
# --------------------------------------
add_executable(main_fk_tester src/main_fk_tester.cpp)

target_link_libraries(main_fk_tester PRIVATE ik_solver robot_arm Eigen3::Eigen)

target_include_directories(main_fk_tester PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/ik_solver/include
    ${CMAKE_CURRENT_SOURCE_DIR}/robot_arm/include
)
