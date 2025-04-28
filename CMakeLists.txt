cmake_minimum_required(VERSION 3.10)
project(IKProject)

set(CMAKE_CXX_STANDARD 17)

# Add the ik_solver library as a subdirectory
add_subdirectory(ik_solver)
add_subdirectory(robot_arm)


# Main executable
add_executable(ik_project src/main.cpp)

# Link the ik_solver library to the main executable
target_link_libraries(ik_project PRIVATE ik_solver robot_arm)

# Include directories for the main executable
target_include_directories(ik_project PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ik_solver/include)
