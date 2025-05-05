cmake_minimum_required(VERSION 3.10)

# Create the static library for robot_arm
add_library(robot_arm STATIC
    src/RobotArm.cpp
    src/Vector3D.cpp
    src/Joint.cpp
    src/Link.cpp
)

# Make the include directories public
target_include_directories(robot_arm PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link against ik_solver since RobotArm uses InverseKinematics
target_link_libraries(robot_arm PRIVATE ik_solver)

# If you need any compiler definitions or options
target_compile_options(robot_arm PRIVATE -Wall -Wextra)

# If you need to export symbols (important for shared libraries on Windows)
set_target_properties(robot_arm PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)