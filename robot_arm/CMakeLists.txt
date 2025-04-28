# robot_arm/CMakeLists.txt

add_library(robot_arm
    src/RobotArm.cpp
)

target_include_directories(robot_arm PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
