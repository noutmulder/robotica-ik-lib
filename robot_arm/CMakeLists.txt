cmake_minimum_required(VERSION 3.10)

# Vind Eigen3 (verplicht)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Maak de statische library voor robot_arm aan
add_library(robot_arm STATIC
    src/RobotArm.cpp
    src/Vector3D.cpp
    src/Joint.cpp
    src/Link.cpp
)

# Public include directories zodat andere targets (zoals main) ze ook kunnen gebruiken
target_include_directories(robot_arm PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}  # Zorg dat headers Eigen beschikbaar zijn in publieke headers zoals RobotArm.hpp
)

# Link afhankelijkheden
target_link_libraries(robot_arm
    PRIVATE
        ik_solver
        Eigen3::Eigen
)

# Extra compiler flags
target_compile_options(robot_arm PRIVATE -Wall -Wextra)

# Zorg voor nette symbol export (belangrijk bij shared builds)
set_target_properties(robot_arm PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)
