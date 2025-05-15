# Create the static library for ik_solver
add_library(ik_solver STATIC
    src/IKSolver.cpp
    src/InverseKinematics.cpp
)

# Make the include directories public, so the main project can find them
target_include_directories(ik_solver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
