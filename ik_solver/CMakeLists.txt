# ik_solver/CMakeLists.txt

add_library(ik_solver
    src/IKSolver.cpp
    src/InverseKinematics.cpp
)

target_include_directories(ik_solver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
