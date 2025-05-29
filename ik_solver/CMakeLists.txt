cmake_minimum_required(VERSION 3.10)

# Vind en link Eigen3 (verplicht)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Maak de statische library voor ik_solver aan
add_library(ik_solver STATIC
    src/IKSolver.cpp
    src/InverseKinematics.cpp
)

# Public include directories, zodat andere targets de headers van deze library kunnen gebruiken
target_include_directories(ik_solver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}  # Voeg de Eigen-headers toe aan de public interface
)

# Link tegen Eigen
target_link_libraries(ik_solver PRIVATE Eigen3::Eigen)
