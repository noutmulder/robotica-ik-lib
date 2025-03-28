cmake_minimum_required(VERSION 3.10)
project(IKProject)

set(CMAKE_CXX_STANDARD 17)

# Voeg de librarymap toe
add_subdirectory(ik_solver)

# Main executable
add_executable(ik_project src/main.cpp)


# Koppel de library aan je main programma
target_link_libraries(ik_project PRIVATE ik_solver)
