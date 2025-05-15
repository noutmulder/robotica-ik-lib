cmake_minimum_required(VERSION 3.10)
project(IKProject)

set(CMAKE_CXX_STANDARD 17)

# Gebruik FetchContent om externe dependencies binnen te halen
include(FetchContent)

# Haal de Eigen3 bibliotheek op van GitLab
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0  # Specificeer de versie die je wilt gebruiken
)

# Maak Eigen3 beschikbaar voor het project
FetchContent_MakeAvailable(eigen)

# Voeg de subdirectories voor ik_solver en robot_arm toe
add_subdirectory(ik_solver)
add_subdirectory(robot_arm)

# Maak de hoofd-executable voor het project
add_executable(ik_project src/main.cpp)

# Koppel de benodigde libraries
target_link_libraries(ik_project PRIVATE ik_solver robot_arm Eigen3::Eigen)

# Voeg de include-paden toe voor de hoofd-executable
target_include_directories(ik_project PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/ik_solver/include
    ${CMAKE_CURRENT_SOURCE_DIR}/robot_arm/include  
    ${eigen_SOURCE_DIR}
)

# Optioneel: Extra executable voor het testen van Forward Kinematics
add_executable(main_fk_tester src/main_fk_tester.cpp)
target_link_libraries(main_fk_tester PRIVATE ik_solver robot_arm Eigen3::Eigen)
target_include_directories(main_fk_tester PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/ik_solver/include
    ${CMAKE_CURRENT_SOURCE_DIR}/robot_arm/include
    ${eigen_SOURCE_DIR}
)
