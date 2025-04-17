# Add all source files (including Vector3D.cpp) to the library
file(GLOB LIB_SOURCES src/*.cpp)  # This will include Vector3D.cpp if it's in src/
file(GLOB LIB_HEADERS include/ik_solver/*.hpp)

# Create the ik_solver library from the source files
add_library(ik_solver ${LIB_SOURCES})

# Ensure the include directory is set correctly for the library
target_include_directories(ik_solver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
