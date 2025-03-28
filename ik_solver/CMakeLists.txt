# Maak de library van alle .cpp bestanden
file(GLOB LIB_SOURCES src/*.cpp)
file(GLOB LIB_HEADERS include/ik_solver/*.hpp)

add_library(ik_solver ${LIB_SOURCES})

# Zorg dat de headers beschikbaar zijn voor projecten die deze library gebruiken
target_include_directories(ik_solver PUBLIC include)
target_include_directories(ik_solver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

