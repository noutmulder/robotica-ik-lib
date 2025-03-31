# Maak de library van alle .cpp bestanden
file(GLOB LIB_SOURCES src/*.cpp)
file(GLOB LIB_HEADERS include/ik_solver/*.hpp)

# Maak de ik_solver library van de .cpp bestanden
add_library(ik_solver ${LIB_SOURCES})

# Zorg dat de headers beschikbaar zijn voor projecten die deze library gebruiken
target_include_directories(ik_solver PUBLIC include)
