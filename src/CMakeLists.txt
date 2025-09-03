set(SOURCE_FILES main.cpp Node/Node.cpp Parser/Parser.cpp Solver/Solver.cpp)

add_executable(SNU_Spice ${SOURCE_FILES})
add_library(SNU_Spice_lib ${SOURCE_FILES})

# Add include directories 
target_include_directories(SNU_Spice PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(SNU_Spice_lib PUBLIC ${PROJECT_SOURCE_DIR}/include)
