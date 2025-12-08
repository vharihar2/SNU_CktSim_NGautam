file(GLOB ELEMENTS_LINEAR_SRC "${CMAKE_CURRENT_SOURCE_DIR}/Elements/Linear/*.cpp")
# Nonlinear element sources live in Elements/Nonlinear (no underscore)
file(GLOB ELEMENTS_NONLINEAR_SRC "${CMAKE_CURRENT_SOURCE_DIR}/Elements/Nonlinear/*.cpp")

set(SOURCE_FILES
    main.cpp
    CircuitElement.cpp
    Node/Node.cpp
    Parser/Parser.cpp
    Solver/Solver.cpp
    ${ELEMENTS_LINEAR_SRC}
    ${ELEMENTS_NONLINEAR_SRC}
)

add_executable(SNU_Spice ${SOURCE_FILES})
add_library(SNU_Spice_lib ${SOURCE_FILES})

# Add include directories 
target_include_directories(SNU_Spice PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(SNU_Spice_lib PUBLIC ${PROJECT_SOURCE_DIR}/include)
