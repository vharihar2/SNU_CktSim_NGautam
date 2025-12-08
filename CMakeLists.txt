cmake_minimum_required(VERSION 3.24) # CMake version check

project(SNU_Spice) # Create project "SNU_Spice"

set(CMAKE_CXX_STANDARD 17) # Enable c++14 standard
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Export compile commands for clangd
set(CMAKE_CXX_STANDARD_REQUIRED ON) # googletest requires atleast c++14

# Include Ctest module
include(CTest)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/0953a17a4281fc26831da647ad3fcd5e21e6473b.zip
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
# set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Doxygen Integration
find_package(Doxygen REQUIRED dot OPTIONAL_COMPONENTS mscgen dia)

if(DOXYGEN_FOUND)
  set(DOXYGEN_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/docs)
  set(DOXYGEN_LAYOUT_FILE ${PROJECT_SOURCE_DIR}/DoxygenLayout.xml)

  # Here we must make Doxygen do not generate the docs for dependency See
  # https://cmake.org/cmake/help/latest/module/FindDoxygen.html#variable:DOXYGEN_EXCLUDE_PATTERNS
  # Exclude external, build and tests directories from documentation
  set(DOXYGEN_EXCLUDE_PATTERNS */external/* */build/* */tests/*)

  # Treeview and appearence of the documentation pages
  set(DOXYGEN_DISABLE_INDEX YES)
  set(DOXYGEN_GENERATE_TREEVIEW YES)
  set(DOXYGEN_FULL_SIDEBAR YES)

  # Disable pdf documentation generation
  set(DOXYGEN_GENERATE_LATEX NO)
  set(DOXYGEN_JAVADOC_AUTOBRIEF YES)

  # Dot graph options
  set(DOXYGEN_HAVE_DOT YES)
  set(DOXYGEN_CLASS_DIAGRAMS YES)
  set(DOXYGEN_COLLABORATION_GRAPH YES)
  set(DOXYGEN_UML_LOOK YES)
  set(DOXYGEN_HIDE_UNDOC_RELATIONS NO)
  set(DOXYGEN_CLASS_GRAPH YES)
  set(DOXYGEN_CALL_GRAPH YES)
  set(DOXYGEN_CALLER_GRAPH YES)
  set(DOXYGEN_COLLABORATION_GRAPH YES)
  set(DOXYGEN_BUILTIN_STL_SUPPORT YES)
  set(DOXYGEN_TEMPLATE_RELATIONS YES)
  set(DOXYGEN_DOT_TRANSPARENT YES)
  set(DOXYGEN_DOT_GRAPH_MAX_NODES 100)

  # Disable to generate warnings for undocumented functions
  set(DOXYGEN_EXTRACT_ALL YES)
  set(DOXYGEN_EXTRACT_PRIVATE YES)
  set(DOXYGEN_EXTRACT_PACKAGE YES)
  set(DOXYGEN_EXTRACT_STATIC YES)
  set(DOXYGEN_EXTRACT_LOCALMETHODS YES)

  # Add a target "documentation" to generate documentation with Doxygen
  doxygen_add_docs(documentation ${PROJECT_SOURCE_DIR})
else()
  message("Doxygen need to be installed to generate the doxygen documentation")
endif()

add_subdirectory(./src) # Adding the source code subdir
add_subdirectory(./tests) # Adding the tests subdir
