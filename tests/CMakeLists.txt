enable_testing()

# Discover all test source files under this tests directory (recursively)
file(GLOB_RECURSE TEST_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

# Build the test executable from all discovered test source files.
add_executable(
  tests
  ${TEST_SOURCES}
)

# Ensure the tests can include the project's public headers.
target_include_directories(
  tests
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

# Link test executable against GoogleTest and the project library so tests
# can exercise internal functionality without duplicating sources.
target_link_libraries(
  tests
  PRIVATE
    GTest::gtest_main
    SNU_Spice_lib
)

# (Optional) place test binary in a dedicated output dir to keep build tree tidy.
set_target_properties(tests PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

# Register tests with CTest using GoogleTest discovery.
include(GoogleTest)
gtest_discover_tests(tests)
