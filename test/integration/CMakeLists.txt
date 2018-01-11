# This is to make test_config.h visible
include_directories("${CMAKE_BINARY_DIR}")

ign_get_sources(tests)

ign_build_tests(
  TYPE INTEGRATION
  SOURCES ${tests})
