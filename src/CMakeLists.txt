# Turn on C++11 support for the library
set(CMAKE_CXX_STANDARD 11)

# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable
ign_get_libsources_and_unittests(sources gtest_sources)

# Create the library target
ign_add_library(${PROJECT_LIBRARY_TARGET_NAME} ${sources})

# Link the libraries that we always need
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
  PRIVATE
    ${DL_TARGET})

# Include the interface directories that we always need.
ign_target_interface_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
  ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

# Create installation instructions for the library target. This must be called
# in the same scope that the target is created.
ign_install_library()

# Build the unit tests
ign_build_tests(
  TYPE UNIT
  SOURCES ${gtest_sources}
  INCLUDE_DIRS
    # Used to make internal source file headers visible to the unit tests
    ${CMAKE_CURRENT_SOURCE_DIR}
    # Used to make test-directory headers visible to the unit tests
    ${PROJECT_SOURCE_DIR}
    # Used to make test_config.h visible to the unit tests
    ${PROJECT_BINARY_DIR})
