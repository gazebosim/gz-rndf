cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find the Ignition RNDF library
find_package(ignition-rndf0 QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-RNDF_CXX_FLAGS}")
include_directories(${IGNITION-RNDF_INCLUDE_DIRS})
link_directories(${IGNITION-RNDF_LIBRARY_DIRS})

# Generate examples
add_executable(rndf_info rndfInfo.cc)
target_link_libraries(rndf_info ${IGNITION-RNDF_LIBRARIES})

if (MSVC)

  # Don't pull in the Windows min/max macros
  add_definitions(-DNOMINMAX)

endif()
