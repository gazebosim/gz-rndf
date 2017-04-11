include (${project_cmake_dir}/Utils.cmake)

set (sources
  Checkpoint.cc
  Exit.cc
  Lane.cc
  ParkingSpot.cc
  ParserUtils.cc
  Perimeter.cc
  RNDF.cc
  RNDFNode.cc
  Segment.cc
  UniqueId.cc
  Waypoint.cc
  Zone.cc
)

set (gtest_sources
  Checkpoint_TEST.cc
  Exit_TEST.cc
  Lane_TEST.cc
  ParkingSpot_TEST.cc
  ParserUtils_TEST.cc
  Perimeter_TEST.cc
  RNDF_TEST.cc
  RNDFNode_TEST.cc
  Segment_TEST.cc
  UniqueId_TEST.cc
  Waypoint_TEST.cc
  Zone_TEST.cc
)

include_directories(${CMAKE_BINARY_DIR}/ ${CMAKE_BINARY_DIR}/test/)

ign_build_tests(${gtest_sources})

if (UNIX)
  ign_add_library(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION} ${sources})
elseif(WIN32)
  add_library(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION} STATIC ${sources})
endif()

target_link_libraries(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION}
  ${IGNITION-MATH_LIBRARIES})

if(WIN32)
  target_link_libraries(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION}
    ws2_32 Iphlpapi
  )
else()
  target_link_libraries(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION}
  )
endif()

ign_install_library(${PROJECT_NAME_LOWER}${PROJECT_MAJOR_VERSION})

