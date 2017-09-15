cmake_minimum_required(VERSION 2.8)
project(djiosdk-linux-sample)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

if (COVERAGE)
  set(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage -fPIC")
  set(GCC_COVERAGE_LINK_FLAGS    "-lgcov")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")
endif ()

include_directories(../../../osdk-core/inc)
include_directories(../../../osdk-wrapper/inc)
include_directories(../../../platform/linux/inc)
include_directories(inc)
if (LIDAR_LOGGING)
  include_directories(../../../sensor-integration/velodyne_vlp16puck_lite_API_example/vlp16lidar-driver/inc)
endif ()

if (USE_PRECISION_MISSIONS)
  include_directories("${precision_missions_source_dir}/inc")

  # Local Mission Planning libraries
  include_directories("${precision_missions_source_dir}/dependencies")
  include_directories("${precision_missions_source_dir}/dependencies/csv")
endif()

FILE(GLOB SOURCE_FILES inc/*.h src/*.cpp)
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

add_dependencies(${PROJECT_NAME} djiosdk-wrapper)

set(BLOCKING_LINK_LIBRARIES djiosdk-core djiosdk-platform djiosdk-wrapper)

if(LIDAR_LOGGING)
  set(BLOCKING_LINK_LIBRARIES ${BLOCKING_LINK_LIBRARIES} vlp16lidar-driver)
  #target_link_libraries(${PROJECT_NAME} vlp16lidar-driver djiosdk-core djiosdk-platform djiosdk-wrapper ${MISSIONPLAN})
endif()

target_link_libraries(${PROJECT_NAME} ${BLOCKING_LINK_LIBRARIES})

# Shared library case: All we need to do is link against the library, and
# anything else we need (in this case, some Boost libraries):


# Shared library case: All we need to do is link against the library, and
# anything else we need (in this case, some Boost libraries):
if(NOT UHD_USE_STATIC_LIBS)
    message(STATUS "Linking against shared UHD library.")
    target_link_libraries(${PROJECT_NAME} ${UHD_LIBRARIES} ${Boost_LIBRARIES})
# Shared library case: All we need to do is link against the library, and
# anything else we need (in this case, some Boost libraries):
else(NOT UHD_USE_STATIC_LIBS)
    message(STATUS "Linking against static UHD library.")
    target_link_libraries(${PROJECT_NAME}
        # We could use ${UHD_LIBRARIES}, but linking requires some extra flags,
        # so we use this convenience variable provided to us
        ${UHD_STATIC_LIB_LINK_FLAG}
        # Also, when linking statically, we need to pull in all the deps for
        # UHD as well, because the dependencies don't get resolved automatically
        ${UHD_STATIC_LIB_DEPS}
    )
endif(NOT UHD_USE_STATIC_LIBS)

### Once it's built... ########################################################
# Here, you would have commands to install your program.
# We will skip these in this example.
