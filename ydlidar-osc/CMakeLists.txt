project(ydlidar-osc)

set(SOURCES
  src/main.cpp
  src/lidar.cpp
  src/osc.cpp
  src/utils.cpp
)

add_executable(ydlidar-osc ${SOURCES})

include_directories(${CMAKE_SOURCE_DIR}/include)
