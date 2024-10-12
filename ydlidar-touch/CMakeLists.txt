project(ydlidar-touch)

set(SOURCES
  src/main.cpp
  src/lidar.cpp
  src/osc.cpp
  src/utils.cpp
)

add_executable(ydlidar-touch ${SOURCES})

include_directories(${CMAKE_SOURCE_DIR}/include)
