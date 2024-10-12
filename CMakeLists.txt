cmake_minimum_required(VERSION 3.10)
project(art-project)

set(CMAKE_CXX_STANDARD 14)

# Enable CMP0079 policy
cmake_policy(SET CMP0079 NEW)

# Add subdirectory for ydlidar-touch
add_subdirectory(ydlidar-touch)

# Link oscpack as a static library
add_library(oscpack STATIC IMPORTED)
set_target_properties(oscpack PROPERTIES
    IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/lib/oscpack/lib/oscpack.lib
    INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_SOURCE_DIR}/lib/oscpack/include
)

# Link oscpack to ydlidar-touch
target_link_libraries(ydlidar-touch oscpack)
