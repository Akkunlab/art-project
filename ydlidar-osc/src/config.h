#pragma once

#include <string>

#define DEFAULT_DEV_MODE false
#define DEFAULT_MIN_ANGLE_DEGREE -180.0f
#define DEFAULT_MAX_ANGLE_DEGREE 180.0f
#define DEFAULT_IP_ADDRESS "127.0.0.1"
#define DEFAULT_OSC_PORT 7000

#define OUTPUT_BUFFER_SIZE 8192
#define OSC_ADDRESS "/lidar/scan"
#define dev_printf(...) if(isDevelopmentMode) { printf(__VA_ARGS__); }

const std::string DEFAULT_COORD_MODE = "polar";

extern bool isDevelopmentMode;
extern float minAngleDegree;
extern float maxAngleDegree;
extern std::string coordMode;
