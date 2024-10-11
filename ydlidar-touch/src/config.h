#pragma once

#define DEFAULT_DEV_MODE false
#define DEFAULT_MIN_ANGLE_DEGREE -180.0f
#define DEFAULT_MAX_ANGLE_DEGREE 180.0f
#define DEFAULT_TOUCH_MIN_RANGE 0.1f
#define DEFAULT_TOUCH_MAX_RANGE 0.5f
#define DEFAULT_CLUSTER_MAX_DISTANCE 0.05f
#define DEFAULT_MIN_CLUSTER_SIZE 3
#define DEFAULT_IP_ADDRESS "127.0.0.1"
#define DEFAULT_OSC_PORT 7000

#define OUTPUT_BUFFER_SIZE 8192
#define OSC_ADDRESS "/lidar/touch"
#define dev_printf(...) if(isDevelopmentMode) { printf(__VA_ARGS__); }

extern bool isDevelopmentMode;
extern float minAngleDegree;
extern float maxAngleDegree;
