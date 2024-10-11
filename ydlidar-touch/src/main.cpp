#include "config.h"
#include "lidar.h"
#include "osc.h"
#include "utils.h"
#include <iostream>
#include <core/base/timer.h>

bool isDevelopmentMode = DEFAULT_DEV_MODE;
float minAngleDegree = DEFAULT_MIN_ANGLE_DEGREE;
float maxAngleDegree = DEFAULT_MAX_ANGLE_DEGREE;

int main(int argc, char *argv[]) {
  
  /* 変数設定 */

  // 開発モードの設定
  setupParameter("Enable development mode? [yes/no]", isDevelopmentMode, DEFAULT_DEV_MODE, [](const std::string& input) {
    return input == "yes" || input == "y" || input == "no" || input == "n";
  });

  // 最小角度の設定
  setupParameter("Enter min angle in degrees", minAngleDegree, DEFAULT_MIN_ANGLE_DEGREE, [](const std::string& input) {
    float value = std::stof(input);
    return value >= -180.0f && value <= 0.0f;
  });

  // 最大角度の設定
  setupParameter("Enter max angle in degrees", maxAngleDegree, DEFAULT_MAX_ANGLE_DEGREE, [](const std::string& input) {
    float value = std::stof(input);
    return value >= 0.0f && value <= 180.0f;
  });

  // タッチ検出の最小距離の設定
  float touchMinRange = DEFAULT_TOUCH_MIN_RANGE;
  setupParameter("Enter touch detection min range in meters", touchMinRange, DEFAULT_TOUCH_MIN_RANGE);

  // タッチ検出の最大距離設定
  float touchMaxRange = DEFAULT_TOUCH_MAX_RANGE;
  setupParameter("Enter touch detection max range in meters", touchMaxRange, DEFAULT_TOUCH_MAX_RANGE);

  // クラスタリングの最大距離の設定
  float maxClusterDistance = DEFAULT_CLUSTER_MAX_DISTANCE;
  setupParameter("Enter max distance between points in a cluster", maxClusterDistance, DEFAULT_CLUSTER_MAX_DISTANCE);

  // クラスタの最小ポイント数設定
  int minClusterSize = DEFAULT_MIN_CLUSTER_SIZE;
  setupParameter("Enter minimum number of points in a cluster", minClusterSize, DEFAULT_MIN_CLUSTER_SIZE);

  // IPアドレスの設定
  std::string ipAddress = DEFAULT_IP_ADDRESS;
  setupParameter("Enter IP address", ipAddress, std::string(DEFAULT_IP_ADDRESS), isValidIpAddress);

  // OSCポート番号の設定
  int oscPort = DEFAULT_OSC_PORT;
  setupParameter("Enter OSC target port", oscPort, DEFAULT_OSC_PORT, isValidPort);

  /* OSC設定 */
  OSC osc;
  osc.setup(ipAddress, oscPort);

  /* LIDAR設定 */
  Lidar lidar;
  std::string port = lidar.selectPort();
  int baudrate = lidar.selectBaudrate();
  bool isSingleChannel = lidar.selectSingleChannel();
  float frequency = lidar.selectFrequency();

  if (port.empty() || baudrate == -1 || frequency == -1) {
    fprintf(stderr, "Invalid configuration detected. Exiting.\n");
    fflush(stderr);
    return -1;
  }

  /* 設定確認 */
  std::cout << "\n========== Settings ==========" << std::endl;
  std::cout << "Development Mode: " << (isDevelopmentMode ? "Enabled" : "Disabled") << std::endl;
  std::cout << "Min angle: " << minAngleDegree << std::endl;
  std::cout << "Max angle: " << maxAngleDegree << std::endl;
  std::cout << "Touch Min Range: " << touchMinRange << std::endl;
  std::cout << "Touch Max Range: " << touchMaxRange << std::endl;
  std::cout << "Max Cluster Distance: " << maxClusterDistance << std::endl;
  std::cout << "Min Cluster Size: " << minClusterSize << std::endl;
  std::cout << "OSC IP Address: " << ipAddress << std::endl;
  std::cout << "OSC Port: " << oscPort << std::endl;
  std::cout << "LIDAR Port: " << port << std::endl;
  std::cout << "Baudrate: " << baudrate << std::endl;
  std::cout << "One-way: " << (isSingleChannel ? "Yes" : "No") << std::endl;
  std::cout << "Frequency: " << frequency << std::endl;
  std::cout << "==============================\n" << std::endl;

  uint32_t t = getms();
  int c = 0;

  if (!lidar.initialize()) {
    fprintf(stderr, "Failed to initialize Lidar\n");
    fflush(stderr);
    return -1;
  }

  if (!lidar.start()) {
    fprintf(stderr, "Failed to start Lidar\n");
    fflush(stderr);
    return -1;
  }

  /* Scan設定 */
  LaserScan scan;

  while (ydlidar::os_isOk()) {
    if (lidar.doProcessSimple(scan)) {
      std::vector<std::pair<float, float>> touchPoints = getTouchPoints(scan, touchMinRange, touchMaxRange);
      std::vector<std::pair<float, float>> clusterCenters;
      performClustering(touchPoints, maxClusterDistance, minClusterSize, clusterCenters);

      if (!clusterCenters.empty()) osc.sendClusterData(clusterCenters);

      dev_printf("Clusters detected: %zu\n", clusterCenters.size());
    } else {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }
    if (!c++) {
      printf("Time consuming [%u] from initialization to parsing to point cloud data\n", getms() - t);
    }
  }

  lidar.stop();
  lidar.disconnect();

  return 0;
}
