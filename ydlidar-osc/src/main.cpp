#include "config.h"
#include "lidar.h"
#include "osc.h"
#include "utils.h"
#include <string>
#include <iostream>
#include <core/base/timer.h>

bool isDevelopmentMode = DEFAULT_DEV_MODE;
float minAngleDegree = DEFAULT_MIN_ANGLE_DEGREE;
float maxAngleDegree = DEFAULT_MAX_ANGLE_DEGREE;
std::string coordMode = DEFAULT_COORD_MODE;

int main(int argc, char *argv[]) {
  
  /* �ϐ��ݒ� */

  // �J�����[�h�̐ݒ�
  setupParameter("Enable development mode? [yes/no]", isDevelopmentMode, DEFAULT_DEV_MODE, [](const std::string& input) {
    return input == "yes" || input == "y" || input == "no" || input == "n";
  });
  
  // �o�͍��W�̐ݒ�
  setupParameter("Select output coordinate system [polar/cartesian]", coordMode, DEFAULT_COORD_MODE, [](const std::string& input) {
    return input == "polar" || input == "cartesian";
  });

  // �ŏ��p�x�̐ݒ�
  setupParameter("Enter min angle in degrees", minAngleDegree, DEFAULT_MIN_ANGLE_DEGREE, [](const std::string& input) {
    float value = std::stof(input);
    return value >= -180.0f && value <= 0.0f;
  });

  // �ő�p�x�̐ݒ�
  setupParameter("Enter max angle in degrees", maxAngleDegree, DEFAULT_MAX_ANGLE_DEGREE, [](const std::string& input) {
    float value = std::stof(input);
    return value >= 0.0f && value <= 180.0f;
  });

  // IP�A�h���X�̐ݒ�
  std::string ipAddress = DEFAULT_IP_ADDRESS;
  setupParameter("Enter IP address", ipAddress, std::string(DEFAULT_IP_ADDRESS), isValidIpAddress);

  // OSC�|�[�g�ԍ��̐ݒ�
  int oscPort = DEFAULT_OSC_PORT;
  setupParameter("Enter OSC target port", oscPort, DEFAULT_OSC_PORT, isValidPort);

  /* OSC�ݒ� */
  OSC osc;
  osc.setup(ipAddress, oscPort);

  /* LIDAR�ݒ� */
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

  /* �ݒ�m�F */
  std::cout << "\n========== Settings ==========" << std::endl;
  std::cout << "Development Mode: " << (isDevelopmentMode ? "Enabled" : "Disabled") << std::endl;
  std::cout << "Coordinate System: " << coordMode << std::endl;
  std::cout << "Min angle: " << minAngleDegree << std::endl;
  std::cout << "Max angle: " << maxAngleDegree << std::endl;
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

  /* Scan�ݒ� */
  LaserScan scan;

  while (ydlidar::os_isOk()) {
    if (lidar.doProcessSimple(scan)) {
      osc.sendScanData(scan);
      dev_printf("Scan received [%llu] points scanFreq [%.02f]\n", scan.points.size(), scan.scanFreq);
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
