#pragma once

#include <string>
#include <vector>
#include <utility>
#include <CYdLidar.h>
#include <functional>

template<typename T>

void setupParameter(const std::string& prompt, T& parameter, const T& defaultValue, std::function<bool(const std::string&)> validation = nullptr) {
  std::cout << prompt << " [default: " << defaultValue << ", type 'd' to use default]: ";
  std::string input;
  std::cin >> input;

  if (input == "d") {
    parameter = defaultValue;  // �f�t�H���g�l���g�p
  } else {
    try {
      if constexpr (std::is_same<T, int>::value) {
        int value = std::stoi(input);
        if (!validation || validation(input)) {
          parameter = value;  // �����Ƃ��ăp�����[�^��ݒ�
        } else {
          throw std::invalid_argument("Invalid input");
        }
      } else if constexpr (std::is_same<T, float>::value) {
        float value = std::stof(input);
        if (!validation || validation(input)) {
          parameter = value;  // ���������_���Ƃ��ăp�����[�^��ݒ�
        } else {
          throw std::invalid_argument("Invalid input");
        }
      } else if constexpr (std::is_same<T, std::string>::value) {
        if (!validation || validation(input)) {
          parameter = input;  // ������Ƃ��ăp�����[�^��ݒ�
        } else {
          throw std::invalid_argument("Invalid input");
        }
      } else if constexpr (std::is_same<T, bool>::value) {
        parameter = (input == "yes" || input == "y" || input == "true");  // true/false �̓��͂�����
      }
    } catch (const std::invalid_argument&) {
      std::cerr << "Invalid input. Using default value " << defaultValue << std::endl;
      parameter = defaultValue;
    }
  }
}

bool isValidIpAddress(const std::string &ipAddress);
bool isValidPort(const std::string& input);

float calculateDistance(float x1, float y1, float x2, float y2);

void performClustering(const std::vector<std::pair<float, float>>& points, float maxDistance, int minClusterSize, std::vector<std::pair<float, float>>& clusterCenters);

std::vector<std::pair<float, float>> getTouchPoints(const LaserScan& scan, float touchMinRange, float touchMaxRange);
