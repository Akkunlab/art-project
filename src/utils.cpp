#include "utils.h"
#include <cmath>
#include <sstream>
#include <vector>
#include <iostream>
#include <type_traits>
#include <stdexcept>
#include <CYdLidar.h>

/* IPアドレスのバリデーション */
bool isValidIpAddress(const std::string &ipAddress) {
  std::vector<int> octets;
  std::stringstream ss(ipAddress);
  std::string item;
  int octet;

  while (std::getline(ss, item, '.')) {
    try {
      octet = std::stoi(item);
    }
    catch (const std::invalid_argument&) {
      return false;
    }
    if (octet < 0 || octet > 255) {
      return false;
    }
    octets.push_back(octet);
  }

  return octets.size() == 4;
}

/* ポートのバリデーション */
bool isValidPort(const std::string& input) {
  try {
    int port = std::stoi(input);
    return port > 0 && port <= 65535;
  } catch (const std::invalid_argument&) {
    return false;
  } catch (const std::out_of_range&) {
    return false;
  }
}

/* 距離計算 */
float calculateDistance(float x1, float y1, float x2, float y2) {
  return sqrtf((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

/* クラスタリングを行う関数 */
void performClustering(const std::vector<std::pair<float, float>>& points, float maxDistance, int minClusterSize, std::vector<std::pair<float, float>>& clusterCenters) {
  std::vector<bool> visited(points.size(), false);
  std::vector<std::vector<std::pair<float, float>>> clusters;

  for (size_t i = 0; i < points.size(); ++i) {
    if (visited[i]) continue;

    visited[i] = true;
    std::vector<std::pair<float, float>> cluster;
    cluster.push_back(points[i]);

    std::vector<size_t> neighbors;
    for (size_t j = 0; j < points.size(); ++j) {
      if (!visited[j]) {
        float dist = calculateDistance(points[i].first, points[i].second, points[j].first, points[j].second);
        if (dist <= maxDistance) {
          visited[j] = true;
          cluster.push_back(points[j]);
        }
      }
    }

    if (cluster.size() >= static_cast<size_t>(minClusterSize)) {
      clusters.push_back(cluster);
    }
  }

  // 各クラスタの中心を計算
  for (const auto& cluster : clusters) {
    float sumX = 0.0f;
    float sumY = 0.0f;
    for (const auto& point : cluster) {
      sumX += point.first;
      sumY += point.second;
    }
    float centerX = sumX / cluster.size();
    float centerY = sumY / cluster.size();
    clusterCenters.emplace_back(centerX, centerY);
  }
}

/* 指定された距離範囲内のタッチポイントを抽出する関数 */
std::vector<std::pair<float, float>> getTouchPoints(const LaserScan& scan, float touchMinRange, float touchMaxRange) {
  std::vector<std::pair<float, float>> touchPoints;

  for (const auto& pnt : scan.points) {

    /* 極座標 (距離, 角度) を直交座標 (x, y) に変換 */
    float x = pnt.range * cos(pnt.angle);
    float y = pnt.range * sin(pnt.angle);

    /* ポイントがタッチ検出範囲内かどうかを確認 */
    if (pnt.range >= touchMinRange && pnt.range <= touchMaxRange) {
        touchPoints.emplace_back(x, y);
    }
  }

  return touchPoints;
}
