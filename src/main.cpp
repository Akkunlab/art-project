/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, EAIBOT, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include "CYdLidar.h"
#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <core/base/timer.h>
#include <core/common/ydlidar_help.h>

#define OUTPUT_BUFFER_SIZE 8192
#define DEFAULT_OSC_ADDRESS "127.0.0.1"
#define DEFAULT_OSC_PORT 7000
#define DEFAULT_MIN_ANGLE_DEGREE -180.0f
#define DEFAULT_MAX_ANGLE_DEGREE 180.0f
#define dev_printf(...) if(isDevelopmentMode) { printf(__VA_ARGS__); }

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

using namespace std;
using namespace ydlidar;

bool isDevelopmentMode = false;

// IPアドレスのバリデーション関数
bool isValidIpAddress(const std::string & ipAddress) {
    std::vector<int> octets;
    std::stringstream ss(ipAddress);
    std::string item;
    int octet;

    while (std::getline(ss, item, '.')) {
        try {
            octet = std::stoi(item);
        }
        catch (const std::invalid_argument&) {
            return false;  // 数字でない部分があれば無効
        }
        if (octet < 0 || octet > 255) {
            return false;  // 各セグメントは0-255の範囲内
        }
        octets.push_back(octet);
    }

    return octets.size() == 4;  // セグメントが4つあるか確認
}

// ポートのバリデーション関数
bool isValidPort(int port) {
    return port > 0 && port <= 65535;  // 1-65535の範囲か
}

// 度をラジアンに変換する関数
float degreeToRadian(float degree) {
    return degree * (M_PI / 180.0f);
}

// 距離の計算関数
float calculateDistance(float x1, float y1, float x2, float y2) {
    return sqrtf((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// クラスタリングを行う関数
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

/**
 * @brief ydlidar test
 * @param argc
 * @param argv
 * @return
 * @par Flow chart
 * Step1: instance CYdLidar.\n
 * Step2: set paramters.\n
 * Step3: initialize SDK and LiDAR.(::CYdLidar::initialize)\n
 * Step4: Start the device scanning routine which runs on a separate thread and enable motor.(::CYdLidar::turnOn)\n
 * Step5: Get the LiDAR Scan Data.(::CYdLidar::doProcessSimple)\n
 * Step6: Stop the device scanning thread and disable motor.(::CYdLidar::turnOff)\n
 * Step7: Uninitialize the SDK and Disconnect the LiDAR.(::CYdLidar::disconnecting)\n
 */

int main(int argc, char *argv[])
{
  // 開発モードの設定
  std::cout << "Enable development mode? [yes/no]:";
  std::string inputDevMode;
  std::cin >> inputDevMode;
  std::transform(inputDevMode.begin(), inputDevMode.end(), inputDevMode.begin(),
      [](unsigned char c) { return std::tolower(c); });
  if (inputDevMode.find("y") != std::string::npos) isDevelopmentMode = true;

  // 出力形式の設定
  std::cout << "Select output coordinate system [polar/cartesian]:";
  std::string coordMode;
  std::cin >> coordMode;
  std::transform(coordMode.begin(), coordMode.end(), coordMode.begin(),
      [](unsigned char c) { return std::tolower(c); });
  if (coordMode == "d" || coordMode.find("c") != std::string::npos) {
      coordMode = "cartesian";
  } else {
      coordMode = "polar";
  }

  // 出力範囲の設定
  float minAngleDegree;
  float maxAngleDegree;
  float minAngle;
  float maxAngle;

  std::cout << "Enter min angle in degrees [default: " << DEFAULT_MIN_ANGLE_DEGREE << ", type 'd' to use default]: ";
  std::string minAngleInput;
  std::cin >> minAngleInput;
  if (minAngleInput == "d") {
      minAngleDegree = DEFAULT_MIN_ANGLE_DEGREE;
  }
  else {
      minAngleDegree = std::stof(minAngleInput);
  }
  minAngle = degreeToRadian(minAngleDegree);

  std::cout << "Enter max angle in degrees [default: " << DEFAULT_MAX_ANGLE_DEGREE << ", type 'd' to use default]: ";
  std::string maxAngleInput;
  std::cin >> maxAngleInput;
  if (maxAngleInput == "d") {
      maxAngleDegree = DEFAULT_MAX_ANGLE_DEGREE;
  }
  else {
      maxAngleDegree = std::stof(maxAngleInput);
  }
  maxAngle = degreeToRadian(maxAngleDegree);

  // タッチ検出のための距離範囲を設定
  float touchMinRange = 0.1f; // タッチとみなす最小距離（メートル）
  float touchMaxRange = 0.5f; // タッチとみなす最大距離（メートル）

  // タッチ検出の最小距離を入力
  std::cout << "Enter touch detection min range in meters [default: 0.1]: ";
  std::string touchMinInput;
  std::cin >> touchMinInput;
  if (touchMinInput != "d") {
      touchMinRange = std::stof(touchMinInput);
  }

  // タッチ検出の最大距離を入力
  std::cout << "Enter touch detection max range in meters [default: 0.5]: ";
  std::string touchMaxInput;
  std::cin >> touchMaxInput;
  if (touchMaxInput != "d") {
      touchMaxRange = std::stof(touchMaxInput);
  }

  // クラスタリングのパラメータ設定
  float maxClusterDistance = 0.05f; // 同一クラスタとみなすポイント間の最大距離（メートル）
  int minClusterSize = 3;           // 有効なクラスタとみなす最小ポイント数

  std::cout << "Enter max distance between points in a cluster [default: 0.05]: ";
  std::string clusterDistanceInput;
  std::cin >> clusterDistanceInput;
  if (clusterDistanceInput != "d") {
      maxClusterDistance = std::stof(clusterDistanceInput);
  }

  std::cout << "Enter minimum number of points in a cluster [default: 3]: ";
  std::string minClusterSizeInput;
  std::cin >> minClusterSizeInput;
  if (minClusterSizeInput != "d") {
      minClusterSize = std::stoi(minClusterSizeInput);
  }


  // OSCのIPアドレスとポートを設定
  std::string oscAddress = DEFAULT_OSC_ADDRESS;
  int oscPort = DEFAULT_OSC_PORT;

  // IPアドレスの入力
  std::cout << "Enter OSC target IP address [default: " << DEFAULT_OSC_ADDRESS << ", type 'd' to use default]: ";
  std::string inputAddress;
  std::cin >> inputAddress;
  if (inputAddress == "d") {
      oscAddress = DEFAULT_OSC_ADDRESS;
  }
  else {
      if (isValidIpAddress(inputAddress)) {
          oscAddress = inputAddress;
      }
      else {
          std::cerr << "Invalid IP address. Using default IP " << DEFAULT_OSC_ADDRESS << "." << std::endl;
      }
  }

  // ポートの入力
  std::cout << "Enter OSC target port [default: " << DEFAULT_OSC_PORT << ", type 'd' to use default]: ";
  std::string portInput;
  std::cin >> portInput;
  if (portInput == "d") {
      oscPort = DEFAULT_OSC_PORT;
  }
  else {
      try {
          oscPort = std::stoi(portInput);  // 入力があれば整数に変換
          if (!isValidPort(oscPort)) {
              std::cerr << "Invalid port number. Using default port " << DEFAULT_OSC_PORT << "." << std::endl;
              oscPort = DEFAULT_OSC_PORT;
          }
      }
      catch (const std::invalid_argument&) {
          std::cerr << "Invalid input. Using default port " << DEFAULT_OSC_PORT << "." << std::endl;
          oscPort = DEFAULT_OSC_PORT;
      }
      catch (const std::out_of_range&) {
          std::cerr << "Port number out of range. Using default port " << DEFAULT_OSC_PORT << "." << std::endl;
          oscPort = DEFAULT_OSC_PORT;
      }
  }

  // OSC送信用ソケットの設定
  UdpTransmitSocket transmitSocket(IpEndpointName(oscAddress.c_str(), oscPort));
  char buffer[OUTPUT_BUFFER_SIZE];

  std::string port;
  ydlidar::os_init();

  std::map<std::string, std::string> ports =
      ydlidar::lidarPortList();
  std::map<std::string, std::string>::iterator it;

  if (ports.size() == 1)
  {
    port = ports.begin()->second;
  }
  else
  {
    int id = 0;

    for (it = ports.begin(); it != ports.end(); it++) {
        dev_printf("[%d] %s %s\n", id, it->first.c_str(), it->second.c_str());
        id++;
    }

    if (ports.empty())
    {
      printf("Not Lidar was detected. Please enter the lidar serial port:");
      std::cin >> port;
    }
    else
    {
      while (ydlidar::os_isOk())
      {
        printf("Please select the lidar port:");
        std::string number;
        std::cin >> number;

        if ((size_t)atoi(number.c_str()) >= ports.size())
        {
          continue;
        }

        it = ports.begin();
        id = atoi(number.c_str());

        while (id)
        {
          id--;
          it++;
        }

        port = it->second;
        break;
      }
    }
  }

  int baudrate = 128000;
  std::map<int, int> baudrateList;
  baudrateList[0] = 115200;
  baudrateList[1] = 128000;
  baudrateList[2] = 150000;
  baudrateList[3] = 153600;
  baudrateList[4] = 230400;
  baudrateList[5] = 460800;
  baudrateList[6] = 512000;

  printf("Baudrate:\n");

  for (std::map<int, int>::iterator it = baudrateList.begin();
       it != baudrateList.end(); it++)
  {
    printf("[%d] %d\n", it->first, it->second);
  }

  while (ydlidar::os_isOk())
  {
    printf("Please select the lidar baudrate:");
    std::string number;
    std::cin >> number;

    if ((size_t)atoi(number.c_str()) > baudrateList.size())
    {
      continue;
    }

    baudrate = baudrateList[atoi(number.c_str())];
    break;
  }

  if (!ydlidar::os_isOk())
    return 0;

  //单通还是双通
  bool isSingleChannel = false;
  std::string input_channel;
  printf("Whether the Lidar is one-way communication [yes/no]:");
  std::cin >> input_channel;
  std::transform(input_channel.begin(), input_channel.end(),
                 input_channel.begin(),
                 [](unsigned char c)
                 {
                   return std::tolower(c); // correct
                 });
  if (input_channel.find("y") != std::string::npos)
    isSingleChannel = true;

  if (!ydlidar::os_isOk())
    return 0;

  //转速
  float frequency = 5.0;
  std::string input_frequency;
  while (ydlidar::os_isOk() && !isSingleChannel)
  {
    printf("Please enter the lidar scan frequency[5-12]:");
    std::cin >> input_frequency;
    frequency = atof(input_frequency.c_str());
    if (frequency <= 12 && frequency >= 5.0)
    {
      break;
    }
    fprintf(stderr, "Invalid scan frequency,"
      "The scanning frequency range is 5 to 12 HZ, Please re-enter.\n");
  }

  if (!ydlidar::os_isOk())
    return 0;

  // 設定の確認
  std::cout << "\n========== Settings ==========" << std::endl;
  std::cout << "Development Mode: " << (isDevelopmentMode ? "Enabled" : "Disabled") << std::endl;
  std::cout << "Coordinate System: " << coordMode << std::endl;
  std::cout << "Min angle: " << minAngleDegree << std::endl;
  std::cout << "Max angle: " << maxAngleDegree << std::endl;
  std::cout << "Touch Min Range: " << touchMinRange << std::endl;
  std::cout << "Touch Max Range: " << touchMaxRange << std::endl;
  std::cout << "Max Cluster Distance: " << maxClusterDistance << std::endl;
  std::cout << "Min Cluster Size: " << minClusterSize << std::endl;
  std::cout << "OSC IP Address: " << oscAddress << std::endl;
  std::cout << "OSC Port: " << oscPort << std::endl;
  std::cout << "Baudrate: " << baudrate << std::endl;
  std::cout << "One-way: " << (isSingleChannel ? "Yes" : "No") << std::endl;
  std::cout << "Frequency: " << frequency << std::endl;
  std::cout << "==============================\n" << std::endl;

  CYdLidar laser;
  //////////////////////string property/////////////////
  /// lidar port
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  /// ignore array
  std::string ignore_array;
  ignore_array.clear();
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                    ignore_array.size());

  //////////////////////int property/////////////////
  /// lidar baudrate
  laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
  /// tof lidar
  int optval = TYPE_TRIANGLE;
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_SERIAL;
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = isSingleChannel ? 3 : 4;
  optval = 5;
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
  /// Intenstiy bit count
  optval = 10;
  laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = false;
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  b_optvalue = false;
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  b_optvalue = false;
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  b_optvalue = false;
  laser.setlidaropt(LidarPropSingleChannel, &isSingleChannel, sizeof(bool));
  /// intensity
  b_optvalue = false;
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = true;
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
  /// HeartBeat
  b_optvalue = false;
  laser.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = maxAngleDegree;
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = minAngleDegree;
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  /// unit: m
  f_optvalue = 64.f;
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.05f;
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

  //禁用阳光玻璃过滤
  laser.enableGlassNoise(false);
  laser.enableSunNoise(false);

  //设置是否获取底板设备信息（默认仅尝试获取模组设备信息）
  laser.setBottomPriority(true);
  //启用调试
  laser.setEnableDebug(false);

  uint32_t t = getms(); //时间
  int c = 0; //计数

  bool ret = laser.initialize();
  if (!ret)
  {
    fprintf(stderr, "Fail to initialize %s\n", laser.DescribeError());
    fflush(stderr);
    return -1;
  }

  ret = laser.turnOn();
  if (!ret)
  {
    fprintf(stderr, "Fail to start %s\n", laser.DescribeError());
    fflush(stderr);
    return -1;
  }

  //获取用户版本
  // if (ret && ydlidar::os_isOk())
  // {
  //   std::string userVersion;
  //   if (laser.getUserVersion(userVersion))
  //   {
  //     printf("User version %s\n", userVersion.c_str());
  //   }
  // }

  //获取设备信息
  if (ret)
  {
    device_info di;
    memset(&di, 0, DEVICEINFOSIZE);
    if (laser.getDeviceInfo(di, EPT_Module)) {
      ydlidar::core::common::printfDeviceInfo(di, EPT_Module);
    }
    else {
      printf("Fail to get module device info\n");
    }

    if (laser.getDeviceInfo(di, EPT_Base)) {
      ydlidar::core::common::printfDeviceInfo(di, EPT_Base);
    }
    else {
      printf("Fail to get baseplate device info\n");
    }
  }

  LaserScan scan;
  while (ydlidar::os_isOk())
  {
    if (laser.doProcessSimple(scan))
    {
      // タッチ検出のためのポイントを格納するベクター
      std::vector<std::pair<float, float>> touchPoints;

      for (size_t i = 0; i < scan.points.size(); ++i)
      {
        const LaserPoint& pnt = scan.points.at(i);
        float x = pnt.range * cos(pnt.angle);
        float y = pnt.range * sin(pnt.angle);

        // タッチ検出の距離範囲内か確認
        if (pnt.range >= touchMinRange && pnt.range <= touchMaxRange)
        {
          touchPoints.emplace_back(x, y);
        }
      }

      // クラスタリングを実行
      std::vector<std::pair<float, float>> clusterCenters;
      performClustering(touchPoints, maxClusterDistance, minClusterSize, clusterCenters);

      // クラスタの中心座標をOSCで送信
      if (!clusterCenters.empty())
      {
        char buffer[OUTPUT_BUFFER_SIZE];
        osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);
        p << osc::BeginMessage("/lidar/touchClusters");

        for (const auto& center : clusterCenters)
        {
          p << center.first << center.second; // クラスタの中心座標を送信
        }

        p << osc::EndMessage;
        transmitSocket.Send(p.Data(), p.Size());
      }

      dev_printf("Clusters detected: %zu\n", clusterCenters.size());
    }
    else
    {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }
    if (!c++)
    {
      printf("Time consuming [%u] from initialization to parsing to point cloud data\n",
        getms() - t);
    }
  }

  laser.turnOff();
  laser.disconnecting();

  return 0;
}
