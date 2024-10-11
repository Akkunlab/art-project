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

#include "config.h"
#include "lidar.h"
#include "osc.h"
#include <iostream>
#include <algorithm>
#include <cctype>
#include <core/base/timer.h>
#include <core/common/ydlidar_help.h>

Lidar::Lidar() {
  ydlidar::os_init();
}

Lidar::~Lidar() {
  stop();
  disconnect();
}

void Lidar::setupLidarOptions() {
  int optval;
  bool b_optvalue;
  float f_optvalue;

  std::string ignore_array;
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(), ignore_array.size());

  optval = TYPE_TRIANGLE;
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  optval = YDLIDAR_TYPE_SERIAL;
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  optval = 5;
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  optval = 4;
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
  optval = 10;
  laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

  b_optvalue = false;
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  b_optvalue = false;
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
  b_optvalue = false;
  laser.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));

  f_optvalue = maxAngleDegree;
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = minAngleDegree;
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  f_optvalue = 64.f;
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.05f;
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));

  laser.enableGlassNoise(false);
  laser.enableSunNoise(false);
  laser.setBottomPriority(true);
  laser.setEnableDebug(false);
}

std::map<std::string, std::string> Lidar::getPortList() {
  return ydlidar::lidarPortList();
}

std::map<int, int> Lidar::getBaudrateList() {
  std::map<int, int> baudrateList;
  baudrateList[0] = 115200;
  baudrateList[1] = 128000;
  baudrateList[2] = 150000;
  baudrateList[3] = 153600;
  baudrateList[4] = 230400;
  baudrateList[5] = 460800;
  baudrateList[6] = 512000;
  return baudrateList;
}

void Lidar::setPort(const std::string& port) {
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
}

void Lidar::setBaudrate(int baudrate) {
  laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
}

void Lidar::setSingleChannel(bool isSingleChannel) {
  laser.setlidaropt(LidarPropSingleChannel, &isSingleChannel, sizeof(bool));
}

void Lidar::setFrequency(float frequency) {
  laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));
}

std::string Lidar::selectPort() {
  std::map<std::string, std::string> ports = getPortList();
  std::string port;

  if (ports.size() == 1) {
    port = ports.begin()->second;
  } else {
    int id = 0;
    for (const auto& pair : ports) {
      printf("[%d] %s %s\n", id, pair.first.c_str(), pair.second.c_str());
      id++;
    }

    if (ports.empty()) {
      printf("No Lidar was detected. Please enter the lidar serial port:");
      std::cin >> port;
    } else {
      while (ydlidar::os_isOk()) {
        printf("Please select the lidar port:");
        std::string number;
        std::cin >> number;

        if ((size_t)atoi(number.c_str()) >= ports.size()) continue;

        auto it = ports.begin();
        std::advance(it, atoi(number.c_str()));
        port = it->second;
        break;
      }
    }
  }

  if (!ydlidar::os_isOk()) return "";

  setPort(port);
  return port;
}

int Lidar::selectBaudrate() {
  std::map<int, int> baudrateList = getBaudrateList();
  int baudrate = 230400;

  printf("Baudrate:\n");
  for (const auto& pair : baudrateList) {
    printf("[%d] %d\n", pair.first, pair.second);
  }

  while (ydlidar::os_isOk()) {
    printf("Please select the lidar baudrate:");
    std::string number;
    std::cin >> number;

    if ((size_t)atoi(number.c_str()) >= baudrateList.size()) continue;

    baudrate = baudrateList[atoi(number.c_str())];
    break;
  }

  if (!ydlidar::os_isOk()) return -1;

  setBaudrate(baudrate);
  return baudrate;
}

bool Lidar::selectSingleChannel() {
  bool isSingleChannel = false;
  std::string input_channel;
  printf("Whether the Lidar is one-way communication [yes/no]:");
  std::cin >> input_channel;
  std::transform(input_channel.begin(), input_channel.end(), input_channel.begin(), [](unsigned char c) { return std::tolower(c); });

  if (input_channel.find("y") != std::string::npos) isSingleChannel = true;

  setSingleChannel(isSingleChannel);
  return isSingleChannel;
}

float Lidar::selectFrequency() {
  float frequency = 5.0;
  std::string input_frequency;
  while (ydlidar::os_isOk()) {
    printf("Please enter the lidar scan frequency[5-12]:");
    std::cin >> input_frequency;
    frequency = atof(input_frequency.c_str());

    if (frequency <= 12 && frequency >= 5.0) break;

    fprintf(stderr, "Invalid scan frequency, The scanning frequency range is 5 to 12 HZ, Please re-enter.\n");
  }

  if (!ydlidar::os_isOk()) return -1;

  setFrequency(frequency);
  return frequency;
}

bool Lidar::initialize() {
  setupLidarOptions();
  return laser.initialize();
}

bool Lidar::start() {
  return laser.turnOn();
}

void Lidar::stop() {
  laser.turnOff();
}

void Lidar::disconnect() {
  laser.disconnecting();
}

bool Lidar::doProcessSimple(LaserScan& scan) {
  return laser.doProcessSimple(scan);
}
