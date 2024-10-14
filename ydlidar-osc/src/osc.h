#pragma once

#include <vector>
#include <utility>
#include <string>
#include <ip\UdpSocket.h>
#include "CYdLidar.h"

class OSC {
public:
  OSC();
  ~OSC();

  void setup(const std::string& ipAddress, int port);
  void sendScanData(const LaserScan& scan);

private:
  UdpTransmitSocket* transmitSocket;
};
