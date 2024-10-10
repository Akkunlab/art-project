#pragma once

#include <vector>
#include <utility>
#include <string>
#include <ip\UdpSocket.h>

class OSC {
public:
  OSC();
  ~OSC();

  void setup(const std::string& ipAddress, int port);
  void sendClusterData(const std::vector<std::pair<float, float>>& clusterCenters);

private:
  UdpTransmitSocket* transmitSocket;
};
