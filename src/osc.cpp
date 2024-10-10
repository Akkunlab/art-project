#include "config.h"
#include "osc.h"
#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"
#include <iostream>
#include <vector>
#include <utility>

OSC::OSC() : transmitSocket(nullptr) {
}

OSC::~OSC() {
  if (transmitSocket != nullptr) delete transmitSocket;
}

void OSC::setup(const std::string& ipAddress, int port) {
  if (transmitSocket != nullptr) delete transmitSocket;

  transmitSocket = new UdpTransmitSocket(IpEndpointName(ipAddress.c_str(), port));
}

void OSC::sendClusterData(const std::vector<std::pair<float, float>>& clusterCenters) {
  char buffer[OUTPUT_BUFFER_SIZE];
  osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);
  p << osc::BeginMessage(OSC_ADDRESS);

  for (const auto& center : clusterCenters) {
    p << center.first << center.second;
  }

  p << osc::EndMessage;
  transmitSocket->Send(p.Data(), p.Size());
}
