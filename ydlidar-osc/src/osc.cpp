#include "config.h"
#include "osc.h"
#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"
#include "CYdLidar.h"
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

void OSC::sendScanData(const LaserScan& scan) {
  char buffer[OUTPUT_BUFFER_SIZE];
  osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);
  p << osc::BeginMessage(OSC_ADDRESS);

  if (coordMode == "polar") {
    for (size_t i = 0; i < scan.points.size(); ++i) {
      const LaserPoint& pnt = scan.points.at(i);
      p << (float)pnt.angle << (float)pnt.range;
    }
  } else {
    for (size_t i = 0; i < scan.points.size(); ++i) {
      const LaserPoint& pnt = scan.points.at(i);
      float x = pnt.range * cos(pnt.angle);
      float y = pnt.range * sin(pnt.angle);
      p << x << y;
    }
  }

  p << osc::EndMessage;
  transmitSocket->Send(p.Data(), p.Size());
}
