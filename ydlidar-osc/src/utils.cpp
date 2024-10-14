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
