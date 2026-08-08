#include <arpa/inet.h>

#include <charconv>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "protocol/remote_control_protocol.h"
#include "vehicle/vehicle.h"

namespace {

// 车端入口解析完成后的启动配置
struct VehicleOptions {
  std::string vehicle_id;
  std::uint16_t local_port = 0;
  std::vector<sockaddr_in> cockpit_endpoints;
};

// 将十进制文本解析为有效的非零端口号
bool parsePort(std::string_view text, std::uint16_t &port) {
  unsigned int value = 0;
  const auto result =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size() ||
      value == 0 || value > 65535) {
    return false;
  }
  port = static_cast<std::uint16_t>(value);
  return true;
}

// 解析 ipv4:port 格式的驾驶舱通信端点
bool parseEndpoint(std::string_view text, sockaddr_in &endpoint) {
  const auto separator = text.rfind(':');
  if (separator == std::string_view::npos)
    return false;

  const std::string ip(text.substr(0, separator));
  std::uint16_t port = 0;
  endpoint = {};
  endpoint.sin_family = AF_INET;
  if (!parsePort(text.substr(separator + 1), port) ||
      inet_pton(AF_INET, ip.c_str(), &endpoint.sin_addr) != 1) {
    return false;
  }
  endpoint.sin_port = htons(port);
  return true;
}

// 校验车辆 ID 能否安全写入定长协议字段和 Web JSON
bool validVehicleId(const std::string &vehicle_id) {
  return !vehicle_id.empty() &&
         vehicle_id.size() < sizeof(RemoteDrivingState{}.vehicle_id) &&
         vehicle_id.find_first_of("\\\"") == std::string::npos;
}

// 按“车辆 ID、本地端口、驾驶舱端点...”的固定顺序解析参数
std::optional<VehicleOptions> parseArguments(int argc, char *argv[]) {
  if (argc < 4)
    return std::nullopt;

  VehicleOptions options;
  options.vehicle_id = argv[1];
  if (!validVehicleId(options.vehicle_id) ||
      !parsePort(argv[2], options.local_port)) {
    return std::nullopt;
  }

  options.cockpit_endpoints.reserve(static_cast<std::size_t>(argc - 3));
  for (int index = 3; index < argc; ++index) {
    sockaddr_in endpoint{};
    if (!parseEndpoint(argv[index], endpoint))
      return std::nullopt;
    options.cockpit_endpoints.push_back(endpoint);
  }
  return options;
}

} // namespace

// 解析本地端口和驾驶舱端点后启动车端进程
int main(int argc, char *argv[]) {
  auto options = parseArguments(argc, argv);
  if (!options) {
    std::cerr << "用法：" << argv[0]
              << " vehicle_id vehicle_port cockpit_ipv4:port...\n";
    return 1;
  }

  Vehicle vehicle(std::move(options->vehicle_id), options->local_port,
                  std::move(options->cockpit_endpoints));
  return vehicle.run();
}
