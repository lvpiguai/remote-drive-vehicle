#pragma once

#include <netinet/in.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "remote_drive.pb.h"
#include "vehicle/cyber_chassis_gateway.h"
#include "vehicle/vehicle_control_session.h"

// 车端进程：接收远控指令，维护控制会话并周期回传车辆状态
class Vehicle {
public:
  Vehicle(std::string vehicle_id, std::uint16_t local_port,
          std::vector<sockaddr_in> cockpit_endpoints);
  ~Vehicle();

  Vehicle(const Vehicle &) = delete;
  Vehicle &operator=(const Vehicle &) = delete;

  // 初始化资源并持续运行车端事件循环
  int run();

private:
  using Clock = std::chrono::steady_clock;

  // 创建通信 socket 并设置驾驶舱地址
  bool initialize();

  // 接收并处理一个远程控制数据包
  void receiveControlPacket();

  // 编码并回传当前车辆状态
  void sendState();

  // 从底盘网关最近快照组装车辆状态
  remote_drive::protocol::ChassisState drivingState() const;

  std::string vehicle_id_;
  std::uint16_t local_port_;
  int socket_fd_ = -1;
  std::vector<sockaddr_in> cockpit_endpoints_;
  std::string controller_id_;
  CyberChassisGateway chassis_gateway_;
  std::optional<VehicleControlSession> control_session_;
  std::uint32_t heartbeat_seq_ = 1;
  std::uint32_t state_seq_ = 1;
};
