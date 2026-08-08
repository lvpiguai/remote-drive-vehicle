#include "vehicle/vehicle.h"

#include "protocol/udp_codec.h"
#include <poll.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <string>
#include <utility>

namespace {

namespace pb = remote_drive::protocol;

constexpr auto kHeartbeatInterval = std::chrono::seconds(1);
constexpr auto kStateInterval = std::chrono::milliseconds(100);

// 将 UDP 来源地址转换为控制者标识
VehicleControlSession::ControllerId controllerId(const sockaddr_in &address) {
  // 本地原型用来源 IP:port 作为驾驶舱控制者身份
  return (static_cast<std::uint64_t>(address.sin_addr.s_addr) << 16) |
         address.sin_port;
}

// 判断两个 IPv4 UDP 端点是否完全相同
bool sameEndpoint(const sockaddr_in &left, const sockaddr_in &right) {
  return left.sin_family == right.sin_family &&
         left.sin_addr.s_addr == right.sin_addr.s_addr &&
         left.sin_port == right.sin_port;
}

} // namespace

// 保存车端实例标识和本地监听端口
Vehicle::Vehicle(std::string vehicle_id, std::uint16_t local_port,
                 std::vector<sockaddr_in> cockpit_endpoints)
    : vehicle_id_(std::move(vehicle_id)), local_port_(local_port),
      cockpit_endpoints_(std::move(cockpit_endpoints)),
      chassis_gateway_(vehicle_id_), control_session_(chassis_gateway_) {}

// 主循环：初始化通信，循环处理控制、发送心跳和车辆状态
int Vehicle::run() {
  // 绑定车端 UDP socket
  if (!initialize())
    return 1;
  std::cout << "车端 " << vehicle_id_ << " 已监听 UDP " << local_port_ << '\n';

  // 首轮循环立即发送心跳和状态，避免等待一个完整周期
  auto last_heartbeat = Clock::now() - kHeartbeatInterval;
  auto last_state = Clock::now() - kStateInterval;

  while (true) {
    // 使用同一个 socket 接收驾驶舱控制并发送心跳和车辆状态
    pollfd descriptor{udp_channel_.fd(), POLLIN, 0};
    const int result = poll(&descriptor, 1, 100);
    if (result < 0) {
      perror("vehicle poll");
      return 1;
    }
    if (descriptor.revents & POLLIN)
      receiveControlPacket();

    const auto now = Clock::now();
    checkControlSession(now);

    if (now - last_heartbeat >= kHeartbeatInterval) {
      sendHeartbeat();
      last_heartbeat = now;
    }

    if (now - last_state >= kStateInterval) {
      sendState();
      last_state = now;
    }
  }
}

// 初始化车端 UDP 通道
bool Vehicle::initialize() {
  if (!udp_channel_.bindPort(local_port_)) {
    perror("vehicle bind");
    return false;
  }
  return true;
}

// 解码一条控制数据报，并将合法指令交给当前远控会话
void Vehicle::receiveControlPacket() {
  UdpDatagram datagram;
  if (!udp_channel_.receive(datagram))
    return;
  const sockaddr_in &source = datagram.source;

  // 过滤未配置的驾驶舱端点
  const bool known_cockpit =
      std::any_of(cockpit_endpoints_.begin(), cockpit_endpoints_.end(),
                  [&](const sockaddr_in &endpoint) {
                    return sameEndpoint(endpoint, source);
                  });
  if (!known_cockpit)
    return;

  // 解码并校验控制协议
  const auto packet = udp_codec::decodePacket(datagram.payload.data(),
                                              datagram.payload.size());
  if (!packet || packet->body_case() != pb::UdpPacket::kControl) {
    std::cout << "[控制接收] result=INVALID_PACKET size="
              << datagram.payload.size() << '\n';
    return;
  }
  const pb::RemoteDriveControlCommand &command = packet->control();
  const std::uint32_t sequence = packet->sequence();

  // 基于来源端点识别控制者
  const auto controller = controllerId(source);

  // 由会话完成指令校验、控制权管理和底盘发布
  const auto outcome =
      control_session_.handleCommand(command, sequence, controller);
  if (outcome.ended) {
    std::cout << "车辆已退出远程驾驶模式\n";
  } else if (outcome.applied &&
             command.remote_mode() == pb::REMOTE_MODE_ENTER) {
    std::cout << "车辆已进入远程驾驶模式\n";
  }
}

// 编码并向全部驾驶舱发送车辆在线心跳
void Vehicle::sendHeartbeat() {
  const auto packet =
      udp_codec::encodeHeartbeat(vehicle_id_, heartbeat_seq_++);
  for (const auto &endpoint : cockpit_endpoints_) {
    if (!udp_channel_.send(endpoint, packet.data(), packet.size())) {
      perror("send vehicle heartbeat");
    }
  }
}

// 编码并发送一份当前车辆状态快照
void Vehicle::sendState() {
  const pb::ChassisState state = drivingState();
  const auto packet = udp_codec::encodeDrivingState(state, state_seq_++);
  for (const auto &endpoint : cockpit_endpoints_) {
    if (!udp_channel_.send(endpoint, packet.data(), packet.size())) {
      perror("send vehicle state");
    }
  }
}

// 独立检查控制会话超时，不依赖心跳或状态发送周期
void Vehicle::checkControlSession(Clock::time_point now) {
  if (control_session_.tick(now))
    return;

  std::cout << "远控指令超时，车辆已恢复自动模式\n";
}

// 将底盘网关状态复制到车端共享协议结构
pb::ChassisState Vehicle::drivingState() const {
  const auto latest_state = chassis_gateway_.latestState();
  pb::ChassisState state = latest_state.value_or(pb::ChassisState{});
  state.set_vehicle_id(vehicle_id_);
  state.set_controller_id(control_session_.controllerId());
  if (!latest_state) {
    state.set_parking(true);
  }
  return state;
}
