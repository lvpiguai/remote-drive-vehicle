#include "vehicle/vehicle.h"

#include "protocol/udp_protocol.h"
#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <utility>

namespace {

// 将完整数据报发送到指定地址
bool sendBytes(int fd, const sockaddr_in &address, const void *data,
               std::size_t size) {
  return sendto(fd, data, size, 0, reinterpret_cast<const sockaddr *>(&address),
                sizeof(address)) == static_cast<ssize_t>(size);
}

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
      chassis_gateway_(vehicle_id_) {}

// 释放车端持有的 UDP socket
Vehicle::~Vehicle() {
  if (socket_fd_ >= 0)
    close(socket_fd_);
}

// 初始化通信并持续处理控制、心跳和状态回传
int Vehicle::run() {
  if (!initialize())
    return 1;
  std::cout << "车端 " << vehicle_id_ << " 已监听 UDP " << local_port_ << '\n';

  // 首轮循环立即发送心跳和状态，避免等待一个完整周期
  auto last_heartbeat = Clock::now() - std::chrono::seconds(1);
  auto last_state = Clock::now() - std::chrono::milliseconds(100);

  while (true) {
    // 使用同一个 socket 接收驾驶舱控制并发送心跳和车辆状态
    pollfd descriptor{socket_fd_, POLLIN, 0};
    const int result = poll(&descriptor, 1, 100);
    if (result < 0) {
      perror("vehicle poll");
      return 1;
    }
    if (descriptor.revents & POLLIN)
      receiveControlPacket();

    const auto now = Clock::now();

    // 每秒向驾驶舱发送一次车辆在线心跳并递增独立序列号
    if (now - last_heartbeat >= std::chrono::seconds(1)) {
      const auto packet = remote_protocol::encodeHeartbeat(
          vehicle_id_, heartbeat_seq_++);
      for (const auto &endpoint : cockpit_endpoints_) {
        sendBytes(socket_fd_, endpoint, packet.data(), packet.size());
      }
      last_heartbeat = now;
    }

    // 每 100ms 检查远控会话并回传最近底盘状态快照
    if (now - last_state >= std::chrono::milliseconds(100)) {
      if (control_session_ && !control_session_->tick(now)) {
        control_session_.reset();
        controller_id_.clear();
        std::cout << "远控指令超时，车辆已恢复自动模式\n";
      }
      sendState();
      last_state = now;
    }
  }
}

// 创建并绑定车端 UDP socket
bool Vehicle::initialize() {
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    perror("vehicle socket");
    return false;
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(local_port_);
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&address),
           sizeof(address)) < 0) {
    perror("vehicle bind");
    return false;
  }
  return true;
}

// 解码一条控制数据报，并将合法指令交给当前远控会话
void Vehicle::receiveControlPacket() {
  // 接收控制数据报及来源端点
  std::uint8_t buffer[256]{};
  sockaddr_in source{};
  socklen_t source_size = sizeof(source);
  const ssize_t size =
      recvfrom(socket_fd_, buffer, sizeof(buffer), 0,
               reinterpret_cast<sockaddr *>(&source), &source_size);
  if (size <= 0)
    return;

  // 过滤未配置的驾驶舱端点
  const bool known_cockpit =
      std::any_of(cockpit_endpoints_.begin(), cockpit_endpoints_.end(),
                  [&](const sockaddr_in &endpoint) {
                    return sameEndpoint(endpoint, source);
                  });
  if (!known_cockpit)
    return;

  // 解码并校验控制协议
  const auto packet =
      remote_protocol::decodePacket(buffer, static_cast<std::size_t>(size));
  if (!packet || packet->body != remote_protocol::PacketBody::CONTROL_CMD) {
    std::cout << "[控制接收] result=INVALID_PACKET size=" << size << '\n';
    return;
  }
  const RemoteCtlCmd &command = packet->control;
  const std::uint32_t sequence = packet->sequence;

  // 基于来源端点识别控制者
  const auto controller = controllerId(source);

  // 无会话时仅接受有效的进入远控指令
  if (!control_session_) {
    if (command.remoteMode != RemoteMode::REMOTE_ENTER || sequence == 0)
      return;
    control_session_.emplace(chassis_gateway_, controller);
    controller_id_ = command.cockpit_id;
  } else if (controller == control_session_->controller() &&
             controller_id_ != command.cockpit_id) {
    // 拒绝活动端点冒用其他驾驶舱 ID
    return;
  }

  // 会话校验控制权与序号并发布到底盘网关
  const auto outcome =
      control_session_->handleCommand(command, sequence, controller);
  if (outcome.ended) {
    // 退出会话并清理控制者
    control_session_.reset();
    controller_id_.clear();
    std::cout << "车辆已退出远程驾驶模式\n";
  } else if (outcome.applied &&
             command.remoteMode == RemoteMode::REMOTE_ENTER) {
    std::cout << "车辆已进入远程驾驶模式\n";
  }
}

// 编码并发送一份当前车辆状态快照
void Vehicle::sendState() {
  const RemoteDrivingState state = drivingState();
  const auto packet = remote_protocol::encodeDrivingState(state, state_seq_++);
  for (const auto &endpoint : cockpit_endpoints_) {
    if (!sendBytes(socket_fd_, endpoint, packet.data(), packet.size())) {
      perror("send vehicle state");
    }
  }
}

// 将底盘网关状态复制到车端共享协议结构
RemoteDrivingState Vehicle::drivingState() const {
  RemoteDrivingState state =
      chassis_gateway_.latestState().value_or(RemoteDrivingState{});
  std::memcpy(state.vehicle_id, vehicle_id_.data(),
              std::min(vehicle_id_.size(), sizeof(state.vehicle_id) - 1));
  std::memcpy(state.controller_id, controller_id_.data(),
              std::min(controller_id_.size(), sizeof(state.controller_id) - 1));
  return state;
}
