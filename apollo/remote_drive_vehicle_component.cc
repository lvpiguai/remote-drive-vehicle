#include "modules/remote_drive_vehicle/apollo/remote_drive_vehicle_component.h"

#include <arpa/inet.h>
#include <poll.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <utility>

#include "cyber/component/component.h"
#include "modules/remote_drive_vehicle/include/protocol/udp_codec.h"
#include "modules/remote_drive_vehicle/proto/remote_drive_vehicle_config.pb.h"

namespace remote_drive::vehicle {

namespace {

constexpr std::size_t kMaxVehicleIdLength = 19;

// 将配置中的 IPv4 地址转换为 UDP socket 地址
bool parseEndpoint(const UdpEndpoint &config, sockaddr_in &endpoint) {
  endpoint = {};
  endpoint.sin_family = AF_INET;
  if (config.port() == 0 || config.port() > 65535 ||
      inet_pton(AF_INET, config.ip().c_str(), &endpoint.sin_addr) != 1) {
    return false;
  }
  endpoint.sin_port = htons(static_cast<std::uint16_t>(config.port()));
  return true;
}

// vehicle_id 会进入 UDP 协议和 Web 状态，限制长度及字符串转义字符
bool validVehicleId(const std::string &vehicle_id) {
  return !vehicle_id.empty() && vehicle_id.size() <= kMaxVehicleIdLength &&
         vehicle_id.find_first_of("\\\"") == std::string::npos;
}

// 使用来源 IP 和端口生成当前进程内稳定的控制者标识
VehicleControlSession::ControllerId controllerId(const sockaddr_in &address) {
  return (static_cast<std::uint64_t>(address.sin_addr.s_addr) << 16) |
         address.sin_port;
}

// 驾驶舱必须同时匹配配置中的来源 IP 和端口
bool sameEndpoint(const sockaddr_in &left, const sockaddr_in &right) {
  return left.sin_family == right.sin_family &&
         left.sin_addr.s_addr == right.sin_addr.s_addr &&
         left.sin_port == right.sin_port;
}

}  // namespace

// 先停止 UDP 线程，再向底盘发送仍处于活动状态的安全退出指令
RemoteDriveVehicleComponent::~RemoteDriveVehicleComponent() {
  running_ = false;
  if (udp_worker_.joinable()) {
    udp_worker_.join();
  }
  if (control_writer_) {
    if (const auto command = session_.stop(true)) {
      control_writer_->Write(*command);
    }
  }
}

// Cyber 框架已在调用 Init() 前根据 DAG 创建好基类 node_
bool RemoteDriveVehicleComponent::Init() {
  if (!loadConfig()) return false;
  if (!udp_channel_.bindPort(local_port_)) return false;

  control_writer_ = node_->CreateWriter<protocol::RemoteDriveControlCommand>(
      control_channel_);
  if (!control_writer_) return false;

  running_ = true;
  udp_worker_ = std::thread(&RemoteDriveVehicleComponent::runLoop, this);
  return true;
}

// Proc() 由 DAG 中配置的底盘状态 Reader 触发
bool RemoteDriveVehicleComponent::Proc(
    const std::shared_ptr<protocol::ChassisState> &state) {
  onChassisState(state);
  return state != nullptr;
}

// 配置属于当前车辆部署，加载后转换为工作线程直接使用的运行参数
bool RemoteDriveVehicleComponent::loadConfig() {
  RemoteDriveVehicleConfig config;
  if (!GetProtoConfig(&config) || !config.IsInitialized()) return false;

  if (!validVehicleId(config.vehicle_id()) || config.local_port() == 0 ||
      config.local_port() > 65535 || config.cockpits().empty() ||
      config.control_channel().empty() || config.heartbeat_interval_ms() == 0 ||
      config.state_interval_ms() == 0) {
    return false;
  }

  std::vector<sockaddr_in> endpoints;
  endpoints.reserve(config.cockpits_size());
  for (const auto &cockpit : config.cockpits()) {
    sockaddr_in endpoint{};
    if (!parseEndpoint(cockpit, endpoint)) return false;
    endpoints.push_back(endpoint);
  }

  vehicle_id_ = config.vehicle_id();
  control_channel_ = config.control_channel();
  local_port_ = static_cast<std::uint16_t>(config.local_port());
  heartbeat_interval_ =
      std::chrono::milliseconds(config.heartbeat_interval_ms());
  state_interval_ = std::chrono::milliseconds(config.state_interval_ms());
  cockpit_endpoints_ = std::move(endpoints);
  return true;
}

// UDP 工作线程串行处理网络输入、会话超时和周期发送
void RemoteDriveVehicleComponent::runLoop() {
  auto last_heartbeat = Clock::now() - heartbeat_interval_;
  auto last_state = Clock::now() - state_interval_;

  while (running_) {
    pollfd descriptor{udp_channel_.fd(), POLLIN, 0};
    const int result = poll(&descriptor, 1, 100);
    if (result > 0 && (descriptor.revents & POLLIN)) {
      receiveControlPacket();
    }

    const auto now = Clock::now();
    if (const auto command = session_.tick(now)) {
      control_writer_->Write(*command);
    }

    if (now - last_heartbeat >= heartbeat_interval_) {
      sendHeartbeat();
      last_heartbeat = now;
    }

    if (now - last_state >= state_interval_) {
      sendState();
      last_state = now;
    }
  }
}

// 只接收静态白名单内驾驶舱发送且通过会话校验的控制指令
void RemoteDriveVehicleComponent::receiveControlPacket() {
  UdpDatagram datagram;
  if (!udp_channel_.receive(datagram)) return;

  const bool known_cockpit =
      std::any_of(cockpit_endpoints_.begin(), cockpit_endpoints_.end(),
                  [&](const sockaddr_in &endpoint) {
                    return sameEndpoint(endpoint, datagram.source);
                  });
  if (!known_cockpit) return;

  const auto packet = udp_codec::decodePacket(datagram.payload.data(),
                                              datagram.payload.size());
  if (!packet || packet->body_case() != protocol::UdpPacket::kControl) return;

  const auto controller = controllerId(datagram.source);
  const auto &command = packet->control();
  if (session_.accept(command, packet->sequence(), controller)) {
    control_writer_->Write(command);
  }
}

// 心跳用于让驾驶舱发现车辆及刷新在线状态
void RemoteDriveVehicleComponent::sendHeartbeat() {
  const auto packet =
      udp_codec::encodeHeartbeat(vehicle_id_, heartbeat_seq_++);
  for (const auto &endpoint : cockpit_endpoints_) {
    udp_channel_.send(endpoint, packet.data(), packet.size());
  }
}

// 状态来源是 Cyber Reader 最近一次收到的真实底盘状态
void RemoteDriveVehicleComponent::sendState() {
  const protocol::ChassisState state = vehicleState();
  const auto packet = udp_codec::encodeDrivingState(state, state_seq_++);
  for (const auto &endpoint : cockpit_endpoints_) {
    udp_channel_.send(endpoint, packet.data(), packet.size());
  }
}

// 尚未收到真实底盘状态时返回一个驻车状态，避免表示车辆可直接控制
protocol::ChassisState RemoteDriveVehicleComponent::vehicleState() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  protocol::ChassisState state = latest_state_.value_or(protocol::ChassisState{});
  state.set_vehicle_id(vehicle_id_);
  state.set_controller_id(session_.controllerId());
  if (!latest_state_) {
    state.set_parking(true);
  }
  return state;
}

// Reader 回调只更新缓存，不在 Cyber 调度线程中执行 UDP 发送
void RemoteDriveVehicleComponent::onChassisState(
    const std::shared_ptr<protocol::ChassisState> &state) {
  if (!state) return;
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_state_ = *state;
}

CYBER_REGISTER_COMPONENT(RemoteDriveVehicleComponent)

}  // namespace remote_drive::vehicle
