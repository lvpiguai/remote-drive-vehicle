#include "modules/remote_drive_vehicle/include/remote_drive_component.h"

#include <arpa/inet.h>
#include <poll.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <utility>

#include "cyber/component/component.h"
#include "modules/remote_drive_vehicle/include/protocol_codec.h"
#include "modules/remote_drive_vehicle/proto/remote_drive_vehicle_config.pb.h"

namespace remote_drive::vehicle {

namespace {

constexpr std::size_t kMaxVehicleIdLength = 19;

// 底盘控制 Channel
constexpr char kControlChannel[] = "/remote_drive/control_cmd";

// 解析驾驶舱端点
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

// 校验车辆 ID
bool validVehicleId(const std::string &vehicle_id) {
  return !vehicle_id.empty() && vehicle_id.size() <= kMaxVehicleIdLength &&
         vehicle_id.find_first_of("\\\"") == std::string::npos;
}

// 生成会话来源标识
VehicleControlSession::ControllerId controllerId(const sockaddr_in &address) {
  return (static_cast<std::uint64_t>(address.sin_addr.s_addr) << 16) |
         address.sin_port;
}

// 比较 UDP 端点
bool sameEndpoint(const sockaddr_in &left, const sockaddr_in &right) {
  return left.sin_family == right.sin_family &&
         left.sin_addr.s_addr == right.sin_addr.s_addr &&
         left.sin_port == right.sin_port;
}

}  // namespace

// 停止组件
RemoteDriveComponent::~RemoteDriveComponent() {
  // 停止 UDP 工作线程
  running_ = false;
  if (udp_worker_.joinable()) {
    udp_worker_.join();
  }

  // 正常关闭时退出活动会话
  if (control_writer_) {
    if (const auto exit_command = session_.stop()) {
      control_writer_->Write(*exit_command);
    }
  }
}

// 初始化组件
bool RemoteDriveComponent::Init() {
  // 加载配置并绑定 UDP 端口
  if (!loadConfig()) return false;
  if (!udp_channel_.bindPort(local_port_)) return false;

  // 创建底盘控制 Writer
  control_writer_ = node_->CreateWriter<protocol::RemoteDriveControlCommand>(
      kControlChannel);
  if (!control_writer_) return false;

  // 启动 UDP 工作线程
  running_ = true;
  udp_worker_ = std::thread(&RemoteDriveComponent::runLoop, this);
  return true;
}

// 缓存底盘状态
bool RemoteDriveComponent::Proc(
    const std::shared_ptr<protocol::ChassisState> &state) {
  if (!state) return false;

  // 更新共享状态缓存
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_state_ = *state;
  return true;
}

// 加载车端配置
bool RemoteDriveComponent::loadConfig() {
  // 读取并校验基础字段
  RemoteDriveVehicleConfig config;
  if (!GetProtoConfig(&config) || !config.IsInitialized()) return false;

  if (!validVehicleId(config.vehicle_id()) || config.local_port() == 0 ||
      config.local_port() > 65535 || config.cockpits().empty() ||
      config.heartbeat_interval_ms() == 0 || config.state_interval_ms() == 0) {
    return false;
  }

  // 解析驾驶舱白名单
  std::vector<sockaddr_in> endpoints;
  endpoints.reserve(config.cockpits_size());
  for (const auto &cockpit : config.cockpits()) {
    sockaddr_in endpoint{};
    if (!parseEndpoint(cockpit, endpoint)) return false;
    endpoints.push_back(endpoint);
  }

  // 保存运行参数
  vehicle_id_ = config.vehicle_id();
  local_port_ = static_cast<std::uint16_t>(config.local_port());
  heartbeat_interval_ =
      std::chrono::milliseconds(config.heartbeat_interval_ms());
  state_interval_ = std::chrono::milliseconds(config.state_interval_ms());
  cockpit_endpoints_ = std::move(endpoints);
  return true;
}

// 运行 UDP 工作循环
void RemoteDriveComponent::runLoop() {
  auto last_heartbeat = Clock::now() - heartbeat_interval_;
  auto last_state = Clock::now() - state_interval_;

  while (running_) {
    // 等待驾驶舱控制包
    pollfd descriptor{udp_channel_.fd(), POLLIN, 0};
    const int result = poll(&descriptor, 1, 100);
    if (result > 0 && (descriptor.revents & POLLIN)) {
      receiveControlPacket();
    }

    const auto now = Clock::now();

    // 驾驶舱断联时只发送一次退出请求
    if (const auto timeout_exit_command = session_.checkTimeout(now)) {
      control_writer_->Write(*timeout_exit_command);
    }

    // 周期发送车辆心跳
    if (now - last_heartbeat >= heartbeat_interval_) {
      sendHeartbeat();
      last_heartbeat = now;
    }

    // 周期发送车辆状态
    if (now - last_state >= state_interval_) {
      sendState();
      last_state = now;
    }
  }
}

// 校验并转发控制包
void RemoteDriveComponent::receiveControlPacket() {
  // 接收一个 UDP 数据报
  const auto datagram = udp_channel_.receive();
  if (!datagram) return;

  // 校验驾驶舱来源
  const bool known_cockpit =
      std::any_of(cockpit_endpoints_.begin(), cockpit_endpoints_.end(),
                  [&](const sockaddr_in &endpoint) {
                    return sameEndpoint(endpoint, datagram->source);
                  });
  if (!known_cockpit) return;

  // 解码控制包
  const auto packet = protocol_codec::decodePacket(datagram->payload.data(),
                                                   datagram->payload.size());
  if (!packet ||
      packet->body_case() != protocol::ProtocolPacket::kControl) {
    return;
  }

  // 通过会话校验后立即转发
  const auto controller = controllerId(datagram->source);
  const auto &control_command = packet->control();
  if (session_.accept(control_command, packet->sequence(), controller)) {
    control_writer_->Write(control_command);
  }
}

// 发送车辆心跳
void RemoteDriveComponent::sendHeartbeat() {
  // 编码并广播心跳
  const auto packet =
      protocol_codec::encodeHeartbeat(vehicle_id_, heartbeat_seq_++);
  if (packet.empty()) return;

  for (const auto &endpoint : cockpit_endpoints_) {
    udp_channel_.send(endpoint, packet.data(), packet.size());
  }
}

// 发送车辆状态
void RemoteDriveComponent::sendState() {
  // 编码并广播状态
  const protocol::ChassisState state = vehicleState();
  const auto packet =
      protocol_codec::encodeDrivingState(state, state_seq_++);
  if (packet.empty()) return;

  for (const auto &endpoint : cockpit_endpoints_) {
    udp_channel_.send(endpoint, packet.data(), packet.size());
  }
}

// 生成状态快照
protocol::ChassisState RemoteDriveComponent::vehicleState() const {
  // 复制最近一次底盘状态
  std::lock_guard<std::mutex> lock(state_mutex_);
  protocol::ChassisState state = latest_state_.value_or(protocol::ChassisState{});

  // 补充网关会话信息
  state.set_vehicle_id(vehicle_id_);
  state.set_controller_id(session_.controllerId());

  // 无底盘状态时默认驻车
  if (!latest_state_) {
    state.set_parking(true);
  }
  return state;
}

CYBER_REGISTER_COMPONENT(RemoteDriveComponent)

}  // namespace remote_drive::vehicle
