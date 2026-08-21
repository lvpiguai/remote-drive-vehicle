#include "modules/remote_drive_vehicle/include/remote_drive_component.h"

#include <arpa/inet.h>
#include <poll.h>

#include <chrono>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "cyber/component/component.h"
#include "modules/remote_drive_vehicle/include/protocol_codec.h"
#include "modules/remote_drive_vehicle/proto/remote_drive_vehicle_config.pb.h"

namespace remote_drive::vehicle {

namespace {

constexpr std::size_t kMaxIdLength = 19;
constexpr std::uint16_t kCockpitUdpPort = 7005;
constexpr std::uint16_t kVehicleUdpPort = 7006;
constexpr auto kStateInterval = std::chrono::milliseconds(20);
constexpr auto kVehicleStateTimeout = std::chrono::milliseconds(500);

// 底盘控制 Channel
constexpr char kControlChannel[] = "/remote_drive/control_cmd";

// 解析驾驶舱端点
bool parseCockpitAddress(const ConfiguredCockpit &config,
                         sockaddr_in &address) {
  address = {};
  address.sin_family = AF_INET;
  if (inet_pton(AF_INET, config.ip().c_str(), &address.sin_addr) != 1) {
    return false;
  }
  address.sin_port = htons(kCockpitUdpPort);
  return true;
}

// 校验协议标识字段
bool validId(const std::string &id) {
  return !id.empty() && id.size() <= kMaxIdLength &&
         id.find_first_of("\\\"") == std::string::npos;
}

// 比较 UDP 端点
bool sameAddress(const sockaddr_in &left, const sockaddr_in &right) {
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
    if (const auto exit_command = session_.stopRemoteControl()) {
      control_writer_->Write(*exit_command);
    }
  }
}

// 初始化组件
bool RemoteDriveComponent::Init() {
  // 加载配置并绑定 UDP 端口
  if (!loadConfig()) return false;
  if (!udp_channel_.bindPort(kVehicleUdpPort)) return false;

  // 创建底盘控制 Writer
  control_writer_ = node_->CreateWriter<protocol::ControlCommand>(
      kControlChannel);
  if (!control_writer_) return false;

  // 启动 UDP 工作线程
  running_ = true;
  udp_worker_ =
      std::thread(&RemoteDriveComponent::runRemoteControlLoop, this);
  return true;
}

// 缓存车辆状态
bool RemoteDriveComponent::Proc(
    const std::shared_ptr<protocol::VehicleState> &state) {
  if (!state) return false;

  // 更新共享状态缓存
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_state_ = *state;
  last_state_receive_time_ = Clock::now();
  return true;
}

// 加载车端配置
bool RemoteDriveComponent::loadConfig() {
  // 读取并校验基础字段
  RemoteDriveVehicleConfig config;
  if (!GetProtoConfig(&config) || !config.IsInitialized()) return false;

  if (!validId(config.vehicle_id()) || config.cockpits().empty()) {
    return false;
  }

  // 解析驾驶舱白名单
  std::unordered_map<std::string, sockaddr_in> cockpit_addresses;
  std::unordered_set<std::uint32_t> cockpit_ips;
  cockpit_addresses.reserve(config.cockpits_size());
  for (const auto &cockpit : config.cockpits()) {
    sockaddr_in cockpit_address{};
    if (!validId(cockpit.cockpit_id()) ||
        !parseCockpitAddress(cockpit, cockpit_address) ||
        !cockpit_addresses.emplace(cockpit.cockpit_id(), cockpit_address)
             .second ||
        !cockpit_ips.insert(cockpit_address.sin_addr.s_addr).second) {
      return false;
    }
  }

  // 保存运行参数
  vehicle_id_ = config.vehicle_id();
  cockpit_addresses_ = std::move(cockpit_addresses);
  return true;
}

// 运行远控通信循环
void RemoteDriveComponent::runRemoteControlLoop() {
  auto last_state_send_time = Clock::now() - kStateInterval;

  while (running_) {
    // 等待驾驶舱控制包
    pollfd descriptor{udp_channel_.fd(), POLLIN, 0};
    const int result =
        poll(&descriptor, 1, static_cast<int>(kStateInterval.count()));
    const auto now = Clock::now();
    const bool vehicle_state_fresh = hasFreshVehicleState(now);

    if (result > 0 && (descriptor.revents & POLLIN)) {
      receiveControlPacket(vehicle_state_fresh);
    }

    // 驾驶舱断联或车辆状态超时时只发送一次退出请求
    if (!vehicle_state_fresh || session_.controlTimedOut(now)) {
      if (const auto exit_command = session_.stopRemoteControl()) {
        control_writer_->Write(*exit_command);
      }
    }

    // 周期发送车辆状态
    if (vehicle_state_fresh &&
        now - last_state_send_time >= kStateInterval) {
      sendState();
      last_state_send_time = now;
    }
  }
}

// 判断最近一次 Cyber RT 车辆状态是否超时
bool RemoteDriveComponent::hasFreshVehicleState(Clock::time_point now) const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_state_ && last_state_receive_time_ &&
         now - *last_state_receive_time_ < kVehicleStateTimeout;
}

// 校验并转发控制包
void RemoteDriveComponent::receiveControlPacket(bool vehicle_state_fresh) {
  // 接收一个 UDP 数据报
  const auto datagram = udp_channel_.receive();
  if (!datagram) return;

  // 车辆状态不可用时丢弃控制包，避免基于旧状态继续远控
  if (!vehicle_state_fresh) return;

  // 解码控制包
  const auto packet = protocol_codec::decodePacket(
      datagram->payload.data(), datagram->payload.size());
  if (!packet ||
      packet->body_case() != protocol::ProtocolPacket::kControl) {
    return;
  }

  // 校验驾驶舱身份与来源映射
  const auto &control_command = packet->control();
  const auto configured = cockpit_addresses_.find(control_command.cockpit_id());
  if (configured == cockpit_addresses_.end() ||
      !sameAddress(configured->second, datagram->source)) {
    return;
  }

  // 通过会话校验后立即转发
  if (session_.acceptControlCommand(control_command, packet->sequence())) {
    control_writer_->Write(control_command);
  }
}

// 发送车辆状态
void RemoteDriveComponent::sendState() {
  const auto state = vehicleState();
  if (!state) return;

  // 编码并广播状态
  const auto packet =
      protocol_codec::encodeVehicleState(*state, state_seq_++);
  if (packet.empty()) return;

  for (const auto &cockpit : cockpit_addresses_) {
    udp_channel_.send(cockpit.second, packet.data(), packet.size());
  }
}

// 获取当前车辆状态
std::optional<protocol::VehicleState>
RemoteDriveComponent::vehicleState() const {
  protocol::VehicleState state;
  {
    // 复制最近一次车辆状态
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!latest_state_) return std::nullopt;
    state = *latest_state_;
  }

  // 补充网关会话信息
  state.set_vehicle_id(vehicle_id_);
  state.set_cockpit_id(session_.cockpitId());
  return state;
}

CYBER_REGISTER_COMPONENT(RemoteDriveComponent)

}  // namespace remote_drive::vehicle
