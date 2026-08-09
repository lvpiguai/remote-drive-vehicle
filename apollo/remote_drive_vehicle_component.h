#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "modules/remote_drive_vehicle/include/network/udp_channel.h"
#include "modules/remote_drive_vehicle/include/vehicle/vehicle_control_session.h"
#include "modules/remote_drive_vehicle/proto/remote_drive.pb.h"

namespace remote_drive::vehicle {

// 车端 Cyber RT 组件：连接驾驶舱 UDP 通信与底盘控制、状态 Channel
class RemoteDriveVehicleComponent final
    : public apollo::cyber::Component<protocol::ChassisState> {
 public:
  ~RemoteDriveVehicleComponent() override;

  // 加载车端配置、创建控制 Writer 并启动 UDP 工作线程
  bool Init() override;

  // 接收底盘状态并缓存，供 UDP 工作线程回传驾驶舱
  bool Proc(const std::shared_ptr<protocol::ChassisState> &state) override;

 private:
  using Clock = VehicleControlSession::Clock;

  // 从 DAG 指定的 pb.txt 加载并校验当前车辆配置
  bool loadConfig();

  // 持续处理控制包、会话超时、心跳和车辆状态发送
  void runLoop();

  // 接收并过滤一个驾驶舱控制包，允许后写入底盘控制 Channel
  void receiveControlPacket();

  // 向配置中的全部驾驶舱发送在线心跳
  void sendHeartbeat();

  // 向配置中的全部驾驶舱发送最近一次底盘状态
  void sendState();

  // 生成带有当前车辆和控制者标识的状态快照
  protocol::ChassisState vehicleState() const;

  // 更新由 Cyber Reader 接收到的底盘状态缓存
  void onChassisState(const std::shared_ptr<protocol::ChassisState> &state);

  // 使用 Cyber Component 基类提供的 node_ 向底盘控制 Channel 发布指令
  std::shared_ptr<apollo::cyber::Writer<protocol::RemoteDriveControlCommand>>
      control_writer_;

  // 当前车辆的静态部署配置
  std::string vehicle_id_;
  std::string control_channel_;
  std::uint16_t local_port_ = 0;
  std::chrono::milliseconds heartbeat_interval_{1000};
  std::chrono::milliseconds state_interval_{100};
  std::vector<sockaddr_in> cockpit_endpoints_;

  // UDP 收发和远控会话只在 udp_worker_ 中处理
  UdpChannel udp_channel_;
  VehicleControlSession session_;
  std::thread udp_worker_;
  std::atomic<bool> running_{false};

  // Proc() 写入、UDP 工作线程读取，使用互斥锁保护状态快照
  mutable std::mutex state_mutex_;
  std::optional<protocol::ChassisState> latest_state_;

  // 心跳包和状态包分别维护独立递增序号
  std::uint32_t heartbeat_seq_ = 1;
  std::uint32_t state_seq_ = 1;
};

}  // namespace remote_drive::vehicle
