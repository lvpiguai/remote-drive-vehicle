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
#include "modules/remote_drive_vehicle/include/udp_channel.h"
#include "modules/remote_drive_vehicle/include/vehicle_control_session.h"
#include "modules/remote_drive_vehicle/proto/remote_drive.pb.h"

namespace remote_drive::vehicle {

// 车端 UDP/Cyber 网关
class RemoteDriveComponent final
    : public apollo::cyber::Component<protocol::ChassisState> {
 public:
  ~RemoteDriveComponent() override;

  // 初始化组件
  bool Init() override;

  // 缓存底盘状态
  bool Proc(const std::shared_ptr<protocol::ChassisState> &state) override;

 private:
  using Clock = VehicleControlSession::Clock;

  // 加载车端配置
  bool loadConfig();

  // 运行 UDP 工作循环
  void runLoop();

  // 校验并转发控制包
  void receiveControlPacket();

  // 发送车辆状态
  void sendState();

  // 生成状态快照
  protocol::ChassisState vehicleState() const;

  // 底盘控制 Writer
  std::shared_ptr<apollo::cyber::Writer<protocol::RemoteDriveControlCommand>>
      control_writer_;

  // 当前车辆的静态部署配置
  std::string vehicle_id_;
  std::uint16_t local_port_ = 0;
  std::chrono::milliseconds state_interval_{20};
  std::vector<sockaddr_in> cockpit_addresses_;

  // UDP 工作线程状态
  UdpChannel udp_channel_;
  VehicleControlSession session_;
  std::thread udp_worker_;
  std::atomic<bool> running_{false};

  // 底盘状态缓存
  mutable std::mutex state_mutex_;
  std::optional<protocol::ChassisState> latest_state_;

  // UDP 发送序号
  std::uint32_t state_seq_ = 1;
};

}  // namespace remote_drive::vehicle
