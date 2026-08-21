#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "modules/remote_drive_vehicle/include/udp_channel.h"
#include "modules/remote_drive_vehicle/include/vehicle_control_session.h"
#include "modules/remote_drive_vehicle/proto/remote_drive.pb.h"

namespace remote_drive::vehicle {

// 车端 UDP/Cyber 网关
class RemoteDriveComponent final
    : public apollo::cyber::Component<protocol::VehicleState> {
 public:
  ~RemoteDriveComponent() override;

  // 初始化组件
  bool Init() override;

  // 缓存车辆状态
  bool Proc(const std::shared_ptr<protocol::VehicleState> &state) override;

 private:
  using Clock = VehicleControlSession::Clock;

  // 加载车端配置
  bool loadConfig();

  // 运行远控通信循环
  void runRemoteControlLoop();

  // 判断 Cyber RT 车辆状态是否仍然有效
  bool hasFreshVehicleState(Clock::time_point now) const;

  // 校验并转发控制包
  void receiveControlPacket(bool vehicle_state_fresh);

  // 发送车辆状态
  void sendState();

  // 获取当前车辆状态
  std::optional<protocol::VehicleState> vehicleState() const;

  // 底盘控制 Writer
  std::shared_ptr<apollo::cyber::Writer<protocol::ControlCommand>>
      control_writer_;

  // 当前车辆的静态部署配置
  std::string vehicle_id_;
  std::unordered_map<std::string, sockaddr_in> cockpit_addresses_;

  // UDP 工作线程状态
  UdpChannel udp_channel_;
  VehicleControlSession session_;
  std::thread udp_worker_;
  std::atomic<bool> running_{false};

  // 车辆状态缓存
  mutable std::mutex state_mutex_;
  std::optional<protocol::VehicleState> latest_state_;
  std::optional<Clock::time_point> last_state_receive_time_;

  // UDP 发送序号
  std::uint32_t state_seq_ = 1;
};

}  // namespace remote_drive::vehicle
