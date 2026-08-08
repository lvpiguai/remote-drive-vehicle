#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "remote_drive.pb.h"
#include "vehicle/chassis_gateway.h"

// Cyber RT 底盘通信适配器的本地占位实现
class CyberChassisGateway : public ChassisGateway {
 public:
  explicit CyberChassisGateway(std::string vehicle_id);

  bool publishControl(
      const remote_drive::protocol::RemoteDriveControlCommand &command,
      std::uint32_t sequence) override;
  std::optional<remote_drive::protocol::ChassisState> latestState() const override;

  // Apollo Reader 收到底盘状态后更新缓存
  void updateState(const remote_drive::protocol::ChassisState &state);

 private:
  std::string vehicle_id_;
  std::optional<remote_drive::protocol::ChassisState> latest_state_;
};
