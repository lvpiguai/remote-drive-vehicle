#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "protocol/remote_control_protocol.h"
#include "vehicle/chassis_gateway.h"

// Cyber RT 底盘通信适配器的本地占位实现
class CyberChassisGateway : public ChassisGateway {
 public:
  explicit CyberChassisGateway(std::string vehicle_id);

  bool publishControl(const RemoteCtlCmd &command,
                      std::uint32_t sequence) override;
  std::optional<RemoteDrivingState> latestState() const override;

  // Apollo Reader 收到底盘状态后更新缓存
  void updateState(const RemoteDrivingState &state);

 private:
  std::string vehicle_id_;
  std::optional<RemoteDrivingState> latest_state_;
};
