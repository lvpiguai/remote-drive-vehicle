#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "remote_drive.pb.h"
#include "vehicle/chassis_gateway.h"

// 车端从接受 REMOTE_ENTER 到退出远控的一次控制会话
class VehicleControlSession {
 public:
  using Clock = std::chrono::steady_clock;
  using ControllerId = std::uint64_t;

  enum class Result {
    ACCEPTED,
    INVALID_COMMAND,
    STALE_SEQUENCE,
    CONTROLLER_BUSY,
  };

  struct Outcome {
    Result result = Result::ACCEPTED;
    bool applied = false;
    bool ended = false;
  };

  explicit VehicleControlSession(ChassisGateway &chassis_gateway);
  ~VehicleControlSession();

  // 校验并处理已经解码的控制指令
  Outcome handleCommand(
      const remote_drive::protocol::RemoteDriveControlCommand &command,
      std::uint32_t sequence, ControllerId source,
      Clock::time_point now = Clock::now());

  // 检查控制超时；会话结束时返回 false
  bool tick(Clock::time_point now = Clock::now());

  const std::string &controllerId() const { return controller_id_; }

 private:
  static bool isValidCommand(
      const remote_drive::protocol::RemoteDriveControlCommand &command);
  bool shouldLogControl(
      const remote_drive::protocol::RemoteDriveControlCommand &command,
      Result result, bool applied);
  void leaveRemote(bool emergency_stop);

  ChassisGateway &chassis_gateway_;
  std::optional<ControllerId> controller_;
  std::string controller_id_;
  remote_drive::protocol::RemoteDriveControlCommand latest_command_{};
  std::uint32_t last_sequence_ = 0;
  Clock::time_point last_control_time_{};
  remote_drive::protocol::RemoteDriveControlCommand last_logged_command_{};
  Result last_logged_result_ = Result::ACCEPTED;
  bool last_logged_applied_ = false;
  bool has_logged_control_ = false;
  bool active_ = false;
};
