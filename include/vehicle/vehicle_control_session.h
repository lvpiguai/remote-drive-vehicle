#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "remote_drive.pb.h"

// 车端从接受 REMOTE_ENTER 到退出远控的一次控制会话
class VehicleControlSession {
 public:
  using Clock = std::chrono::steady_clock;
  using ControllerId = std::uint64_t;

  // 判断一条控制指令是否允许转发到底盘控制通道
  bool accept(
      const remote_drive::protocol::RemoteDriveControlCommand &command,
      std::uint32_t sequence, ControllerId source,
      Clock::time_point now = Clock::now());

  // 检查控制超时；超时时返回一条需要转发到底盘的安全退出指令
  std::optional<remote_drive::protocol::RemoteDriveControlCommand> tick(
      Clock::time_point now = Clock::now());

  // 主动结束当前远控会话；会话活跃时返回需要转发的安全退出指令
  std::optional<remote_drive::protocol::RemoteDriveControlCommand> stop(
      bool emergency_stop);

  const std::string &controllerId() const { return controller_id_; }

 private:
  static bool isValidCommand(
      const remote_drive::protocol::RemoteDriveControlCommand &command);
  remote_drive::protocol::RemoteDriveControlCommand leaveRemote(
      bool emergency_stop);

  std::optional<ControllerId> controller_;
  std::string controller_id_;
  std::uint32_t last_sequence_ = 0;
  Clock::time_point last_control_time_{};
  bool active_ = false;
};
