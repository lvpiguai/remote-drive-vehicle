#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "remote_drive.pb.h"

// 驾驶舱远控会话
class VehicleControlSession {
 public:
  using Clock = std::chrono::steady_clock;
  using ControllerId = std::uint64_t;

  // 校验并接收控制命令
  bool accept(
      const remote_drive::protocol::RemoteDriveControlCommand &command,
      std::uint32_t sequence, ControllerId source,
      Clock::time_point now = Clock::now());

  // 检查会话超时
  std::optional<remote_drive::protocol::RemoteDriveControlCommand> checkTimeout(
      Clock::time_point now = Clock::now());

  // 主动结束会话
  std::optional<remote_drive::protocol::RemoteDriveControlCommand> stop();

 const std::string &controllerId() const { return controller_id_; }

 private:
  // 生成退出请求
  remote_drive::protocol::RemoteDriveControlCommand leaveRemote();

  // 清空会话状态
  void clear();

  std::optional<ControllerId> controller_;
  std::string controller_id_;
  std::uint32_t last_sequence_ = 0;
  Clock::time_point last_control_time_{};
  bool active_ = false;
};
