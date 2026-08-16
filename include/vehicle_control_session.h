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
  using ControllerSource = std::uint64_t;

  // 校验并接收控制命令
  bool acceptControlCommand(
      const remote_drive::protocol::ControlCommand &command,
      std::uint32_t sequence, ControllerSource source,
      Clock::time_point now = Clock::now());

  // 当前控制会话是否超时
  bool controlTimedOut(Clock::time_point now = Clock::now()) const;

  // 主动结束会话
  std::optional<remote_drive::protocol::ControlCommand>
  stopRemoteControl();

  const std::string &cockpitId() const { return cockpit_id_; }

 private:
  // 清空状态
  void reset();

  std::optional<ControllerSource> controller_source_;
  std::string cockpit_id_;
  std::uint32_t last_sequence_ = 0;
  Clock::time_point last_control_time_{};
  bool has_controller_ = false;
};
