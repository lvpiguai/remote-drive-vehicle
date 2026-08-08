#pragma once

#include <chrono>
#include <cstdint>

#include "protocol/remote_control_protocol.h"
#include "simulation/chassis_simulator.h"

// 车端从接受 REMOTE_ENTER 到退出远控的一次控制会话
class VehicleControlSession {
 public:
  using Clock = std::chrono::steady_clock;
  using ControllerId = std::uint64_t;

  enum class Result {
    ACCEPTED,
    STALE_SEQUENCE,
    CONTROLLER_BUSY,
  };

  struct Outcome {
    Result result = Result::ACCEPTED;
    bool applied = false;
    bool ended = false;
  };

  VehicleControlSession(ChassisSimulator &chassis, ControllerId controller);
  ~VehicleControlSession();

  // 校验并处理已经解码的控制指令
  Outcome handleCommand(const RemoteCtlCmd &command, std::uint32_t sequence,
                        ControllerId source,
                        Clock::time_point now = Clock::now());

  // 检查控制超时；会话结束时返回 false
  bool tick(Clock::time_point now = Clock::now());

  ControllerId controller() const { return controller_; }

 private:
  bool shouldLogControl(const RemoteCtlCmd &command, Result result,
                        bool applied);
  void leaveRemote(bool emergency_stop);

  ChassisSimulator &chassis_;
  ControllerId controller_;
  RemoteCtlCmd latest_command_{};
  std::uint32_t last_sequence_ = 0;
  Clock::time_point last_control_time_{};
  RemoteCtlCmd last_logged_command_{};
  Result last_logged_result_ = Result::ACCEPTED;
  bool last_logged_applied_ = false;
  bool has_logged_control_ = false;
  bool active_ = true;
};
