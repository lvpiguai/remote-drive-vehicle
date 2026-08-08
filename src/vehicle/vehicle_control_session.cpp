#include "vehicle/vehicle_control_session.h"

#include <cstring>
#include <iostream>

namespace {

constexpr auto kRemoteControlTimeout = std::chrono::milliseconds(1500);

// 将挡位转换为日志文本
const char *gearName(GearInfo gear) {
  switch (gear) {
    case GearInfo::NEUTRAL:
      return "N";
    case GearInfo::REVERSE_1:
      return "R1";
    case GearInfo::REVERSE_2:
      return "R2";
    case GearInfo::DRIVE_1:
      return "D1";
    case GearInfo::DRIVE_2:
      return "D2";
    case GearInfo::DRIVE_3:
      return "D3";
  }
  return "UNKNOWN";
}

// 将远控模式指令转换为日志文本
const char *remoteModeName(RemoteMode mode) {
  switch (mode) {
    case RemoteMode::REMOTE_NO_CONTROL:
      return "NONE";
    case RemoteMode::REMOTE_ENTER:
      return "ENTER";
    case RemoteMode::REMOTE_EXIT:
      return "EXIT";
  }
  return "UNKNOWN";
}

// 将三态开关指令转换为日志文本
const char *switchCommandName(SwitchCommand command) {
  switch (command) {
    case SwitchCommand::NO_CTL:
      return "NO_CTL";
    case SwitchCommand::OFF:
      return "OFF";
    case SwitchCommand::ON:
      return "ON";
  }
  return "UNKNOWN";
}

// 将控制处理结果转换为日志文本
const char *resultName(VehicleControlSession::Result result) {
  switch (result) {
    case VehicleControlSession::Result::ACCEPTED:
      return "ACCEPTED";
    case VehicleControlSession::Result::STALE_SEQUENCE:
      return "STALE_SEQUENCE";
    case VehicleControlSession::Result::CONTROLLER_BUSY:
      return "CONTROLLER_BUSY";
  }
  return "UNKNOWN";
}

// 输出车端控制指令处理摘要
void logReceivedControl(const RemoteCtlCmd &command, std::uint32_t sequence,
                        VehicleControlSession::Result result, bool applied) {
  std::cout << "[控制接收] seq=" << sequence
            << " steering=" << command.steering_angle
            << " acc=" << command.acc_pedal
            << " brake=" << command.brake_pedal
            << " gear=" << gearName(command.gear)
            << " remote=" << remoteModeName(command.remoteMode)
            << " parking=" << switchCommandName(command.parking)
            << " emergency=" << switchCommandName(command.remote_emergency)
            << " result=" << resultName(result) << " applied=" << applied
            << '\n';
}

}  // namespace

// 创建车端会话并让底盘进入远控模式
VehicleControlSession::VehicleControlSession(ChassisSimulator &chassis,
                                             ControllerId controller)
    : chassis_(chassis), controller_(controller) {
  chassis_.enterRemote();
}

// 异常销毁活动会话时执行安全退出
VehicleControlSession::~VehicleControlSession() {
  if (active_) leaveRemote(true);
}

// 校验来源和序号后将控制指令应用到底盘
VehicleControlSession::Outcome VehicleControlSession::handleCommand(
    const RemoteCtlCmd &command, std::uint32_t sequence, ControllerId source,
    Clock::time_point now) {
  const auto finish = [&](Result result, bool applied, bool ended = false) {
    if (shouldLogControl(command, result, applied)) {
      logReceivedControl(command, sequence, result, applied);
    }
    return Outcome{result, applied, ended};
  };

  if (source != controller_) {
    return finish(Result::CONTROLLER_BUSY, false);
  }
  if (sequence <= last_sequence_) {
    return finish(Result::STALE_SEQUENCE, false);
  }

  if (command.remoteMode == RemoteMode::REMOTE_EXIT) {
    latest_command_ = command;
    leaveRemote(false);
    return finish(Result::ACCEPTED, true, true);
  }

  chassis_.applyCommand(command);
  latest_command_ = command;
  last_sequence_ = sequence;
  last_control_time_ = now;
  return finish(Result::ACCEPTED, true);
}

// 判断控制内容或处理结果是否发生变化
bool VehicleControlSession::shouldLogControl(const RemoteCtlCmd &command,
                                             Result result, bool applied) {
  const bool command_changed =
      !has_logged_control_ ||
      std::memcmp(&command, &last_logged_command_, sizeof(command)) != 0;
  const bool outcome_changed = !has_logged_control_ ||
                               result != last_logged_result_ ||
                               applied != last_logged_applied_;
  if (!command_changed && !outcome_changed) return false;

  last_logged_command_ = command;
  last_logged_result_ = result;
  last_logged_applied_ = applied;
  has_logged_control_ = true;
  return true;
}

// 检查控制指令是否超时
bool VehicleControlSession::tick(Clock::time_point now) {
  if (now - last_control_time_ < kRemoteControlTimeout) return true;
  leaveRemote(true);
  return false;
}

// 应用安全覆盖并让底盘退出远控模式
void VehicleControlSession::leaveRemote(bool emergency_stop) {
  if (!active_) return;
  latest_command_.remoteMode = RemoteMode::REMOTE_EXIT;
  latest_command_.acc_pedal = 0;
  latest_command_.brake_pedal = 100;
  latest_command_.gear = GearInfo::NEUTRAL;
  latest_command_.parking = SwitchCommand::ON;
  latest_command_.remote_emergency =
      emergency_stop ? SwitchCommand::ON : SwitchCommand::NO_CTL;
  chassis_.applyCommand(latest_command_);
  chassis_.exitRemote();
  active_ = false;
}
