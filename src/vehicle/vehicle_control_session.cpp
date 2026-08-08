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

// 生成退出远控时发布到底盘的安全控制指令
RemoteCtlCmd safeExitCommand(bool emergency_stop) {
  RemoteCtlCmd command{};
  command.remoteMode = RemoteMode::REMOTE_EXIT;
  command.acc_pedal = 0;
  command.brake_pedal = 100;
  command.gear = GearInfo::NEUTRAL;
  command.parking = SwitchCommand::ON;
  command.remote_emergency =
      emergency_stop ? SwitchCommand::ON : SwitchCommand::NO_CTL;
  return command;
}

}  // namespace

// 创建车端远控会话
VehicleControlSession::VehicleControlSession(ChassisGateway &chassis_gateway,
                                             ControllerId controller)
    : chassis_gateway_(chassis_gateway), controller_(controller) {}

// 异常销毁活动会话时执行安全退出
VehicleControlSession::~VehicleControlSession() {
  if (active_) leaveRemote(true);
}

// 校验来源和序号后将控制指令发布到底盘网关
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

  if (!chassis_gateway_.publishControl(command, sequence)) {
    return finish(Result::ACCEPTED, false);
  }
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

// 发布安全退出控制并结束远控会话
void VehicleControlSession::leaveRemote(bool emergency_stop) {
  if (!active_) return;
  latest_command_ = safeExitCommand(emergency_stop);
  chassis_gateway_.publishControl(latest_command_, last_sequence_ + 1);
  active_ = false;
}
