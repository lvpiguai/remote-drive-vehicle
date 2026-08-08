#include "vehicle/vehicle_control_session.h"

#include <cmath>
#include <cstddef>
#include <iostream>

namespace {

namespace pb = remote_drive::protocol;

constexpr auto kRemoteControlTimeout = std::chrono::milliseconds(1500);
constexpr std::size_t kMaxCockpitIdLength = 19;

bool validSwitch(pb::SwitchCommand command) {
  return pb::SwitchCommand_IsValid(command);
}

// 将挡位转换为日志文本
const char *gearName(pb::Gear gear) {
  switch (gear) {
    case pb::GEAR_NEUTRAL:
      return "N";
    case pb::GEAR_REVERSE_1:
      return "R1";
    case pb::GEAR_REVERSE_2:
      return "R2";
    case pb::GEAR_DRIVE_1:
      return "D1";
    case pb::GEAR_DRIVE_2:
      return "D2";
    case pb::GEAR_DRIVE_3:
      return "D3";
    default:
      break;
  }
  return "UNKNOWN";
}

// 将远控模式指令转换为日志文本
const char *remoteModeName(pb::RemoteMode mode) {
  switch (mode) {
    case pb::REMOTE_MODE_NO_CONTROL:
      return "NONE";
    case pb::REMOTE_MODE_ENTER:
      return "ENTER";
    case pb::REMOTE_MODE_EXIT:
      return "EXIT";
    default:
      break;
  }
  return "UNKNOWN";
}

// 将三态开关指令转换为日志文本
const char *switchCommandName(pb::SwitchCommand command) {
  switch (command) {
    case pb::SWITCH_NO_CONTROL:
      return "NO_CTL";
    case pb::SWITCH_OFF:
      return "OFF";
    case pb::SWITCH_ON:
      return "ON";
    default:
      break;
  }
  return "UNKNOWN";
}

// 将控制处理结果转换为日志文本
const char *resultName(VehicleControlSession::Result result) {
  switch (result) {
    case VehicleControlSession::Result::ACCEPTED:
      return "ACCEPTED";
    case VehicleControlSession::Result::INVALID_COMMAND:
      return "INVALID_COMMAND";
    case VehicleControlSession::Result::STALE_SEQUENCE:
      return "STALE_SEQUENCE";
    case VehicleControlSession::Result::CONTROLLER_BUSY:
      return "CONTROLLER_BUSY";
  }
  return "UNKNOWN";
}

// 输出车端控制指令处理摘要
void logReceivedControl(const pb::RemoteDriveControlCommand &command,
                        std::uint32_t sequence,
                        VehicleControlSession::Result result, bool applied) {
  std::cout << "[控制接收] seq=" << sequence
            << " steering=" << command.steering_angle()
            << " acc=" << command.accelerator_percent()
            << " brake=" << command.brake_percent()
            << " gear=" << gearName(command.gear())
            << " remote=" << remoteModeName(command.remote_mode())
            << " parking=" << switchCommandName(command.parking())
            << " emergency=" << switchCommandName(command.remote_emergency())
            << " result=" << resultName(result) << " applied=" << applied
            << '\n';
}

// 生成退出远控时发布到底盘的安全控制指令
pb::RemoteDriveControlCommand safeExitCommand(bool emergency_stop) {
  pb::RemoteDriveControlCommand command;
  command.set_remote_mode(pb::REMOTE_MODE_EXIT);
  command.set_accelerator_percent(0);
  command.set_brake_percent(100);
  command.set_gear(pb::GEAR_NEUTRAL);
  command.set_parking(pb::SWITCH_ON);
  command.set_remote_emergency(
      emergency_stop ? pb::SWITCH_ON : pb::SWITCH_NO_CONTROL);
  return command;
}

}  // namespace

// 创建车端远控会话
VehicleControlSession::VehicleControlSession(ChassisGateway &chassis_gateway)
    : chassis_gateway_(chassis_gateway) {}

// 异常销毁活动会话时执行安全退出
VehicleControlSession::~VehicleControlSession() {
  if (active_) leaveRemote(true);
}

bool VehicleControlSession::isValidCommand(
    const pb::RemoteDriveControlCommand &command) {
  return !command.cockpit_id().empty() &&
         command.cockpit_id().size() <= kMaxCockpitIdLength &&
         std::isfinite(command.steering_angle()) &&
         std::isfinite(command.accelerator_percent()) &&
         std::isfinite(command.brake_percent()) &&
         command.steering_angle() >= -90.0 &&
         command.steering_angle() <= 90.0 &&
         command.accelerator_percent() >= 0.0 &&
         command.accelerator_percent() <= 100.0 &&
         command.brake_percent() >= 0.0 &&
         command.brake_percent() <= 100.0 &&
         pb::Gear_IsValid(command.gear()) &&
         pb::Bucket_IsValid(command.bucket()) &&
         pb::RemoteMode_IsValid(command.remote_mode()) &&
         validSwitch(command.parking()) && validSwitch(command.horn()) &&
         validSwitch(command.spray()) &&
         validSwitch(command.remote_emergency()) &&
         validSwitch(command.window_wiper()) &&
         validSwitch(command.light_brake()) &&
         validSwitch(command.light_position()) &&
         validSwitch(command.light_near()) &&
         validSwitch(command.light_far()) &&
         validSwitch(command.light_turn_left()) &&
         validSwitch(command.light_turn_right()) &&
         validSwitch(command.light_working_rear()) &&
         validSwitch(command.light_danger()) &&
         validSwitch(command.light_reverse()) &&
         validSwitch(command.light_double_flash()) &&
         validSwitch(command.light_front()) &&
         validSwitch(command.light_working_side()) &&
         validSwitch(command.light_fog()) && validSwitch(command.diff_lock());
}

// 校验来源和序号后将控制指令发布到底盘网关
VehicleControlSession::Outcome VehicleControlSession::handleCommand(
    const pb::RemoteDriveControlCommand &command, std::uint32_t sequence,
    ControllerId source, Clock::time_point now) {
  const auto finish = [&](Result result, bool applied, bool ended = false) {
    if (shouldLogControl(command, result, applied)) {
      logReceivedControl(command, sequence, result, applied);
    }
    return Outcome{result, applied, ended};
  };

  if (!isValidCommand(command)) {
    return finish(Result::INVALID_COMMAND, false);
  }

  if (!active_) {
    if (command.remote_mode() != pb::REMOTE_MODE_ENTER || sequence == 0) {
      return finish(Result::INVALID_COMMAND, false);
    }
    controller_ = source;
    controller_id_ = command.cockpit_id();
    last_sequence_ = 0;
    last_control_time_ = now;
    active_ = true;
  }

  if (source != *controller_ || command.cockpit_id() != controller_id_) {
    return finish(Result::CONTROLLER_BUSY, false);
  }
  if (sequence <= last_sequence_) {
    return finish(Result::STALE_SEQUENCE, false);
  }

  if (command.remote_mode() == pb::REMOTE_MODE_EXIT) {
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
bool VehicleControlSession::shouldLogControl(
    const pb::RemoteDriveControlCommand &command, Result result, bool applied) {
  const bool command_changed =
      !has_logged_control_ ||
      command.SerializeAsString() != last_logged_command_.SerializeAsString();
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
  if (!active_) return true;
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
  controller_.reset();
  controller_id_.clear();
}
