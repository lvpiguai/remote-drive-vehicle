#include "vehicle/vehicle_control_session.h"

#include <cmath>
#include <cstddef>

namespace {

namespace pb = remote_drive::protocol;

constexpr auto kRemoteControlTimeout = std::chrono::milliseconds(1500);
constexpr std::size_t kMaxCockpitIdLength = 19;

bool validSwitch(pb::SwitchCommand command) {
  return pb::SwitchCommand_IsValid(command);
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

// 校验来源和序号后判断控制指令是否允许转发
bool VehicleControlSession::accept(
    const pb::RemoteDriveControlCommand &command, std::uint32_t sequence,
    ControllerId source, Clock::time_point now) {
  if (!isValidCommand(command)) {
    return false;
  }

  if (!active_) {
    if (command.remote_mode() != pb::REMOTE_MODE_ENTER || sequence == 0) {
      return false;
    }
    controller_ = source;
    controller_id_ = command.cockpit_id();
    last_sequence_ = 0;
    last_control_time_ = now;
    active_ = true;
  }

  if (source != *controller_ || command.cockpit_id() != controller_id_) {
    return false;
  }
  if (sequence <= last_sequence_) {
    return false;
  }

  if (command.remote_mode() == pb::REMOTE_MODE_EXIT) {
    leaveRemote(false);
    return true;
  }

  last_sequence_ = sequence;
  last_control_time_ = now;
  return true;
}

// 检查控制指令是否超时
std::optional<pb::RemoteDriveControlCommand> VehicleControlSession::tick(
    Clock::time_point now) {
  if (!active_) return std::nullopt;
  if (now - last_control_time_ < kRemoteControlTimeout) return std::nullopt;
  return leaveRemote(true);
}

std::optional<pb::RemoteDriveControlCommand> VehicleControlSession::stop(
    bool emergency_stop) {
  if (!active_) return std::nullopt;
  return leaveRemote(emergency_stop);
}

// 生成安全退出控制并结束远控会话
pb::RemoteDriveControlCommand VehicleControlSession::leaveRemote(
    bool emergency_stop) {
  const auto command = safeExitCommand(emergency_stop);
  active_ = false;
  controller_.reset();
  controller_id_.clear();
  return command;
}
