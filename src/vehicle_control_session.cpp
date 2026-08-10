#include "vehicle_control_session.h"

namespace {

namespace pb = remote_drive::protocol;

constexpr auto kRemoteControlTimeout = std::chrono::milliseconds(1500);

}  // namespace

// 校验并更新会话
bool VehicleControlSession::accept(
    const pb::RemoteDriveControlCommand &command, std::uint32_t sequence,
    ControllerId source, Clock::time_point now) {
  // 空闲时只允许 ENTER 建立会话
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

  // 只接受当前驾驶舱的递增序号
  if (source != *controller_ || command.cockpit_id() != controller_id_) {
    return false;
  }
  if (sequence <= last_sequence_) {
    return false;
  }

  // EXIT 转发后立即释放会话
  if (command.remote_mode() == pb::REMOTE_MODE_EXIT) {
    clear();
    return true;
  }

  last_sequence_ = sequence;
  last_control_time_ = now;
  return true;
}

// 检查会话超时
std::optional<pb::RemoteDriveControlCommand>
VehicleControlSession::checkTimeout(Clock::time_point now) {
  // 空闲或未超时时无需退出
  if (!active_) return std::nullopt;
  if (now - last_control_time_ < kRemoteControlTimeout) return std::nullopt;

  // 超时后只生成一次退出请求
  return leaveRemote();
}

// 主动结束会话
std::optional<pb::RemoteDriveControlCommand> VehicleControlSession::stop() {
  if (!active_) return std::nullopt;
  return leaveRemote();
}

// 生成退出请求
pb::RemoteDriveControlCommand VehicleControlSession::leaveRemote() {
  // 先生成退出请求，再清空会话
  pb::RemoteDriveControlCommand exit_command;
  exit_command.set_cockpit_id(controller_id_);
  exit_command.set_remote_mode(pb::REMOTE_MODE_EXIT);
  clear();
  return exit_command;
}

// 清空会话状态
void VehicleControlSession::clear() {
  active_ = false;
  controller_.reset();
  controller_id_.clear();
  last_sequence_ = 0;
  last_control_time_ = {};
}
