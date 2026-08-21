#include "vehicle_control_session.h"

namespace {

namespace pb = remote_drive::protocol;

constexpr auto kRemoteControlTimeout = std::chrono::milliseconds(500);

} // namespace

// 校验并更新会话
bool VehicleControlSession::acceptControlCommand(
    const pb::ControlCommand &command, std::uint32_t sequence,
    Clock::time_point now) {
  // 无会话时只允许 ENTER 建立新的控制会话
  if (!cockpit_id_) {
    if (command.remote_mode_request() != pb::REMOTE_MODE_REQUEST_ENTER) {
      return false;
    }
    cockpit_id_ = command.cockpit_id();
    last_sequence_ = sequence;
    last_control_receive_time_ = now;
    return true;
  }

  // 有会话时只接受当前控制者的递增序号
  if (command.cockpit_id() != *cockpit_id_) {
    return false;
  }
  if (sequence <= last_sequence_) {
    return false;
  }

  // EXIT 转发后立即释放会话
  if (command.remote_mode_request() == pb::REMOTE_MODE_REQUEST_EXIT) {
    reset();
    return true;
  }

  last_sequence_ = sequence;
  last_control_receive_time_ = now;
  return true;
}

// 当前控制会话是否超时
bool VehicleControlSession::controlTimedOut(Clock::time_point now) const {
  return cockpit_id_ &&
         now - last_control_receive_time_ >= kRemoteControlTimeout;
}

// 主动结束会话
std::optional<pb::ControlCommand>
VehicleControlSession::stopRemoteControl() {
  if (!cockpit_id_)
    return std::nullopt;

  // 先生成退出请求，再清空会话
  pb::ControlCommand exit_command;
  exit_command.set_cockpit_id(*cockpit_id_);
  exit_command.set_remote_mode_request(pb::REMOTE_MODE_REQUEST_EXIT);
  reset();
  return exit_command;
}

// 返回当前控制驾驶舱 ID
const std::string &VehicleControlSession::cockpitId() const {
  static const std::string empty;
  return cockpit_id_ ? *cockpit_id_ : empty;
}

// 清空状态
void VehicleControlSession::reset() {
  cockpit_id_.reset();
  last_sequence_ = 0;
  last_control_receive_time_ = {};
}
