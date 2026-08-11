#include <cassert>
#include <chrono>
#include <cstdint>
#include <limits>
#include <string>

#include "protocol_codec.h"
#include "vehicle_control_session.h"

namespace {

namespace pb = remote_drive::protocol;

// 验证心跳包编解码
void testHeartbeatRoundTrip() {
  const auto packet = protocol_codec::encodeHeartbeat("truck_01", 3);
  const auto output =
      protocol_codec::decodePacket(packet.data(), packet.size());
  assert(output);
  assert(output->body_case() == pb::ProtocolPacket::kHeartbeat);
  assert(output->sequence() == 3);
  assert(output->heartbeat().vehicle_id() == "truck_01");

  auto invalid = packet;
  invalid[0] = 0;
  assert(!protocol_codec::decodePacket(invalid.data(), invalid.size()));
  assert(!protocol_codec::decodePacket(packet.data(), 1));

  assert(protocol_codec::encodeHeartbeat("", 4).empty());
}

// 验证控制包编解码
void testControlRoundTrip() {
  pb::RemoteDriveControlCommand input;
  input.set_cockpit_id("cockpit_01");
  input.set_steering_angle(-12.5);
  input.set_accelerator_percent(35);
  input.set_brake_percent(2);
  input.set_gear(pb::GEAR_DRIVE_1);
  input.set_bucket(pb::BUCKET_DOWN);
  input.set_remote_mode_request(pb::REMOTE_MODE_REQUEST_ENTER);
  input.set_horn(pb::SWITCH_ON);
  input.set_light_near(pb::SWITCH_ON);
  input.set_diff_lock(pb::SWITCH_ON);

  const auto packet = protocol_codec::encodeControlCommand(input, 42);
  const auto output =
      protocol_codec::decodePacket(packet.data(), packet.size());
  assert(output);
  assert(output->body_case() == pb::ProtocolPacket::kControl);
  assert(output->sequence() == 42);
  assert(output->control().cockpit_id() == "cockpit_01");
  assert(output->control().steering_angle() == -12.5);
  assert(output->control().accelerator_percent() == 35);
  assert(output->control().gear() == pb::GEAR_DRIVE_1);
  assert(output->control().bucket() == pb::BUCKET_DOWN);
  assert(output->control().remote_mode_request() == pb::REMOTE_MODE_REQUEST_ENTER);
  assert(output->control().horn() == pb::SWITCH_ON);
  assert(output->control().light_near() == pb::SWITCH_ON);
  assert(output->control().diff_lock() == pb::SWITCH_ON);

  auto invalid = packet;
  invalid[0] = 0;
  assert(!protocol_codec::decodePacket(invalid.data(), invalid.size()));
  assert(!protocol_codec::decodePacket(packet.data(), packet.size() - 1));

  input.clear_cockpit_id();
  assert(protocol_codec::encodeControlCommand(input, 43).empty());

  input.set_cockpit_id("cockpit_01");
  input.set_steering_angle(std::numeric_limits<double>::quiet_NaN());
  assert(protocol_codec::encodeControlCommand(input, 44).empty());

  // 拒绝线上非法字段
  auto invalid_fields = *output;
  invalid_fields.mutable_control()->set_accelerator_percent(101);
  std::string invalid_bytes;
  assert(invalid_fields.SerializeToString(&invalid_bytes));
  assert(!protocol_codec::decodePacket(
      reinterpret_cast<const std::uint8_t *>(invalid_bytes.data()),
      invalid_bytes.size()));
}

// 验证状态包编解码
void testStateRoundTrip() {
  pb::ChassisState input;
  input.set_vehicle_id("truck_01");
  input.set_controller_id("cockpit_02");
  input.set_steering_angle(8.25);
  input.set_speed(12);
  input.set_drive_mode(pb::DRIVE_MODE_REMOTE);
  input.set_gear(pb::GEAR_DRIVE_1);

  const auto packet = protocol_codec::encodeDrivingState(input, 7);
  const auto output =
      protocol_codec::decodePacket(packet.data(), packet.size());
  assert(output);
  assert(output->body_case() == pb::ProtocolPacket::kState);
  assert(output->sequence() == 7);
  assert(output->state().steering_angle() == 8.25);
  assert(output->state().speed() == 12);
  assert(output->state().vehicle_id() == "truck_01");
  assert(output->state().controller_id() == "cockpit_02");
  assert(output->state().drive_mode() == pb::DRIVE_MODE_REMOTE);

  auto invalid = packet;
  invalid[0] = 0;
  assert(!protocol_codec::decodePacket(invalid.data(), invalid.size()));

  input.clear_vehicle_id();
  assert(protocol_codec::encodeDrivingState(input, 8).empty());

  input.set_vehicle_id("truck_01");
  input.set_speed(std::numeric_limits<double>::infinity());
  assert(protocol_codec::encodeDrivingState(input, 9).empty());
}

// 验证车端网关流程
void testVehicleControlSession() {
  using Clock = VehicleControlSession::Clock;
  const auto start = Clock::time_point(std::chrono::seconds(1));
  VehicleControlSession session;

  pb::RemoteDriveControlCommand command;
  command.set_cockpit_id("cockpit_01");
  command.set_remote_mode_request(pb::REMOTE_MODE_REQUEST_ENTER);
  command.set_parking(pb::SWITCH_OFF);
  command.set_gear(pb::GEAR_DRIVE_1);
  command.set_bucket(pb::BUCKET_UP);
  command.set_accelerator_percent(20);
  command.set_horn(pb::SWITCH_ON);
  command.set_spray(pb::SWITCH_ON);
  command.set_window_wiper(pb::SWITCH_ON);
  command.set_light_brake(pb::SWITCH_ON);
  command.set_light_position(pb::SWITCH_ON);
  command.set_light_near(pb::SWITCH_ON);
  command.set_light_far(pb::SWITCH_ON);
  command.set_light_turn_left(pb::SWITCH_ON);
  command.set_light_turn_right(pb::SWITCH_ON);
  command.set_light_working_rear(pb::SWITCH_ON);
  command.set_light_danger(pb::SWITCH_ON);
  command.set_light_reverse(pb::SWITCH_ON);
  command.set_light_double_flash(pb::SWITCH_ON);
  command.set_light_front(pb::SWITCH_ON);
  command.set_light_working_side(pb::SWITCH_ON);
  command.set_light_fog(pb::SWITCH_ON);
  command.set_diff_lock(pb::SWITCH_ON);

  assert(session.acceptControlCommand(command, 10, 1, start));
  assert(session.cockpitId() == "cockpit_01");

  // NO_CTL 作为原始控制指令继续发布给真实底盘通道
  command.set_window_wiper(pb::SWITCH_NO_CONTROL);
  command.set_light_near(pb::SWITCH_NO_CONTROL);
  assert(session.acceptControlCommand(command, 11, 1, start));

  command.set_parking(pb::SWITCH_ON);
  assert(session.acceptControlCommand(command, 12, 1, start));

  assert(!session.acceptControlCommand(command, 9, 1, start));
  assert(!session.acceptControlCommand(command, 13, 2, start));

  assert(!session.controlTimedOut(start + std::chrono::milliseconds(1499)));
  assert(session.controlTimedOut(start + std::chrono::milliseconds(1500)));
  const auto timeout_exit = session.stopRemoteControl();
  assert(timeout_exit);
  assert(timeout_exit->cockpit_id() == "cockpit_01");
  assert(timeout_exit->remote_mode_request() == pb::REMOTE_MODE_REQUEST_EXIT);
  assert(timeout_exit->brake_percent() == 0);
  assert(timeout_exit->parking() == pb::SWITCH_NO_CONTROL);
  assert(timeout_exit->remote_emergency() == pb::SWITCH_NO_CONTROL);
  command.set_remote_mode_request(pb::REMOTE_MODE_REQUEST_ENTER);
  command.set_remote_emergency(pb::SWITCH_OFF);
  assert(session.acceptControlCommand(command, 12, 2,
                                      start + std::chrono::seconds(2)));
  command.set_remote_mode_request(pb::REMOTE_MODE_REQUEST_EXIT);
  assert(session.acceptControlCommand(command, 13, 2,
                                      start + std::chrono::seconds(2)));
  assert(session.cockpitId().empty());

  command.set_remote_mode_request(pb::REMOTE_MODE_REQUEST_ENTER);
  assert(session.acceptControlCommand(command, 0, 3,
                                      start + std::chrono::seconds(3)));
  assert(!session.acceptControlCommand(command, 0, 3,
                                       start + std::chrono::seconds(3)));
  assert(session.acceptControlCommand(command, 1, 3,
                                      start + std::chrono::seconds(3)));
}

} // namespace

// 运行协议编解码与会话测试
int main() {
  testHeartbeatRoundTrip();
  testControlRoundTrip();
  testStateRoundTrip();
  testVehicleControlSession();
}
