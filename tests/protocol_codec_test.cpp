#include <cassert>
#include <chrono>
#include <cstdint>
#include <limits>
#include <string>

#include "protocol_codec.h"
#include "vehicle_control_session.h"

namespace {

namespace pb = remote_drive::protocol;

protocol_codec::PacketBytes controlPacketBytes(
    const pb::ControlCommand &command, std::uint32_t sequence) {
  pb::ProtocolPacket packet;
  packet.set_magic(0x52445550);
  packet.set_sequence(sequence);
  packet.mutable_control()->CopyFrom(command);

  std::string bytes;
  assert(packet.SerializeToString(&bytes));
  return {bytes.begin(), bytes.end()};
}

// 验证控制包解码
void testControlDecoding() {
  pb::ControlCommand input;
  input.set_cockpit_id("cockpit_01");
  input.set_steering_angle(-12.5);
  input.set_accelerator_percent(35);
  input.set_brake_percent(2);
  input.set_gear(pb::GEAR_COMMAND_DRIVE);
  input.set_bucket(pb::BUCKET_COMMAND_DOWN);
  input.set_remote_mode_request(pb::REMOTE_MODE_REQUEST_ENTER);
  input.set_horn(pb::SWITCH_ON);
  input.set_light_near(pb::SWITCH_ON);
  input.set_diff_lock(pb::SWITCH_ON);

  const auto packet = controlPacketBytes(input, 42);
  const auto output =
      protocol_codec::decodePacket(packet.data(), packet.size());
  assert(output);
  assert(output->body_case() == pb::ProtocolPacket::kControl);
  assert(output->sequence() == 42);
  assert(output->control().cockpit_id() == "cockpit_01");
  assert(output->control().steering_angle() == -12.5);
  assert(output->control().accelerator_percent() == 35);
  assert(output->control().gear() == pb::GEAR_COMMAND_DRIVE);
  assert(output->control().bucket() == pb::BUCKET_COMMAND_DOWN);
  assert(output->control().remote_mode_request() == pb::REMOTE_MODE_REQUEST_ENTER);
  assert(output->control().horn() == pb::SWITCH_ON);
  assert(output->control().light_near() == pb::SWITCH_ON);
  assert(output->control().diff_lock() == pb::SWITCH_ON);

  auto invalid = packet;
  invalid[0] = 0;
  assert(!protocol_codec::decodePacket(invalid.data(), invalid.size()));
  assert(!protocol_codec::decodePacket(packet.data(), packet.size() - 1));

  input.clear_cockpit_id();
  auto invalid_fields = controlPacketBytes(input, 43);
  assert(!protocol_codec::decodePacket(invalid_fields.data(),
                                       invalid_fields.size()));

  input.set_cockpit_id("cockpit_01");
  input.set_steering_angle(std::numeric_limits<double>::quiet_NaN());
  invalid_fields = controlPacketBytes(input, 44);
  assert(!protocol_codec::decodePacket(invalid_fields.data(),
                                       invalid_fields.size()));

  // 拒绝线上非法字段
  auto invalid_packet = *output;
  invalid_packet.mutable_control()->set_accelerator_percent(101);
  std::string invalid_bytes;
  assert(invalid_packet.SerializeToString(&invalid_bytes));
  assert(!protocol_codec::decodePacket(
      reinterpret_cast<const std::uint8_t *>(invalid_bytes.data()),
      invalid_bytes.size()));
}

// 验证状态包编码
void testStateEncoding() {
  pb::VehicleState input;
  input.set_vehicle_id("truck_01");
  input.set_cockpit_id("cockpit_02");
  input.set_steering_angle(8.25);
  input.set_speed(12);
  input.set_drive_mode(pb::DRIVE_MODE_REMOTE);
  input.set_gear(pb::GEAR_STATE_DRIVE);

  const auto bytes = protocol_codec::encodeVehicleState(input, 7);
  assert(!bytes.empty());

  pb::ProtocolPacket packet;
  assert(packet.ParseFromArray(bytes.data(), static_cast<int>(bytes.size())));
  assert(packet.magic() == 0x52445550);
  assert(packet.body_case() == pb::ProtocolPacket::kState);
  assert(packet.sequence() == 7);
  assert(packet.state().steering_angle() == 8.25);
  assert(packet.state().speed() == 12);
  assert(packet.state().vehicle_id() == "truck_01");
  assert(packet.state().cockpit_id() == "cockpit_02");
  assert(packet.state().drive_mode() == pb::DRIVE_MODE_REMOTE);

  assert(protocol_codec::decodePacket(bytes.data(), bytes.size()));

  input.clear_vehicle_id();
  assert(protocol_codec::encodeVehicleState(input, 8).empty());

  input.set_vehicle_id("truck_01");
  input.set_speed(std::numeric_limits<double>::infinity());
  assert(protocol_codec::encodeVehicleState(input, 9).empty());
}

// 验证车端网关流程
void testVehicleControlSession() {
  using Clock = VehicleControlSession::Clock;
  const auto start = Clock::time_point(std::chrono::seconds(1));
  VehicleControlSession session;

  pb::ControlCommand command;
  command.set_cockpit_id("cockpit_01");
  command.set_remote_mode_request(pb::REMOTE_MODE_REQUEST_ENTER);
  command.set_parking(pb::SWITCH_OFF);
  command.set_gear(pb::GEAR_COMMAND_DRIVE);
  command.set_bucket(pb::BUCKET_COMMAND_UP);
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

  assert(session.acceptControlCommand(command, 10, start));
  assert(session.cockpitId() == "cockpit_01");

  // NO_CTL 作为原始控制指令继续发布给真实底盘通道
  command.set_window_wiper(pb::SWITCH_NO_CONTROL);
  command.set_light_near(pb::SWITCH_NO_CONTROL);
  assert(session.acceptControlCommand(command, 11, start));

  command.set_parking(pb::SWITCH_ON);
  assert(session.acceptControlCommand(command, 12, start));

  assert(!session.acceptControlCommand(command, 9, start));
  command.set_cockpit_id("cockpit_02");
  assert(!session.acceptControlCommand(command, 13, start));
  command.set_cockpit_id("cockpit_01");

  assert(!session.controlTimedOut(start + std::chrono::milliseconds(499)));
  assert(session.controlTimedOut(start + std::chrono::milliseconds(500)));
  const auto timeout_exit = session.stopRemoteControl();
  assert(timeout_exit);
  assert(timeout_exit->cockpit_id() == "cockpit_01");
  assert(timeout_exit->remote_mode_request() == pb::REMOTE_MODE_REQUEST_EXIT);
  assert(timeout_exit->brake_percent() == 0);
  assert(timeout_exit->gear() == pb::GEAR_COMMAND_NO_CONTROL);
  assert(timeout_exit->bucket() == pb::BUCKET_COMMAND_NO_CONTROL);
  assert(timeout_exit->parking() == pb::SWITCH_NO_CONTROL);
  assert(timeout_exit->remote_emergency() == pb::SWITCH_NO_CONTROL);
  command.set_remote_mode_request(pb::REMOTE_MODE_REQUEST_ENTER);
  command.set_remote_emergency(pb::SWITCH_OFF);
  assert(session.acceptControlCommand(command, 12,
                                      start + std::chrono::seconds(2)));
  command.set_remote_mode_request(pb::REMOTE_MODE_REQUEST_EXIT);
  assert(session.acceptControlCommand(command, 13,
                                      start + std::chrono::seconds(2)));
  assert(session.cockpitId().empty());

  command.set_remote_mode_request(pb::REMOTE_MODE_REQUEST_ENTER);
  assert(session.acceptControlCommand(command, 0,
                                      start + std::chrono::seconds(3)));
  assert(!session.acceptControlCommand(command, 0,
                                       start + std::chrono::seconds(3)));
  assert(session.acceptControlCommand(command, 1,
                                      start + std::chrono::seconds(3)));
}

} // namespace

// 运行协议编解码与会话测试
int main() {
  testControlDecoding();
  testStateEncoding();
  testVehicleControlSession();
}
