#include <cassert>
#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "protocol/udp_protocol.h"
#include "vehicle/chassis_gateway.h"
#include "vehicle/vehicle_control_session.h"

namespace {

namespace pb = remote_drive::protocol;

class FakeChassisGateway : public ChassisGateway {
 public:
  bool publishControl(const pb::RemoteDriveControlCommand &command,
                      std::uint32_t sequence) override {
    published_commands.push_back(command);
    published_sequences.push_back(sequence);
    return publish_result;
  }

  std::optional<pb::ChassisState> latestState() const override {
    return latest_state;
  }

  bool publish_result = true;
  std::optional<pb::ChassisState> latest_state;
  std::vector<pb::RemoteDriveControlCommand> published_commands;
  std::vector<std::uint32_t> published_sequences;
};

// 验证心跳包编解码
void testHeartbeatRoundTrip() {
  const auto packet = remote_protocol::encodeHeartbeat("truck_01", 3);
  const auto output =
      remote_protocol::decodePacket(packet.data(), packet.size());
  assert(output);
  assert(output->body_case() == pb::UdpPacket::kHeartbeat);
  assert(output->sequence() == 3);
  assert(output->heartbeat().vehicle_id() == "truck_01");

  auto invalid = packet;
  invalid[0] = 0;
  assert(!remote_protocol::decodePacket(invalid.data(), invalid.size()));
  assert(!remote_protocol::decodePacket(packet.data(), 1));

  const auto empty_id = remote_protocol::encodeHeartbeat("", 4);
  const auto decoded_empty_id =
      remote_protocol::decodePacket(empty_id.data(), empty_id.size());
  assert(decoded_empty_id);
  assert(decoded_empty_id->heartbeat().vehicle_id().empty());
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
  input.set_remote_mode(pb::REMOTE_MODE_ENTER);
  input.set_horn(pb::SWITCH_ON);
  input.set_light_near(pb::SWITCH_ON);
  input.set_diff_lock(pb::SWITCH_ON);

  const auto packet = remote_protocol::encodeControlCommand(input, 42);
  const auto output =
      remote_protocol::decodePacket(packet.data(), packet.size());
  assert(output);
  assert(output->body_case() == pb::UdpPacket::kControl);
  assert(output->sequence() == 42);
  assert(output->control().cockpit_id() == "cockpit_01");
  assert(output->control().steering_angle() == -12.5);
  assert(output->control().accelerator_percent() == 35);
  assert(output->control().gear() == pb::GEAR_DRIVE_1);
  assert(output->control().bucket() == pb::BUCKET_DOWN);
  assert(output->control().remote_mode() == pb::REMOTE_MODE_ENTER);
  assert(output->control().horn() == pb::SWITCH_ON);
  assert(output->control().light_near() == pb::SWITCH_ON);
  assert(output->control().diff_lock() == pb::SWITCH_ON);

  auto invalid = packet;
  invalid[0] = 0;
  assert(!remote_protocol::decodePacket(invalid.data(), invalid.size()));
  assert(!remote_protocol::decodePacket(packet.data(), packet.size() - 1));

  input.clear_cockpit_id();
  const auto empty_id = remote_protocol::encodeControlCommand(input, 43);
  const auto decoded_empty_id =
      remote_protocol::decodePacket(empty_id.data(), empty_id.size());
  assert(decoded_empty_id);
  assert(!VehicleControlSession::isValidCommand(decoded_empty_id->control()));

  input.set_cockpit_id("cockpit_01");
  input.set_steering_angle(std::numeric_limits<double>::quiet_NaN());
  const auto invalid_number =
      remote_protocol::encodeControlCommand(input, 44);
  const auto decoded_invalid_number = remote_protocol::decodePacket(
      invalid_number.data(), invalid_number.size());
  assert(decoded_invalid_number);
  assert(!VehicleControlSession::isValidCommand(
      decoded_invalid_number->control()));
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

  const auto packet = remote_protocol::encodeDrivingState(input, 7);
  const auto output =
      remote_protocol::decodePacket(packet.data(), packet.size());
  assert(output);
  assert(output->body_case() == pb::UdpPacket::kState);
  assert(output->sequence() == 7);
  assert(output->state().steering_angle() == 8.25);
  assert(output->state().speed() == 12);
  assert(output->state().vehicle_id() == "truck_01");
  assert(output->state().controller_id() == "cockpit_02");
  assert(output->state().drive_mode() == pb::DRIVE_MODE_REMOTE);

  auto invalid = packet;
  invalid[0] = 0;
  assert(!remote_protocol::decodePacket(invalid.data(), invalid.size()));

  input.clear_vehicle_id();
  const auto empty_id = remote_protocol::encodeDrivingState(input, 8);
  assert(remote_protocol::decodePacket(empty_id.data(), empty_id.size()));

  input.set_vehicle_id("truck_01");
  input.set_speed(std::numeric_limits<double>::infinity());
  const auto invalid_number = remote_protocol::encodeDrivingState(input, 9);
  assert(remote_protocol::decodePacket(invalid_number.data(),
                                       invalid_number.size()));
}

// 验证车端网关流程
void testVehicleControlSession() {
  using Clock = VehicleControlSession::Clock;
  const auto start = Clock::time_point(std::chrono::seconds(1));
  FakeChassisGateway gateway;
  std::optional<VehicleControlSession> session;
  session.emplace(gateway, 1);

  pb::RemoteDriveControlCommand command;
  command.set_cockpit_id("cockpit_01");
  command.set_remote_mode(pb::REMOTE_MODE_ENTER);
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

  auto invalid_command = command;
  invalid_command.set_accelerator_percent(101);
  const auto invalid =
      session->handleCommand(invalid_command, 9, 1, start);
  assert(invalid.result == VehicleControlSession::Result::INVALID_COMMAND);
  assert(!invalid.applied);
  assert(gateway.published_commands.empty());

  const auto first = session->handleCommand(command, 10, 1, start);
  assert(first.result == VehicleControlSession::Result::ACCEPTED);
  assert(first.applied && !first.ended);
  assert(gateway.published_commands.size() == 1);
  assert(gateway.published_sequences.back() == 10);
  assert(gateway.published_commands.back().remote_mode() ==
         pb::REMOTE_MODE_ENTER);
  assert(gateway.published_commands.back().accelerator_percent() == 20);
  assert(gateway.published_commands.back().gear() == pb::GEAR_DRIVE_1);
  assert(gateway.published_commands.back().bucket() == pb::BUCKET_UP);
  assert(gateway.published_commands.back().horn() == pb::SWITCH_ON);

  // NO_CTL 作为原始控制指令继续发布给真实底盘通道
  command.set_window_wiper(pb::SWITCH_NO_CONTROL);
  command.set_light_near(pb::SWITCH_NO_CONTROL);
  assert(session->handleCommand(command, 11, 1, start).result ==
         VehicleControlSession::Result::ACCEPTED);
  assert(gateway.published_commands.size() == 2);
  assert(gateway.published_commands.back().window_wiper() ==
         pb::SWITCH_NO_CONTROL);
  assert(gateway.published_commands.back().light_near() ==
         pb::SWITCH_NO_CONTROL);

  command.set_parking(pb::SWITCH_ON);
  assert(session->handleCommand(command, 12, 1, start).result ==
         VehicleControlSession::Result::ACCEPTED);
  assert(gateway.published_commands.size() == 3);

  assert(session->handleCommand(command, 9, 1, start).result ==
         VehicleControlSession::Result::STALE_SEQUENCE);
  assert(session->handleCommand(command, 13, 2, start).result ==
         VehicleControlSession::Result::CONTROLLER_BUSY);
  assert(gateway.published_commands.size() == 3);

  assert(session->tick(start + std::chrono::milliseconds(1499)));
  assert(!session->tick(start + std::chrono::milliseconds(1500)));
  assert(gateway.published_commands.size() == 4);
  assert(gateway.published_commands.back().remote_mode() ==
         pb::REMOTE_MODE_EXIT);
  assert(gateway.published_commands.back().gear() == pb::GEAR_NEUTRAL);
  assert(gateway.published_commands.back().parking() == pb::SWITCH_ON);
  assert(gateway.published_commands.back().remote_emergency() ==
         pb::SWITCH_ON);
  session.reset();

  session.emplace(gateway, 2);
  command.set_remote_mode(pb::REMOTE_MODE_ENTER);
  command.set_remote_emergency(pb::SWITCH_OFF);
  assert(session->handleCommand(command, 12, 2, start + std::chrono::seconds(2))
             .result == VehicleControlSession::Result::ACCEPTED);
  command.set_remote_mode(pb::REMOTE_MODE_EXIT);
  const auto exit =
      session->handleCommand(command, 13, 2, start + std::chrono::seconds(2));
  assert(exit.result == VehicleControlSession::Result::ACCEPTED);
  assert(exit.ended);
  assert(gateway.published_commands.back().remote_mode() ==
         pb::REMOTE_MODE_EXIT);
  assert(gateway.published_commands.back().remote_emergency() ==
         pb::SWITCH_NO_CONTROL);
  session.reset();
}

} // namespace

// 运行 UDP protobuf 协议测试
int main() {
  testHeartbeatRoundTrip();
  testControlRoundTrip();
  testStateRoundTrip();
  testVehicleControlSession();
}
