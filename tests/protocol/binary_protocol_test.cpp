#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

#include "protocol/binary_codec.h"
#include "vehicle/chassis_gateway.h"
#include "vehicle/vehicle_control_session.h"

namespace {

class FakeChassisGateway : public ChassisGateway {
 public:
  bool publishControl(const RemoteCtlCmd &command,
                      std::uint32_t sequence) override {
    published_commands.push_back(command);
    published_sequences.push_back(sequence);
    return publish_result;
  }

  std::optional<RemoteDrivingState> latestState() const override {
    return latest_state;
  }

  bool publish_result = true;
  std::optional<RemoteDrivingState> latest_state;
  std::vector<RemoteCtlCmd> published_commands;
  std::vector<std::uint32_t> published_sequences;
};

// 验证心跳包编解码
void testHeartbeatRoundTrip() {
  HeartbeatPayload input{};
  std::memcpy(input.vehicle_id, "truck_01", 8);

  const auto packet = remote_protocol::encodeHeartbeat(input, 3);
  assert(packet.size() == 27);
  assert(packet[2] == static_cast<std::uint8_t>(MsgType::HEARTBEAT));
  assert(remote_protocol::decodeMessageType(packet.data(), packet.size()) ==
         MsgType::HEARTBEAT);

  HeartbeatPayload output{};
  std::uint32_t sequence = 0;
  assert(remote_protocol::decodeHeartbeat(packet.data(), packet.size(), output,
                                          sequence));
  assert(sequence == 3);
  assert(std::strcmp(output.vehicle_id, "truck_01") == 0);

  auto invalid = packet;
  invalid[2] = static_cast<std::uint8_t>(MsgType::VEHICLE_STATE);
  assert(!remote_protocol::decodeHeartbeat(invalid.data(), invalid.size(),
                                           output, sequence));
  invalid = packet;
  invalid[0] = 0;
  assert(!remote_protocol::decodeMessageType(invalid.data(), invalid.size()));
  assert(!remote_protocol::decodeMessageType(packet.data(), 1));
  invalid = packet;
  std::memset(invalid.data() + sizeof(PacketHeader), 'x',
              sizeof(HeartbeatPayload));
  assert(!remote_protocol::decodeHeartbeat(invalid.data(), invalid.size(),
                                           output, sequence));
}

// 验证控制包编解码
void testControlRoundTrip() {
  RemoteCtlCmd input{};
  std::memcpy(input.cockpit_id, "cockpit_01", 10);
  input.steering_angle = -12.5;
  input.acc_pedal = 35;
  input.brake_pedal = 2;
  input.gear = GearInfo::DRIVE_1;
  input.bucket_info = BucketInfo::BUCKET_DOWN;
  input.remoteMode = RemoteMode::REMOTE_ENTER;
  input.horn = SwitchCommand::ON;
  input.light_near = SwitchCommand::ON;
  input.diff_lock = SwitchCommand::ON;

  const auto packet = remote_protocol::encodeControlCommand(input, 42);
  assert(packet.size() == 73);
  assert(packet[0] == 0xCD && packet[1] == 0xAB);
  assert(packet[2] == static_cast<std::uint8_t>(MsgType::CONTROL_CMD));

  RemoteCtlCmd output{};
  std::uint32_t sequence = 0;
  assert(remote_protocol::decodeControlCommand(packet.data(), packet.size(),
                                               output, sequence));
  assert(sequence == 42);
  assert(std::strcmp(output.cockpit_id, "cockpit_01") == 0);
  assert(output.steering_angle == -12.5);
  assert(output.acc_pedal == 35);
  assert(output.gear == GearInfo::DRIVE_1);
  assert(output.bucket_info == BucketInfo::BUCKET_DOWN);
  assert(output.remoteMode == RemoteMode::REMOTE_ENTER);
  assert(output.horn == SwitchCommand::ON);
  assert(output.light_near == SwitchCommand::ON);
  assert(output.diff_lock == SwitchCommand::ON);

  auto invalid = packet;
  invalid[0] = 0;
  assert(!remote_protocol::decodeControlCommand(invalid.data(), invalid.size(),
                                                output, sequence));
  assert(!remote_protocol::decodeControlCommand(
      packet.data(), packet.size() - 1, output, sequence));
  invalid = packet;
  invalid.back() = 3;
  assert(!remote_protocol::decodeControlCommand(invalid.data(), invalid.size(),
                                                output, sequence));
  invalid = packet;
  invalid[sizeof(PacketHeader)] = '\0';
  assert(!remote_protocol::decodeControlCommand(invalid.data(), invalid.size(),
                                                output, sequence));
}

// 验证状态包编解码
void testStateRoundTrip() {
  RemoteDrivingState input{};
  std::memcpy(input.vehicle_id, "truck_01", 8);
  std::memcpy(input.controller_id, "cockpit_02", 10);
  input.steering = 8.25;
  input.speed = 12;
  input.remoteMode = DriveMode::REMOTE;
  input.gear = GearInfo::DRIVE_1;
  input.parking = false;

  const auto packet = remote_protocol::encodeDrivingState(input, 7);
  assert(packet.size() == 85);
  RemoteDrivingState output{};
  std::uint32_t sequence = 0;
  assert(remote_protocol::decodeDrivingState(packet.data(), packet.size(),
                                             output, sequence));
  assert(sequence == 7);
  assert(output.steering == 8.25);
  assert(output.speed == 12);
  assert(std::strcmp(output.vehicle_id, "truck_01") == 0);
  assert(std::strcmp(output.controller_id, "cockpit_02") == 0);
  assert(output.remoteMode == DriveMode::REMOTE);

  auto invalid = packet;
  invalid[sizeof(PacketHeader)] = '\0';
  assert(!remote_protocol::decodeDrivingState(invalid.data(), invalid.size(),
                                              output, sequence));
  invalid = packet;
  std::memset(invalid.data() + sizeof(PacketHeader), 'x',
              sizeof(RemoteDrivingState{}.vehicle_id));
  assert(!remote_protocol::decodeDrivingState(invalid.data(), invalid.size(),
                                              output, sequence));
}

// 验证车端网关流程
void testVehicleControlSession() {
  using Clock = VehicleControlSession::Clock;
  const auto start = Clock::time_point(std::chrono::seconds(1));
  FakeChassisGateway gateway;
  std::optional<VehicleControlSession> session;
  session.emplace(gateway, 1);

  RemoteCtlCmd command{};
  command.remoteMode = RemoteMode::REMOTE_ENTER;
  command.parking = SwitchCommand::OFF;
  command.gear = GearInfo::DRIVE_1;
  command.bucket_info = BucketInfo::BUCKET_UP;
  command.acc_pedal = 20;
  command.horn = SwitchCommand::ON;
  command.spray = SwitchCommand::ON;
  command.window_wiper = SwitchCommand::ON;
  command.light_brake = SwitchCommand::ON;
  command.light_position = SwitchCommand::ON;
  command.light_near = SwitchCommand::ON;
  command.light_far = SwitchCommand::ON;
  command.light_turn_left = SwitchCommand::ON;
  command.light_turn_right = SwitchCommand::ON;
  command.light_working_rear = SwitchCommand::ON;
  command.light_danger = SwitchCommand::ON;
  command.light_reverse = SwitchCommand::ON;
  command.light_double_flash = SwitchCommand::ON;
  command.light_front = SwitchCommand::ON;
  command.light_working_side = SwitchCommand::ON;
  command.light_fog = SwitchCommand::ON;
  command.diff_lock = SwitchCommand::ON;
  const auto first = session->handleCommand(command, 10, 1, start);
  assert(first.result == VehicleControlSession::Result::ACCEPTED);
  assert(first.applied && !first.ended);
  assert(gateway.published_commands.size() == 1);
  assert(gateway.published_sequences.back() == 10);
  assert(gateway.published_commands.back().remoteMode ==
         RemoteMode::REMOTE_ENTER);
  assert(gateway.published_commands.back().acc_pedal == 20);
  assert(gateway.published_commands.back().gear == GearInfo::DRIVE_1);
  assert(gateway.published_commands.back().bucket_info ==
         BucketInfo::BUCKET_UP);
  assert(gateway.published_commands.back().horn == SwitchCommand::ON);

  // NO_CTL 作为原始控制指令继续发布给真实底盘通道
  command.window_wiper = SwitchCommand::NO_CTL;
  command.light_near = SwitchCommand::NO_CTL;
  assert(session->handleCommand(command, 11, 1, start).result ==
         VehicleControlSession::Result::ACCEPTED);
  assert(gateway.published_commands.size() == 2);
  assert(gateway.published_commands.back().window_wiper ==
         SwitchCommand::NO_CTL);
  assert(gateway.published_commands.back().light_near ==
         SwitchCommand::NO_CTL);

  command.parking = SwitchCommand::ON;
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
  assert(gateway.published_commands.back().remoteMode ==
         RemoteMode::REMOTE_EXIT);
  assert(gateway.published_commands.back().gear == GearInfo::NEUTRAL);
  assert(gateway.published_commands.back().parking == SwitchCommand::ON);
  assert(gateway.published_commands.back().remote_emergency ==
         SwitchCommand::ON);
  session.reset();

  session.emplace(gateway, 2);
  command.remoteMode = RemoteMode::REMOTE_ENTER;
  command.remote_emergency = SwitchCommand::OFF;
  assert(session->handleCommand(command, 12, 2, start + std::chrono::seconds(2))
             .result == VehicleControlSession::Result::ACCEPTED);
  command.remoteMode = RemoteMode::REMOTE_EXIT;
  const auto exit =
      session->handleCommand(command, 13, 2, start + std::chrono::seconds(2));
  assert(exit.result == VehicleControlSession::Result::ACCEPTED);
  assert(exit.ended);
  assert(gateway.published_commands.back().remoteMode ==
         RemoteMode::REMOTE_EXIT);
  assert(gateway.published_commands.back().remote_emergency ==
         SwitchCommand::NO_CTL);
  session.reset();
}

} // namespace

// 运行二进制协议测试
int main() {
  testHeartbeatRoundTrip();
  testControlRoundTrip();
  testStateRoundTrip();
  testVehicleControlSession();
}
