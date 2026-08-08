#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <optional>

#include "protocol/binary_codec.h"
#include "vehicle/vehicle_control_session.h"

namespace {

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
  ChassisSimulator chassis;
  std::optional<VehicleControlSession> session;
  session.emplace(chassis, 1);

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
  assert(chassis.status().drive_mode == DriveMode::REMOTE);
  assert(chassis.status().acc_pedal == 20);
  assert(chassis.status().gear == GearInfo::DRIVE_1);
  assert(chassis.status().bucket == BucketInfo::BUCKET_UP);
  assert(chassis.status().horn);
  assert(chassis.status().spray);
  assert(chassis.status().window_wiper);
  assert(chassis.status().light_brake);
  assert(chassis.status().light_position);
  assert(chassis.status().light_near);
  assert(chassis.status().light_far);
  assert(chassis.status().light_turn_left);
  assert(chassis.status().light_turn_right);
  assert(chassis.status().light_working_rear);
  assert(chassis.status().light_danger);
  assert(chassis.status().light_reverse);
  assert(chassis.status().light_double_flash);
  assert(chassis.status().light_front);
  assert(chassis.status().light_working_side);
  assert(chassis.status().light_fog);
  assert(chassis.status().diff_lock);

  // NO_CTL 不覆盖车辆已经执行的开关状态
  command.window_wiper = SwitchCommand::NO_CTL;
  command.light_near = SwitchCommand::NO_CTL;
  assert(session->handleCommand(command, 11, 1, start).result ==
         VehicleControlSession::Result::ACCEPTED);
  assert(chassis.status().window_wiper);
  assert(chassis.status().light_near);

  // 已解除驻车、挂入前进挡且油门有效时，模拟车速应持续增长
  chassis.tick();
  assert(chassis.status().speed > 0);

  command.parking = SwitchCommand::ON;
  assert(session->handleCommand(command, 12, 1, start).result ==
         VehicleControlSession::Result::ACCEPTED);
  assert(chassis.status().acc_pedal == 0);
  assert(chassis.status().brake_pedal == 100);

  assert(session->handleCommand(command, 9, 1, start).result ==
         VehicleControlSession::Result::STALE_SEQUENCE);
  assert(session->handleCommand(command, 13, 2, start).result ==
         VehicleControlSession::Result::CONTROLLER_BUSY);

  assert(session->tick(start + std::chrono::milliseconds(1499)));
  assert(!session->tick(start + std::chrono::milliseconds(1500)));
  session.reset();
  assert(chassis.status().drive_mode == DriveMode::AUTO);
  assert(chassis.status().gear == GearInfo::NEUTRAL);
  assert(chassis.status().parking);
  assert(chassis.status().remote_emergency);
  assert(chassis.status().horn);
  assert(chassis.status().acc_pedal == 0);
  assert(chassis.status().brake_pedal == 100);

  session.emplace(chassis, 2);
  command.remoteMode = RemoteMode::REMOTE_ENTER;
  command.remote_emergency = SwitchCommand::OFF;
  assert(session->handleCommand(command, 12, 2, start + std::chrono::seconds(2))
             .result == VehicleControlSession::Result::ACCEPTED);
  command.remoteMode = RemoteMode::REMOTE_EXIT;
  const auto exit =
      session->handleCommand(command, 13, 2, start + std::chrono::seconds(2));
  assert(exit.result == VehicleControlSession::Result::ACCEPTED);
  assert(exit.ended);
  session.reset();
  assert(chassis.status().drive_mode == DriveMode::AUTO);
  assert(chassis.status().gear == GearInfo::NEUTRAL);
  assert(chassis.status().parking);
  assert(!chassis.status().remote_emergency);
  assert(chassis.status().horn);
}

} // namespace

// 运行二进制协议测试
int main() {
  testHeartbeatRoundTrip();
  testControlRoundTrip();
  testStateRoundTrip();
  testVehicleControlSession();
}
