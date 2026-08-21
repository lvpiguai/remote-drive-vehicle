#include "protocol_codec.h"

#include <cmath>
#include <cstddef>
#include <string>

namespace protocol_codec {
namespace {

namespace pb = remote_drive::protocol;

constexpr std::uint32_t kMagic = 0x52445550; // RDUP
constexpr std::size_t kMaxIdLength = 19;

// 校验标识字段
bool validId(const std::string &id, bool required) {
  return (!required || !id.empty()) && id.size() <= kMaxIdLength &&
         id.find_first_of("\\\"") == std::string::npos;
}

// 校验三态开关
bool validSwitch(pb::SwitchCommand command) {
  return pb::SwitchCommand_IsValid(command);
}

// 校验控制字段
bool validControlCommand(const pb::ControlCommand &command) {
  return validId(command.cockpit_id(), true) &&
         std::isfinite(command.steering_angle()) &&
         std::isfinite(command.accelerator_percent()) &&
         std::isfinite(command.brake_percent()) &&
         command.steering_angle() >= -90.0 &&
         command.steering_angle() <= 90.0 &&
         command.accelerator_percent() >= 0.0 &&
         command.accelerator_percent() <= 100.0 &&
         command.brake_percent() >= 0.0 &&
         command.brake_percent() <= 100.0 &&
         pb::GearCommand_IsValid(command.gear()) &&
         pb::BucketCommand_IsValid(command.bucket()) &&
         pb::RemoteModeRequest_IsValid(command.remote_mode_request()) &&
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

// 校验车辆状态字段
bool validVehicleState(const pb::VehicleState &state) {
  return validId(state.vehicle_id(), true) &&
         validId(state.cockpit_id(), false) &&
         std::isfinite(state.steering_angle()) &&
         std::isfinite(state.speed()) &&
         pb::DriveMode_IsValid(state.drive_mode()) &&
         pb::GearState_IsValid(state.gear()) &&
         pb::BucketState_IsValid(state.bucket());
}

// 根据包体类型校验对应字段
bool validBody(const pb::ProtocolPacket &packet) {
  switch (packet.body_case()) {
    case pb::ProtocolPacket::kControl:
      return validControlCommand(packet.control());
    case pb::ProtocolPacket::kState:
      return validVehicleState(packet.state());
    case pb::ProtocolPacket::BODY_NOT_SET:
      return false;
  }
  return false;
}

} // namespace

// 编码车辆状态
PacketBytes encodeVehicleState(const pb::VehicleState &state,
                               std::uint32_t sequence) {
  if (!validVehicleState(state)) return {};

  pb::ProtocolPacket packet;
  packet.set_magic(kMagic);
  packet.set_sequence(sequence);
  packet.mutable_state()->CopyFrom(state);

  std::string bytes;
  if (!packet.SerializeToString(&bytes)) return {};
  return {bytes.begin(), bytes.end()};
}

// 解码并校验协议包
std::optional<pb::ProtocolPacket> decodePacket(const std::uint8_t *data,
                                               std::size_t size) {
  if (!data || size == 0)
    return std::nullopt;

  pb::ProtocolPacket packet;
  if (!packet.ParseFromArray(data, static_cast<int>(size)) ||
      packet.magic() != kMagic || !validBody(packet)) {
    return std::nullopt;
  }
  return packet;
}

} // namespace protocol_codec
