#include "protocol/udp_protocol.h"

#include <cmath>
#include <string>

namespace remote_protocol {
namespace {

namespace pb = remote_drive::protocol;

bool validId(const std::string &value) {
  return !value.empty() && value.size() <= kMaxIdLength;
}

bool validOptionalId(const std::string &value) {
  return value.size() <= kMaxIdLength;
}

bool validSwitch(pb::SwitchCommand command) {
  return pb::SwitchCommand_IsValid(command);
}

void fillHeader(pb::UdpPacket &packet, std::uint32_t sequence) {
  packet.set_magic(kMagic);
  packet.set_sequence(sequence);
}

UdpPacketBytes serialize(const pb::UdpPacket &packet) {
  std::string bytes;
  if (!packet.SerializeToString(&bytes)) {
    return {};
  }
  return UdpPacketBytes(bytes.begin(), bytes.end());
}

} // namespace

UdpPacketBytes encodeHeartbeat(const std::string &vehicle_id,
                               std::uint32_t sequence) {
  pb::UdpPacket packet;
  fillHeader(packet, sequence);
  packet.mutable_heartbeat()->set_vehicle_id(vehicle_id);
  return serialize(packet);
}

UdpPacketBytes encodeControlCommand(const pb::RemoteDriveControlCommand &command,
                                    std::uint32_t sequence) {
  pb::UdpPacket packet;
  fillHeader(packet, sequence);
  packet.mutable_control()->CopyFrom(command);
  return serialize(packet);
}

UdpPacketBytes encodeDrivingState(const pb::ChassisState &state,
                                  std::uint32_t sequence) {
  pb::UdpPacket packet;
  fillHeader(packet, sequence);
  packet.mutable_state()->CopyFrom(state);
  return serialize(packet);
}

std::optional<DecodedPacket> decodePacket(const std::uint8_t *data,
                                          std::size_t size) {
  if (!data || size == 0)
    return std::nullopt;

  pb::UdpPacket packet;
  if (!packet.ParseFromArray(data, static_cast<int>(size)) ||
      packet.magic() != kMagic) {
    return std::nullopt;
  }

  DecodedPacket decoded{};
  decoded.sequence = packet.sequence();
  switch (packet.body_case()) {
  case pb::UdpPacket::kHeartbeat:
    if (!validId(packet.heartbeat().vehicle_id())) {
      return std::nullopt;
    }
    decoded.body = PacketBody::HEARTBEAT;
    decoded.vehicle_id = packet.heartbeat().vehicle_id();
    return decoded;
  case pb::UdpPacket::kControl:
    if (!validate(packet.control())) {
      return std::nullopt;
    }
    decoded.body = PacketBody::CONTROL_CMD;
    decoded.control.CopyFrom(packet.control());
    return decoded;
  case pb::UdpPacket::kState:
    if (!validate(packet.state())) {
      return std::nullopt;
    }
    decoded.body = PacketBody::VEHICLE_STATE;
    decoded.vehicle_id = packet.state().vehicle_id();
    decoded.state.CopyFrom(packet.state());
    return decoded;
  default:
    return std::nullopt;
  }
}

bool validate(const pb::RemoteDriveControlCommand &command) {
  return validId(command.cockpit_id()) &&
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
         validSwitch(command.light_fog()) &&
         validSwitch(command.diff_lock());
}

bool validate(const pb::ChassisState &state) {
  return validId(state.vehicle_id()) &&
         validOptionalId(state.controller_id()) &&
         std::isfinite(state.steering_angle()) &&
         std::isfinite(state.speed()) &&
         pb::DriveMode_IsValid(state.drive_mode()) &&
         pb::Gear_IsValid(state.gear()) && pb::Bucket_IsValid(state.bucket());
}

} // namespace remote_protocol
