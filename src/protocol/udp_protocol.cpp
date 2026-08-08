#include "protocol/udp_protocol.h"

#include <string>

namespace remote_protocol {
namespace {

namespace pb = remote_drive::protocol;

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

std::optional<pb::UdpPacket> decodePacket(const std::uint8_t *data,
                                          std::size_t size) {
  if (!data || size == 0)
    return std::nullopt;

  pb::UdpPacket packet;
  if (!packet.ParseFromArray(data, static_cast<int>(size)) ||
      packet.magic() != kMagic ||
      packet.body_case() == pb::UdpPacket::BODY_NOT_SET) {
    return std::nullopt;
  }
  return packet;
}

} // namespace remote_protocol
