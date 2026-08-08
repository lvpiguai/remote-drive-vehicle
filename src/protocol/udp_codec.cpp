#include "protocol/udp_codec.h"

#include <string>

namespace udp_codec {
namespace {

namespace pb = remote_drive::protocol;

constexpr std::uint32_t kMagic = 0x52445550; // RDUP

void fillEnvelope(pb::UdpPacket &packet, std::uint32_t sequence) {
  packet.set_magic(kMagic);
  packet.set_sequence(sequence);
}

PacketBytes serialize(const pb::UdpPacket &packet) {
  std::string bytes;
  if (!packet.SerializeToString(&bytes))
    return {};
  return PacketBytes(bytes.begin(), bytes.end());
}

} // namespace

PacketBytes encodeHeartbeat(const std::string &vehicle_id,
                            std::uint32_t sequence) {
  pb::UdpPacket packet;
  fillEnvelope(packet, sequence);
  packet.mutable_heartbeat()->set_vehicle_id(vehicle_id);
  return serialize(packet);
}

PacketBytes encodeControlCommand(const pb::RemoteDriveControlCommand &command,
                                 std::uint32_t sequence) {
  pb::UdpPacket packet;
  fillEnvelope(packet, sequence);
  packet.mutable_control()->CopyFrom(command);
  return serialize(packet);
}

PacketBytes encodeDrivingState(const pb::ChassisState &state,
                               std::uint32_t sequence) {
  pb::UdpPacket packet;
  fillEnvelope(packet, sequence);
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

} // namespace udp_codec
