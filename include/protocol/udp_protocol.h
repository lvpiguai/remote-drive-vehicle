#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "remote_drive.pb.h"

namespace remote_protocol {

constexpr std::uint32_t kMagic = 0x52445550; // RDUP
constexpr std::uint32_t kProtocolVersion = 1;
constexpr std::size_t kMaxIdLength = 19;

enum class PacketBody {
  HEARTBEAT,
  CONTROL_CMD,
  VEHICLE_STATE,
};

struct DecodedPacket {
  PacketBody body = PacketBody::HEARTBEAT;
  std::uint32_t sequence = 0;
  std::string vehicle_id;
  remote_drive::protocol::RemoteDriveControlCommand control{};
  remote_drive::protocol::ChassisState state{};
};

using UdpPacketBytes = std::vector<std::uint8_t>;

// 编码车辆在线心跳包
UdpPacketBytes encodeHeartbeat(const std::string &vehicle_id,
                               std::uint32_t sequence);

// 编码驾驶舱控制指令包
UdpPacketBytes encodeControlCommand(
    const remote_drive::protocol::RemoteDriveControlCommand &command,
    std::uint32_t sequence);

// 编码车端状态回传包
UdpPacketBytes encodeDrivingState(
    const remote_drive::protocol::ChassisState &state, std::uint32_t sequence);

// 解析并校验一个完整 UDP protobuf 包
std::optional<DecodedPacket> decodePacket(const std::uint8_t *data,
                                          std::size_t size);

// 校验控制指令中的数值和枚举范围
bool validate(
    const remote_drive::protocol::RemoteDriveControlCommand &command);
bool validate(const remote_drive::protocol::ChassisState &state);

} // namespace remote_protocol
