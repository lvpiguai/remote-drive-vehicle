#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "remote_drive.pb.h"

namespace remote_protocol {

constexpr std::uint32_t kMagic = 0x52445550; // RDUP

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

// 解析一个结构完整的 UDP protobuf 包
std::optional<remote_drive::protocol::UdpPacket>
decodePacket(const std::uint8_t *data, std::size_t size);

} // namespace remote_protocol
