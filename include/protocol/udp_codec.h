#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "remote_drive.pb.h"

namespace udp_codec {

using PacketBytes = std::vector<std::uint8_t>;

// 将车辆在线心跳封装并序列化为 UDP 负载
PacketBytes encodeHeartbeat(const std::string &vehicle_id,
                            std::uint32_t sequence);

// 将驾驶舱控制指令封装并序列化为 UDP 负载
PacketBytes encodeControlCommand(
    const remote_drive::protocol::RemoteDriveControlCommand &command,
    std::uint32_t sequence);

// 将车端状态封装并序列化为 UDP 负载
PacketBytes encodeDrivingState(
    const remote_drive::protocol::ChassisState &state, std::uint32_t sequence);

// 从 UDP 负载反序列化一个结构完整的 Protobuf 包
std::optional<remote_drive::protocol::UdpPacket>
decodePacket(const std::uint8_t *data, std::size_t size);

} // namespace udp_codec
