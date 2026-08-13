#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "remote_drive.pb.h"

namespace protocol_codec {

using PacketBytes = std::vector<std::uint8_t>;

// 编码控制命令
PacketBytes encodeControlCommand(
    const remote_drive::protocol::RemoteDriveControlCommand &command,
    std::uint32_t sequence);

// 编码车辆状态
PacketBytes encodeDrivingState(
    const remote_drive::protocol::ChassisState &state, std::uint32_t sequence);

// 解码并校验协议包
std::optional<remote_drive::protocol::ProtocolPacket>
decodePacket(const std::uint8_t *data, std::size_t size);

} // namespace protocol_codec
