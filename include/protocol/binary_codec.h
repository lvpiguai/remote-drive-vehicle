#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "protocol/remote_control_protocol.h"

namespace remote_protocol {

constexpr std::uint16_t kMagic = 0xABCD;
constexpr std::size_t kHeartbeatPacketSize =
    sizeof(PacketHeader) + sizeof(HeartbeatPayload);
constexpr std::size_t kControlPacketSize =
    sizeof(PacketHeader) + sizeof(RemoteCtlCmd);
constexpr std::size_t kStatePacketSize =
    sizeof(PacketHeader) + sizeof(RemoteDrivingState);

using HeartbeatPacket = std::array<std::uint8_t, kHeartbeatPacketSize>;
using ControlPacket = std::array<std::uint8_t, kControlPacketSize>;
using StatePacket = std::array<std::uint8_t, kStatePacketSize>;

// 从完整或部分数据报中识别消息类型
std::optional<MsgType> decodeMessageType(const std::uint8_t *data,
                                         std::size_t size);

// 编码车辆在线心跳包
HeartbeatPacket encodeHeartbeat(const HeartbeatPayload &heartbeat,
                                std::uint32_t sequence);

// 编码驾驶舱控制指令包
ControlPacket encodeControlCommand(const RemoteCtlCmd &command,
                                   std::uint32_t sequence);

// 编码车端状态回传包
StatePacket encodeDrivingState(const RemoteDrivingState &state,
                               std::uint32_t sequence);

// 解码并校验车辆在线心跳包
bool decodeHeartbeat(const std::uint8_t *data, std::size_t size,
                     HeartbeatPayload &heartbeat, std::uint32_t &sequence);

// 解码并校验驾驶舱控制指令包
bool decodeControlCommand(const std::uint8_t *data, std::size_t size,
                          RemoteCtlCmd &command, std::uint32_t &sequence);

// 解码并校验车端状态回传包
bool decodeDrivingState(const std::uint8_t *data, std::size_t size,
                        RemoteDrivingState &state, std::uint32_t &sequence);

// 校验控制指令中的数值和枚举范围
bool validate(const RemoteCtlCmd &command);

} // namespace remote_protocol
