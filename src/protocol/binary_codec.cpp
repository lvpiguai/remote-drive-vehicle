#include "protocol/binary_codec.h"

#include <cmath>
#include <cstring>
#include <type_traits>

namespace remote_protocol {
namespace {

// 检查字节区间是否只包含合法布尔值
bool bytesAreBoolean(const std::uint8_t *begin, const std::uint8_t *end) {
  // 在 memcpy 到 bool 前拒绝非 0/1 位模式
  while (begin != end) {
    if (*begin++ > 1)
      return false;
  }
  return true;
}

// 检查字节区间是否只包含合法三态开关命令
bool bytesAreSwitchCommand(const std::uint8_t *begin, const std::uint8_t *end) {
  while (begin != end) {
    if (*begin++ > static_cast<std::uint8_t>(SwitchCommand::ON))
      return false;
  }
  return true;
}

template <typename Payload, std::size_t Size>
// 将业务结构编码为带包头的定长数据报
std::array<std::uint8_t, Size>
encodePacket(const Payload &payload, MsgType type, std::uint32_t sequence) {
  // 1 字节对齐结构体按本机小端布局直接拼接
  std::array<std::uint8_t, Size> packet{};
  PacketHeader header{};
  header.magic = kMagic;
  header.msg_type = static_cast<std::uint8_t>(type);
  header.seq_number = sequence;
  std::memcpy(packet.data(), &header, sizeof(header));
  std::memcpy(packet.data() + sizeof(header), &payload, sizeof(payload));
  return packet;
}

template <typename Payload>
// 校验包头并解出指定类型的 payload
bool decode(const std::uint8_t *data, std::size_t size, MsgType expected_type,
            Payload &payload, std::uint32_t &sequence) {
  if (!data || size != sizeof(PacketHeader) + sizeof(Payload)) {
    return false;
  }

  PacketHeader header{};
  std::memcpy(&header, data, sizeof(header));
  // UDP 一帧必须恰好对应一个完整且类型匹配的协议包
  if (header.magic != kMagic ||
      header.msg_type != static_cast<std::uint8_t>(expected_type)) {
    return false;
  }

  std::memcpy(&payload, data + sizeof(header), sizeof(payload));
  sequence = header.seq_number;
  return true;
}

template <typename Enum>
// 判断枚举底层值是否位于协议范围内
bool enumInRange(Enum value, Enum first, Enum last) {
  using Value = std::underlying_type_t<Enum>;
  return static_cast<Value>(value) >= static_cast<Value>(first) &&
         static_cast<Value>(value) <= static_cast<Value>(last);
}

} // namespace

// 识别消息类型
std::optional<MsgType> decodeMessageType(const std::uint8_t *data,
                                         std::size_t size) {
  if (!data || size < sizeof(PacketHeader))
    return std::nullopt;

  PacketHeader header{};
  std::memcpy(&header, data, sizeof(header));
  if (header.magic != kMagic ||
      header.msg_type < static_cast<std::uint8_t>(MsgType::HEARTBEAT) ||
      header.msg_type > static_cast<std::uint8_t>(MsgType::ERROR_REPORT)) {
    return std::nullopt;
  }
  return static_cast<MsgType>(header.msg_type);
}

// 编码车辆在线心跳包
HeartbeatPacket encodeHeartbeat(const HeartbeatPayload &heartbeat,
                                std::uint32_t sequence) {
  return encodePacket<HeartbeatPayload, kHeartbeatPacketSize>(
      heartbeat, MsgType::HEARTBEAT, sequence);
}

// 编码远程控制指令包
ControlPacket encodeControlCommand(const RemoteCtlCmd &command,
                                   std::uint32_t sequence) {
  return encodePacket<RemoteCtlCmd, kControlPacketSize>(
      command, MsgType::CONTROL_CMD, sequence);
}

// 编码车辆状态包
StatePacket encodeDrivingState(const RemoteDrivingState &state,
                               std::uint32_t sequence) {
  return encodePacket<RemoteDrivingState, kStatePacketSize>(
      state, MsgType::VEHICLE_STATE, sequence);
}

// 解码并校验车辆在线心跳包
bool decodeHeartbeat(const std::uint8_t *data, std::size_t size,
                     HeartbeatPayload &heartbeat, std::uint32_t &sequence) {
  HeartbeatPayload decoded{};
  if (!decode(data, size, MsgType::HEARTBEAT, decoded, sequence) ||
      decoded.vehicle_id[0] == '\0' ||
      std::memchr(decoded.vehicle_id, '\0', sizeof(decoded.vehicle_id)) ==
          nullptr) {
    return false;
  }
  heartbeat = decoded;
  return true;
}

// 解码并校验远程控制指令包
bool decodeControlCommand(const std::uint8_t *data, std::size_t size,
                          RemoteCtlCmd &command, std::uint32_t &sequence) {
  // 驾驶舱 ID、三个 double 和三个枚举之后均为三态开关命令
  constexpr std::size_t kSwitchCommandOffset = sizeof(PacketHeader) + 47;
  if (!data || size != kControlPacketSize ||
      !bytesAreSwitchCommand(data + kSwitchCommandOffset, data + size)) {
    return false;
  }
  RemoteCtlCmd decoded{};
  if (!decode(data, size, MsgType::CONTROL_CMD, decoded, sequence) ||
      decoded.cockpit_id[0] == '\0' ||
      std::memchr(decoded.cockpit_id, '\0', sizeof(decoded.cockpit_id)) ==
          nullptr ||
      !validate(decoded)) {
    return false;
  }
  command = decoded;
  return true;
}

// 解码并校验车辆状态包
bool decodeDrivingState(const std::uint8_t *data, std::size_t size,
                        RemoteDrivingState &state, std::uint32_t &sequence) {
  // 车辆 ID、控制者 ID、两个 double 和三个枚举之后均为实际布尔状态
  constexpr std::size_t kBooleanOffset = sizeof(PacketHeader) + 59;
  if (!data || size != kStatePacketSize ||
      !bytesAreBoolean(data + kBooleanOffset, data + size)) {
    return false;
  }
  RemoteDrivingState decoded{};
  if (!decode(data, size, MsgType::VEHICLE_STATE, decoded, sequence) ||
      decoded.vehicle_id[0] == '\0' ||
      std::memchr(decoded.vehicle_id, '\0', sizeof(decoded.vehicle_id)) ==
          nullptr ||
      std::memchr(decoded.controller_id, '\0', sizeof(decoded.controller_id)) ==
          nullptr ||
      !std::isfinite(decoded.steering) || !std::isfinite(decoded.speed) ||
      !enumInRange(decoded.remoteMode, DriveMode::MANUAL, DriveMode::AUTO) ||
      !enumInRange(decoded.gear, GearInfo::NEUTRAL, GearInfo::DRIVE_3) ||
      !enumInRange(decoded.bucket, BucketInfo::BUCKET_UP,
                   BucketInfo::BUCKET_KEEP)) {
    return false;
  }
  state = decoded;
  return true;
}

// 校验控制指令的数值与枚举范围
bool validate(const RemoteCtlCmd &command) {
  return std::isfinite(command.steering_angle) &&
         std::isfinite(command.acc_pedal) &&
         std::isfinite(command.brake_pedal) &&
         command.steering_angle >= -90.0 && command.steering_angle <= 90.0 &&
         command.acc_pedal >= 0.0 && command.acc_pedal <= 100.0 &&
         command.brake_pedal >= 0.0 && command.brake_pedal <= 100.0 &&
         enumInRange(command.gear, GearInfo::NEUTRAL, GearInfo::DRIVE_3) &&
         enumInRange(command.bucket_info, BucketInfo::BUCKET_UP,
                     BucketInfo::BUCKET_KEEP) &&
         enumInRange(command.remoteMode, RemoteMode::REMOTE_NO_CONTROL,
                     RemoteMode::REMOTE_EXIT) &&
         enumInRange(command.parking, SwitchCommand::NO_CTL,
                     SwitchCommand::ON) &&
         enumInRange(command.horn, SwitchCommand::NO_CTL, SwitchCommand::ON) &&
         enumInRange(command.spray, SwitchCommand::NO_CTL, SwitchCommand::ON) &&
         enumInRange(command.remote_emergency, SwitchCommand::NO_CTL,
                     SwitchCommand::ON) &&
         enumInRange(command.window_wiper, SwitchCommand::NO_CTL,
                     SwitchCommand::ON) &&
         enumInRange(command.light_brake, SwitchCommand::NO_CTL,
                     SwitchCommand::ON) &&
         enumInRange(command.light_position, SwitchCommand::NO_CTL,
                     SwitchCommand::ON) &&
         enumInRange(command.light_near, SwitchCommand::NO_CTL,
                     SwitchCommand::ON) &&
         enumInRange(command.light_far, SwitchCommand::NO_CTL,
                     SwitchCommand::ON) &&
         enumInRange(command.light_turn_left, SwitchCommand::NO_CTL,
                     SwitchCommand::ON) &&
         enumInRange(command.light_turn_right, SwitchCommand::NO_CTL,
                     SwitchCommand::ON) &&
         enumInRange(command.light_working_rear, SwitchCommand::NO_CTL,
                     SwitchCommand::ON) &&
         enumInRange(command.light_danger, SwitchCommand::NO_CTL,
                     SwitchCommand::ON) &&
         enumInRange(command.light_reverse, SwitchCommand::NO_CTL,
                     SwitchCommand::ON) &&
         enumInRange(command.light_double_flash, SwitchCommand::NO_CTL,
                     SwitchCommand::ON) &&
         enumInRange(command.light_front, SwitchCommand::NO_CTL,
                     SwitchCommand::ON) &&
         enumInRange(command.light_working_side, SwitchCommand::NO_CTL,
                     SwitchCommand::ON) &&
         enumInRange(command.light_fog, SwitchCommand::NO_CTL,
                     SwitchCommand::ON) &&
         enumInRange(command.diff_lock, SwitchCommand::NO_CTL,
                     SwitchCommand::ON);
}

} // namespace remote_protocol
