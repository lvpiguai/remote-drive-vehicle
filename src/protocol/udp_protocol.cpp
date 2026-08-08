#include "protocol/udp_protocol.h"

#include "remote_drive.pb.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

namespace remote_protocol {
namespace {

namespace pb = remote_drive::protocol;

// 将 protobuf 字符串安全写入定长协议字段
template <std::size_t Size>
void copyToField(const std::string &value, char (&field)[Size]) {
  std::memset(field, 0, Size);
  std::memcpy(field, value.data(), std::min(value.size(), Size - 1));
}

// 定长 ID 字段转字符串
template <std::size_t Size>
std::string fieldToString(const char (&field)[Size]) {
  return std::string(field, strnlen(field, Size));
}

// 校验写入定长协议字段的 ID
bool validId(const std::string &value) {
  return !value.empty() && value.size() < sizeof(RemoteDrivingState{}.vehicle_id);
}

// 校验可为空的控制者 ID
bool validOptionalId(const std::string &value) {
  return value.size() < sizeof(RemoteDrivingState{}.controller_id);
}

pb::Gear toProto(GearInfo gear) {
  switch (gear) {
    case GearInfo::NEUTRAL: return pb::GEAR_NEUTRAL;
    case GearInfo::REVERSE_1: return pb::GEAR_REVERSE_1;
    case GearInfo::REVERSE_2: return pb::GEAR_REVERSE_2;
    case GearInfo::DRIVE_1: return pb::GEAR_DRIVE_1;
    case GearInfo::DRIVE_2: return pb::GEAR_DRIVE_2;
    case GearInfo::DRIVE_3: return pb::GEAR_DRIVE_3;
  }
  return pb::GEAR_NEUTRAL;
}

bool fromProto(pb::Gear gear, GearInfo &output) {
  switch (gear) {
    case pb::GEAR_NEUTRAL: output = GearInfo::NEUTRAL; return true;
    case pb::GEAR_REVERSE_1: output = GearInfo::REVERSE_1; return true;
    case pb::GEAR_REVERSE_2: output = GearInfo::REVERSE_2; return true;
    case pb::GEAR_DRIVE_1: output = GearInfo::DRIVE_1; return true;
    case pb::GEAR_DRIVE_2: output = GearInfo::DRIVE_2; return true;
    case pb::GEAR_DRIVE_3: output = GearInfo::DRIVE_3; return true;
    default: return false;
  }
}

pb::Bucket toProto(BucketInfo bucket) {
  switch (bucket) {
    case BucketInfo::BUCKET_UP: return pb::BUCKET_UP;
    case BucketInfo::BUCKET_DOWN: return pb::BUCKET_DOWN;
    case BucketInfo::BUCKET_KEEP: return pb::BUCKET_KEEP;
  }
  return pb::BUCKET_KEEP;
}

bool fromProto(pb::Bucket bucket, BucketInfo &output) {
  switch (bucket) {
    case pb::BUCKET_UP: output = BucketInfo::BUCKET_UP; return true;
    case pb::BUCKET_DOWN: output = BucketInfo::BUCKET_DOWN; return true;
    case pb::BUCKET_KEEP: output = BucketInfo::BUCKET_KEEP; return true;
    default: return false;
  }
}

pb::DriveMode toProto(DriveMode mode) {
  switch (mode) {
    case DriveMode::MANUAL: return pb::DRIVE_MODE_MANUAL;
    case DriveMode::STANDBY: return pb::DRIVE_MODE_STANDBY;
    case DriveMode::REMOTE: return pb::DRIVE_MODE_REMOTE;
    case DriveMode::AUTO: return pb::DRIVE_MODE_AUTO;
  }
  return pb::DRIVE_MODE_AUTO;
}

bool fromProto(pb::DriveMode mode, DriveMode &output) {
  switch (mode) {
    case pb::DRIVE_MODE_MANUAL: output = DriveMode::MANUAL; return true;
    case pb::DRIVE_MODE_STANDBY: output = DriveMode::STANDBY; return true;
    case pb::DRIVE_MODE_REMOTE: output = DriveMode::REMOTE; return true;
    case pb::DRIVE_MODE_AUTO: output = DriveMode::AUTO; return true;
    default: return false;
  }
}

pb::RemoteMode toProto(RemoteMode mode) {
  switch (mode) {
    case RemoteMode::REMOTE_NO_CONTROL: return pb::REMOTE_MODE_NO_CONTROL;
    case RemoteMode::REMOTE_ENTER: return pb::REMOTE_MODE_ENTER;
    case RemoteMode::REMOTE_EXIT: return pb::REMOTE_MODE_EXIT;
  }
  return pb::REMOTE_MODE_NO_CONTROL;
}

bool fromProto(pb::RemoteMode mode, RemoteMode &output) {
  switch (mode) {
    case pb::REMOTE_MODE_NO_CONTROL:
      output = RemoteMode::REMOTE_NO_CONTROL;
      return true;
    case pb::REMOTE_MODE_ENTER: output = RemoteMode::REMOTE_ENTER; return true;
    case pb::REMOTE_MODE_EXIT: output = RemoteMode::REMOTE_EXIT; return true;
    default: return false;
  }
}

pb::SwitchCommand toProto(SwitchCommand command) {
  switch (command) {
    case SwitchCommand::NO_CTL: return pb::SWITCH_NO_CONTROL;
    case SwitchCommand::OFF: return pb::SWITCH_OFF;
    case SwitchCommand::ON: return pb::SWITCH_ON;
  }
  return pb::SWITCH_NO_CONTROL;
}

bool fromProto(pb::SwitchCommand command, SwitchCommand &output) {
  switch (command) {
    case pb::SWITCH_NO_CONTROL: output = SwitchCommand::NO_CTL; return true;
    case pb::SWITCH_OFF: output = SwitchCommand::OFF; return true;
    case pb::SWITCH_ON: output = SwitchCommand::ON; return true;
    default: return false;
  }
}

void fillHeader(pb::UdpPacket &packet, pb::MessageType type,
                std::uint32_t sequence) {
  packet.set_magic(kMagic);
  packet.set_protocol_version(kProtocolVersion);
  packet.set_message_type(type);
  packet.set_sequence(sequence);
}

UdpPacketBytes serialize(const pb::UdpPacket &packet) {
  std::string bytes;
  packet.SerializeToString(&bytes);
  return UdpPacketBytes(bytes.begin(), bytes.end());
}

void writeControl(const RemoteCtlCmd &command,
                  pb::RemoteDriveControlCommand &output) {
  output.set_cockpit_id(fieldToString(command.cockpit_id));
  output.set_steering_angle(command.steering_angle);
  output.set_accelerator_percent(command.acc_pedal);
  output.set_brake_percent(command.brake_pedal);
  output.set_gear(toProto(command.gear));
  output.set_bucket(toProto(command.bucket_info));
  output.set_remote_mode(toProto(command.remoteMode));
  output.set_parking(toProto(command.parking));
  output.set_horn(toProto(command.horn));
  output.set_spray(toProto(command.spray));
  output.set_remote_emergency(toProto(command.remote_emergency));
  output.set_window_wiper(toProto(command.window_wiper));
  output.set_light_brake(toProto(command.light_brake));
  output.set_light_position(toProto(command.light_position));
  output.set_light_near(toProto(command.light_near));
  output.set_light_far(toProto(command.light_far));
  output.set_light_turn_left(toProto(command.light_turn_left));
  output.set_light_turn_right(toProto(command.light_turn_right));
  output.set_light_working_rear(toProto(command.light_working_rear));
  output.set_light_danger(toProto(command.light_danger));
  output.set_light_reverse(toProto(command.light_reverse));
  output.set_light_double_flash(toProto(command.light_double_flash));
  output.set_light_front(toProto(command.light_front));
  output.set_light_working_side(toProto(command.light_working_side));
  output.set_light_fog(toProto(command.light_fog));
  output.set_diff_lock(toProto(command.diff_lock));
}

bool readControl(const pb::RemoteDriveControlCommand &input,
                 RemoteCtlCmd &command) {
  if (!validId(input.cockpit_id()) ||
      !fromProto(input.gear(), command.gear) ||
      !fromProto(input.bucket(), command.bucket_info) ||
      !fromProto(input.remote_mode(), command.remoteMode) ||
      !fromProto(input.parking(), command.parking) ||
      !fromProto(input.horn(), command.horn) ||
      !fromProto(input.spray(), command.spray) ||
      !fromProto(input.remote_emergency(), command.remote_emergency) ||
      !fromProto(input.window_wiper(), command.window_wiper) ||
      !fromProto(input.light_brake(), command.light_brake) ||
      !fromProto(input.light_position(), command.light_position) ||
      !fromProto(input.light_near(), command.light_near) ||
      !fromProto(input.light_far(), command.light_far) ||
      !fromProto(input.light_turn_left(), command.light_turn_left) ||
      !fromProto(input.light_turn_right(), command.light_turn_right) ||
      !fromProto(input.light_working_rear(), command.light_working_rear) ||
      !fromProto(input.light_danger(), command.light_danger) ||
      !fromProto(input.light_reverse(), command.light_reverse) ||
      !fromProto(input.light_double_flash(), command.light_double_flash) ||
      !fromProto(input.light_front(), command.light_front) ||
      !fromProto(input.light_working_side(), command.light_working_side) ||
      !fromProto(input.light_fog(), command.light_fog) ||
      !fromProto(input.diff_lock(), command.diff_lock)) {
    return false;
  }

  copyToField(input.cockpit_id(), command.cockpit_id);
  command.steering_angle = input.steering_angle();
  command.acc_pedal = input.accelerator_percent();
  command.brake_pedal = input.brake_percent();
  return validate(command);
}

void writeState(const RemoteDrivingState &state, pb::ChassisState &output) {
  output.set_vehicle_id(fieldToString(state.vehicle_id));
  output.set_controller_id(fieldToString(state.controller_id));
  output.set_steering_angle(state.steering);
  output.set_speed(state.speed);
  output.set_drive_mode(toProto(state.remoteMode));
  output.set_gear(toProto(state.gear));
  output.set_bucket(toProto(state.bucket));
  output.set_parking(state.parking);
  output.set_horn(state.horn);
  output.set_spray(state.spray);
  output.set_emergency(state.emergency);
  output.set_window_wiper(state.window_wiper);
  output.set_light_brake(state.light_brake);
  output.set_light_position(state.light_position);
  output.set_light_near(state.light_near);
  output.set_light_far(state.light_far);
  output.set_light_turn_left(state.light_turn_left);
  output.set_light_turn_right(state.light_turn_right);
  output.set_light_working_rear(state.light_working_rear);
  output.set_light_danger(state.light_danger);
  output.set_light_reverse(state.light_reverse);
  output.set_light_double_flash(state.light_double_flash);
  output.set_light_front(state.light_front);
  output.set_light_working_side(state.light_working_side);
  output.set_light_fog(state.light_fog);
  output.set_diff_lock(state.diff_lock);
}

bool readState(const pb::ChassisState &input, RemoteDrivingState &state) {
  if (!validId(input.vehicle_id()) ||
      !validOptionalId(input.controller_id()) ||
      !fromProto(input.drive_mode(), state.remoteMode) ||
      !fromProto(input.gear(), state.gear) ||
      !fromProto(input.bucket(), state.bucket) ||
      !std::isfinite(input.steering_angle()) ||
      !std::isfinite(input.speed())) {
    return false;
  }

  copyToField(input.vehicle_id(), state.vehicle_id);
  copyToField(input.controller_id(), state.controller_id);
  state.steering = input.steering_angle();
  state.speed = input.speed();
  state.parking = input.parking();
  state.horn = input.horn();
  state.spray = input.spray();
  state.emergency = input.emergency();
  state.window_wiper = input.window_wiper();
  state.light_brake = input.light_brake();
  state.light_position = input.light_position();
  state.light_near = input.light_near();
  state.light_far = input.light_far();
  state.light_turn_left = input.light_turn_left();
  state.light_turn_right = input.light_turn_right();
  state.light_working_rear = input.light_working_rear();
  state.light_danger = input.light_danger();
  state.light_reverse = input.light_reverse();
  state.light_double_flash = input.light_double_flash();
  state.light_front = input.light_front();
  state.light_working_side = input.light_working_side();
  state.light_fog = input.light_fog();
  state.diff_lock = input.diff_lock();
  return true;
}

} // namespace

UdpPacketBytes encodeHeartbeat(const std::string &vehicle_id,
                               std::uint32_t sequence) {
  pb::UdpPacket packet;
  fillHeader(packet, pb::MESSAGE_TYPE_HEARTBEAT, sequence);
  packet.mutable_heartbeat()->set_vehicle_id(vehicle_id);
  return serialize(packet);
}

UdpPacketBytes encodeControlCommand(const RemoteCtlCmd &command,
                                    std::uint32_t sequence) {
  pb::UdpPacket packet;
  fillHeader(packet, pb::MESSAGE_TYPE_CONTROL_CMD, sequence);
  writeControl(command, *packet.mutable_control());
  return serialize(packet);
}

UdpPacketBytes encodeDrivingState(const RemoteDrivingState &state,
                                  std::uint32_t sequence) {
  pb::UdpPacket packet;
  fillHeader(packet, pb::MESSAGE_TYPE_VEHICLE_STATE, sequence);
  writeState(state, *packet.mutable_state());
  return serialize(packet);
}

std::optional<DecodedPacket> decodePacket(const std::uint8_t *data,
                                          std::size_t size) {
  if (!data || size == 0) return std::nullopt;

  pb::UdpPacket packet;
  if (!packet.ParseFromArray(data, static_cast<int>(size)) ||
      packet.magic() != kMagic ||
      packet.protocol_version() != kProtocolVersion) {
    return std::nullopt;
  }

  DecodedPacket decoded{};
  decoded.sequence = packet.sequence();
  switch (packet.body_case()) {
    case pb::UdpPacket::kHeartbeat:
      if (packet.message_type() != pb::MESSAGE_TYPE_HEARTBEAT ||
          !validId(packet.heartbeat().vehicle_id())) {
        return std::nullopt;
      }
      decoded.body = PacketBody::HEARTBEAT;
      decoded.vehicle_id = packet.heartbeat().vehicle_id();
      return decoded;
    case pb::UdpPacket::kControl:
      if (packet.message_type() != pb::MESSAGE_TYPE_CONTROL_CMD ||
          !readControl(packet.control(), decoded.control)) {
        return std::nullopt;
      }
      decoded.body = PacketBody::CONTROL_CMD;
      return decoded;
    case pb::UdpPacket::kState:
      if (packet.message_type() != pb::MESSAGE_TYPE_VEHICLE_STATE ||
          !readState(packet.state(), decoded.state)) {
        return std::nullopt;
      }
      decoded.body = PacketBody::VEHICLE_STATE;
      decoded.vehicle_id = packet.state().vehicle_id();
      return decoded;
    default:
      return std::nullopt;
  }
}

bool validate(const RemoteCtlCmd &command) {
  return fieldToString(command.cockpit_id).size() > 0 &&
         std::isfinite(command.steering_angle) &&
         std::isfinite(command.acc_pedal) &&
         std::isfinite(command.brake_pedal) &&
         command.steering_angle >= -90.0 && command.steering_angle <= 90.0 &&
         command.acc_pedal >= 0.0 && command.acc_pedal <= 100.0 &&
         command.brake_pedal >= 0.0 && command.brake_pedal <= 100.0;
}

} // namespace remote_protocol
