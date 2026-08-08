#pragma once

#include <cstdint>

// 车辆实际挡位和驾驶舱目标挡位
enum class GearInfo : std::uint8_t {
  NEUTRAL = 0,
  REVERSE_1 = 1,
  REVERSE_2 = 2,
  DRIVE_1 = 3,
  DRIVE_2 = 4,
  DRIVE_3 = 5,
};

// 车辆铲斗动作状态
enum class BucketInfo : std::uint8_t {
  BUCKET_UP = 0,
  BUCKET_DOWN = 1,
  BUCKET_KEEP = 2,
};

// 车端回传的实际驾驶模式
enum class DriveMode : std::uint8_t {
  MANUAL = 0,
  STANDBY = 1,
  REMOTE = 2,
  AUTO = 3,
};

// 驾驶舱请求进入、保持或退出远控的会话指令
enum class RemoteMode : std::uint8_t {
  REMOTE_NO_CONTROL = 0,
  REMOTE_ENTER = 1,
  REMOTE_EXIT = 2,
};

// 灯光和辅助功能使用的三态控制指令
enum class SwitchCommand : std::uint8_t {
  NO_CTL = 0,
  OFF = 1,
  ON = 2,
};

// 驾驶舱下发的远程控制指令
struct RemoteCtlCmd {
  char cockpit_id[20]{};
  double steering_angle = 0; // 度：-90=左满，0=回正，90=右满
  double acc_pedal = 0;      // 百分比：0=松开，100=踩到底
  double brake_pedal = 0;    // 百分比：0=松开，100=踩到底
  GearInfo gear = GearInfo::NEUTRAL;
  BucketInfo bucket_info = BucketInfo::BUCKET_KEEP;
  RemoteMode remoteMode = RemoteMode::REMOTE_NO_CONTROL;
  SwitchCommand parking = SwitchCommand::NO_CTL;
  SwitchCommand horn = SwitchCommand::NO_CTL;
  SwitchCommand spray = SwitchCommand::NO_CTL;
  SwitchCommand remote_emergency = SwitchCommand::NO_CTL;
  SwitchCommand window_wiper = SwitchCommand::NO_CTL;
  SwitchCommand light_brake = SwitchCommand::NO_CTL;
  SwitchCommand light_position = SwitchCommand::NO_CTL;
  SwitchCommand light_near = SwitchCommand::NO_CTL;
  SwitchCommand light_far = SwitchCommand::NO_CTL;
  SwitchCommand light_turn_left = SwitchCommand::NO_CTL;
  SwitchCommand light_turn_right = SwitchCommand::NO_CTL;
  SwitchCommand light_working_rear = SwitchCommand::NO_CTL;
  SwitchCommand light_danger = SwitchCommand::NO_CTL;
  SwitchCommand light_reverse = SwitchCommand::NO_CTL;
  SwitchCommand light_double_flash = SwitchCommand::NO_CTL;
  SwitchCommand light_front = SwitchCommand::NO_CTL;
  SwitchCommand light_working_side = SwitchCommand::NO_CTL;
  SwitchCommand light_fog = SwitchCommand::NO_CTL;
  SwitchCommand diff_lock = SwitchCommand::NO_CTL;
};

// 车端周期回传的实际车辆状态和控制权归属
struct RemoteDrivingState {
  char vehicle_id[20]{};
  char controller_id[20]{}; // 当前控制驾驶舱；空字符串表示无人控制
  double steering = 0; // 实际转角（度）：负=左转，0=回正，正=右转
  double speed = 0; // 实际速度：负=倒车，0=静止，正=前进
  DriveMode remoteMode = DriveMode::AUTO;
  GearInfo gear = GearInfo::NEUTRAL;
  BucketInfo bucket = BucketInfo::BUCKET_KEEP;
  bool parking = true;
  bool horn = false;
  bool spray = false;
  bool emergency = false;
  bool window_wiper = false;
  bool light_brake = false;
  bool light_position = false;
  bool light_near = false;
  bool light_far = false;
  bool light_turn_left = false;
  bool light_turn_right = false;
  bool light_working_rear = false;
  bool light_danger = false;
  bool light_reverse = false;
  bool light_double_flash = false;
  bool light_front = false;
  bool light_working_side = false;
  bool light_fog = false;
  bool diff_lock = false;
};
