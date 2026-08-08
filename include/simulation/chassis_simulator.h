#pragma once

#include "protocol/remote_control_protocol.h"

// 底盘模拟器反馈的车辆状态
struct VehicleStatus {
  DriveMode drive_mode = DriveMode::AUTO;
  double steering_angle = 0;
  double acc_pedal = 0;
  double brake_pedal = 100;
  GearInfo gear = GearInfo::NEUTRAL;
  BucketInfo bucket = BucketInfo::BUCKET_KEEP;
  bool parking = true;
  bool horn = false;
  bool spray = false;
  bool remote_emergency = false;
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
  double speed = 0;
};

class ChassisSimulator {
 public:
  // 读取底盘状态
  const VehicleStatus &status() const;

  // 进入远控
  void enterRemote();

  // 退出远控
  void exitRemote();

  // 应用远程控制指令
  void applyCommand(const RemoteCtlCmd &command);

  // 推进模拟状态
  void tick();

 private:
  // 执行安全停车
  void safeStop();

  VehicleStatus status_{};
};
