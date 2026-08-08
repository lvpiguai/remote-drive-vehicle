#include "simulation/chassis_simulator.h"

#include <algorithm>
#include <cmath>

namespace {

// 将三态开关指令应用到实际布尔状态
void applySwitch(SwitchCommand command, bool &state) {
  if (command == SwitchCommand::ON) state = true;
  if (command == SwitchCommand::OFF) state = false;
}

}  // namespace

// 返回当前底盘状态快照
const VehicleStatus &ChassisSimulator::status() const { return status_; }

// 进入远控模式并初始化安全状态
void ChassisSimulator::enterRemote() {
  status_.drive_mode = DriveMode::REMOTE;
  safeStop();
}

// 退出远控模式并恢复安全状态
void ChassisSimulator::exitRemote() {
  status_.drive_mode = DriveMode::AUTO;
  safeStop();
}

// 应用经过网关校验的驾驶指令
void ChassisSimulator::applyCommand(const RemoteCtlCmd &command) {
  status_.steering_angle = command.steering_angle;
  status_.brake_pedal = command.brake_pedal;
  status_.gear = command.gear;
  status_.bucket = command.bucket_info;
  applySwitch(command.parking, status_.parking);
  applySwitch(command.horn, status_.horn);
  applySwitch(command.spray, status_.spray);
  applySwitch(command.remote_emergency, status_.remote_emergency);
  applySwitch(command.window_wiper, status_.window_wiper);
  applySwitch(command.light_brake, status_.light_brake);
  applySwitch(command.light_position, status_.light_position);
  applySwitch(command.light_near, status_.light_near);
  applySwitch(command.light_far, status_.light_far);
  applySwitch(command.light_turn_left, status_.light_turn_left);
  applySwitch(command.light_turn_right, status_.light_turn_right);
  applySwitch(command.light_working_rear, status_.light_working_rear);
  applySwitch(command.light_danger, status_.light_danger);
  applySwitch(command.light_reverse, status_.light_reverse);
  applySwitch(command.light_double_flash, status_.light_double_flash);
  applySwitch(command.light_front, status_.light_front);
  applySwitch(command.light_working_side, status_.light_working_side);
  applySwitch(command.light_fog, status_.light_fog);
  applySwitch(command.diff_lock, status_.diff_lock);

  // 驻车、急停、制动或空挡时不输出油门
  if (status_.parking || status_.remote_emergency) {
    status_.acc_pedal = 0;
    status_.brake_pedal = 100;
  } else {
    status_.acc_pedal =
        (command.brake_pedal > 0 || command.gear == GearInfo::NEUTRAL)
            ? 0
            : command.acc_pedal;
  }
}

// 按当前控制量推进模拟车速
void ChassisSimulator::tick() {
  if (status_.gear == GearInfo::NEUTRAL) {
    status_.speed = 0;
    return;
  }

  double speed = std::abs(status_.speed);
  if (status_.brake_pedal > 0) {
    speed = std::max(0.0, speed - status_.brake_pedal * 0.02);
  } else if ((status_.gear >= GearInfo::DRIVE_1 && status_.speed < 0) ||
             (status_.gear >= GearInfo::REVERSE_1 &&
              status_.gear <= GearInfo::REVERSE_2 && status_.speed > 0)) {
    speed = std::max(0.0, speed - 1.0);
  } else {
    speed = std::min(40.0, speed + status_.acc_pedal * 0.01);
  }
  const bool reversing = status_.gear == GearInfo::REVERSE_1 ||
                         status_.gear == GearInfo::REVERSE_2;
  status_.speed = reversing ? -speed : speed;
}

// 将底盘重置为制动空挡状态
void ChassisSimulator::safeStop() {
  status_.steering_angle = 0;
  status_.acc_pedal = 0;
  status_.brake_pedal = 100;
  status_.gear = GearInfo::NEUTRAL;
  status_.parking = true;
  status_.speed = 0;
}
