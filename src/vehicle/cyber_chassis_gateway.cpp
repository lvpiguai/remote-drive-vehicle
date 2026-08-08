#include "vehicle/cyber_chassis_gateway.h"

#include <iostream>
#include <utility>

// 保存车辆 ID，真实部署时用于绑定 Cyber RT 通道或组件配置
CyberChassisGateway::CyberChassisGateway(std::string vehicle_id)
    : vehicle_id_(std::move(vehicle_id)) {}

// 本地构建只保留发布边界；Apollo 环境由组件实现真实 Writer
bool CyberChassisGateway::publishControl(
    const remote_drive::protocol::RemoteDriveControlCommand &command,
    std::uint32_t sequence) {
  std::cout << "[CyberRT发布控制] vehicle=" << vehicle_id_ << " seq="
            << sequence << " steering=" << command.steering_angle()
            << " acc=" << command.accelerator_percent()
            << " brake=" << command.brake_percent() << '\n';
  return true;
}

// 返回最近一次通过 Cyber RT Reader 收到的底盘状态
std::optional<remote_drive::protocol::ChassisState>
CyberChassisGateway::latestState() const {
  return latest_state_;
}

// 更新底盘状态缓存
void CyberChassisGateway::updateState(
    const remote_drive::protocol::ChassisState &state) {
  latest_state_ = state;
}
