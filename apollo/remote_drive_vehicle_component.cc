#include "modules/remote_drive_vehicle/apollo/remote_drive_vehicle_component.h"

#include "cyber/component/component.h"

namespace remote_drive::vehicle {

bool RemoteDriveVehicleComponent::Init() {
  node_ = apollo::cyber::CreateNode("remote_drive_vehicle");
  control_writer_ =
      node_->CreateWriter<RemoteDriveControlCommand>("/remote_drive/control_cmd");
  return control_writer_ != nullptr;
}

bool RemoteDriveVehicleComponent::Proc(
    const std::shared_ptr<ChassisState> &state) {
  if (!state) return false;
  // 真实接入时在这里缓存底盘状态，并由 UDP 远控网关回传驾驶舱
  return true;
}

CYBER_REGISTER_COMPONENT(RemoteDriveVehicleComponent)

}  // namespace remote_drive::vehicle
