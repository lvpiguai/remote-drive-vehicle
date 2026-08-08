#pragma once

#include <memory>

#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "modules/remote_drive_vehicle/proto/remote_drive_chassis.pb.h"

namespace remote_drive::vehicle {

class RemoteDriveVehicleComponent final
    : public apollo::cyber::Component<ChassisState> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<ChassisState> &state) override;

 private:
  std::shared_ptr<apollo::cyber::Node> node_;
  std::shared_ptr<apollo::cyber::Writer<RemoteDriveControlCommand>>
      control_writer_;
};

}  // namespace remote_drive::vehicle
