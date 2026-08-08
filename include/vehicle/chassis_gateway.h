#pragma once

#include <cstdint>
#include <optional>

#include "remote_drive.pb.h"

// 底盘通信网关：隔离远控会话与真实 Cyber RT 通信
class ChassisGateway {
 public:
  virtual ~ChassisGateway() = default;

  // 发布一条远控指令到底盘控制通道
  virtual bool publishControl(
      const remote_drive::protocol::RemoteDriveControlCommand &command,
      std::uint32_t sequence) = 0;

  // 返回最近一次底盘状态；未收到状态时返回空
  virtual std::optional<remote_drive::protocol::ChassisState>
  latestState() const = 0;
};
