#pragma once

#include <cstdint>
#include <optional>

#include "protocol/remote_control_protocol.h"

// 底盘通信网关：隔离远控会话与真实 Cyber RT 通信
class ChassisGateway {
 public:
  virtual ~ChassisGateway() = default;

  // 发布一条远控指令到底盘控制通道
  virtual bool publishControl(const RemoteCtlCmd &command,
                              std::uint32_t sequence) = 0;

  // 返回最近一次底盘状态；未收到状态时返回空
  virtual std::optional<RemoteDrivingState> latestState() const = 0;
};
