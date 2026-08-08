#pragma once

#include <netinet/in.h>

#include <cstddef>
#include <cstdint>
#include <vector>

// 一条原始 UDP 数据报，包含负载和来源地址
struct UdpDatagram {
  std::vector<std::uint8_t> payload;
  sockaddr_in source{};
};

// UDP 通道：管理 socket 生命周期和原始数据报收发
class UdpChannel {
public:
  UdpChannel() = default;
  ~UdpChannel();

  UdpChannel(const UdpChannel &) = delete;
  UdpChannel &operator=(const UdpChannel &) = delete;

  // 创建并绑定本地 UDP 端口
  bool bindPort(std::uint16_t port);

  // 非阻塞接收一条原始数据报；当前无数据时返回 false
  bool receive(UdpDatagram &datagram);

  // 向指定地址发送一条完整原始数据报
  bool send(const sockaddr_in &destination, const void *data,
            std::size_t size) const;

  // 返回供 poll 使用的 socket
  int fd() const { return fd_; }

private:
  int fd_ = -1;
};
