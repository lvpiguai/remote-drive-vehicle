#pragma once

#include <netinet/in.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

// UDP 接收结果
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
  UdpChannel(UdpChannel &&) = delete;
  UdpChannel &operator=(UdpChannel &&) = delete;

  // 创建并绑定本地 UDP 端口
  bool bindPort(std::uint16_t port);

  // 非阻塞接收一条原始数据报
  std::optional<UdpDatagram> receive();

  // 向指定地址发送一条完整原始数据报
  bool send(const sockaddr_in &destination, const void *data,
            std::size_t size) const;

  // 返回供 poll 使用的 socket
  int fd() const { return fd_; }

private:
  int fd_ = -1;
};
