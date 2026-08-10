#include "udp_channel.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

constexpr std::size_t kReceiveBufferSize = 256;

} // namespace

// 关闭 UDP socket
UdpChannel::~UdpChannel() {
  if (fd_ >= 0)
    close(fd_);
}

// 创建并绑定 UDP socket
bool UdpChannel::bindPort(std::uint16_t port) {
  if (fd_ >= 0)
    return false;

  // 创建 socket
  const int next_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (next_fd < 0)
    return false;

  // 绑定监听端口
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  address.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(next_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) <
      0) {
    close(next_fd);
    return false;
  }

  fd_ = next_fd;
  return true;
}

// 接收 UDP 数据报
std::optional<UdpDatagram> UdpChannel::receive() {
  if (fd_ < 0) return std::nullopt;

  // 读取负载和来源地址
  UdpDatagram datagram;
  datagram.payload.resize(kReceiveBufferSize);
  socklen_t source_size = sizeof(datagram.source);
  const ssize_t size = recvfrom(
      fd_, datagram.payload.data(), datagram.payload.size(), MSG_DONTWAIT,
      reinterpret_cast<sockaddr *>(&datagram.source), &source_size);
  if (size <= 0) {
    return std::nullopt;
  }

  // 保留实际负载长度
  datagram.payload.resize(static_cast<std::size_t>(size));
  return datagram;
}

// 发送 UDP 数据报
bool UdpChannel::send(const sockaddr_in &destination, const void *data,
                      std::size_t size) const {
  if (fd_ < 0)
    return false;

  return sendto(fd_, data, size, 0,
                reinterpret_cast<const sockaddr *>(&destination),
                sizeof(destination)) == static_cast<ssize_t>(size);
}
