#include "udp_channel.h"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>

#include <cassert>
#include <cstdint>

namespace {

// 获取 UDP 通道的本地回环地址
sockaddr_in localAddress(const UdpChannel &channel) {
  sockaddr_in address{};
  socklen_t address_size = sizeof(address);
  assert(getsockname(channel.fd(), reinterpret_cast<sockaddr *>(&address),
                     &address_size) == 0);
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  return address;
}

} // namespace

// 验证 UDP 绑定与回环收发
int main() {
  UdpChannel sender;
  UdpChannel receiver;
  assert(sender.bindPort(0));
  assert(receiver.bindPort(0));

  const std::uint8_t payload[]{1, 2, 3, 4};
  assert(sender.send(localAddress(receiver), payload, sizeof(payload)));

  pollfd descriptor{receiver.fd(), POLLIN, 0};
  assert(poll(&descriptor, 1, 1000) == 1);

  const auto datagram = receiver.receive();
  assert(datagram);
  assert(datagram->payload.size() == sizeof(payload));
  assert(datagram->payload.front() == 1);
  assert(datagram->payload.back() == 4);
}
