# Remote Drive Vehicle

车端独立项目，包含车端进程、远控会话、Cyber RT 底盘通信适配层和协议定义。

## 目录

```text
apps/     # vehicle 入口
include/  # 车端、协议和底盘通信网关头文件
src/      # 实现
proto/    # Cyber RT 控制指令和底盘状态消息定义
apollo/   # Apollo Cyber RT 组件注册骨架
tests/    # 车端测试
```

`include/protocol/` 和 `src/protocol/` 是车端自己的协议实现。驾驶舱项目维护另一份
协议实现，两个项目不共享源码。

车端远控核心不再依赖本地底盘模拟器。`VehicleControlSession` 只负责控制权仲裁、
控制序号校验、远控退出和断联保护；通过 `ChassisGateway` 将控制指令发布到底盘
通信层。真实部署时由 Apollo Cyber RT 组件订阅底盘状态、发布远控控制指令。

## 构建

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

## 运行

```bash
./build/vehicle truck_01 7006 \
  127.0.0.1:7005 \
  127.0.0.1:7015
```

当前 CMake 构建不链接 Apollo Cyber RT。`apollo/` 目录中的组件、DAG 和 launch 文件
用于说明真实 Apollo 环境中的接入方式。
