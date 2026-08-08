# Remote Drive Vehicle

车端独立项目，包含车端进程、远控会话和本地底盘模拟。

## 目录

```text
apps/     # vehicle 入口
include/  # 车端、协议和底盘模拟头文件
src/      # 实现
tests/    # 车端测试
```

`include/protocol/` 和 `src/protocol/` 是车端自己的协议实现。驾驶舱项目维护另一份
协议实现，两个项目不共享源码。

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
