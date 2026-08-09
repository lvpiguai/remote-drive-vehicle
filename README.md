# Remote Drive Vehicle

车端独立项目，包含 Apollo Cyber RT 组件、远控会话、UDP 通信和协议定义。

## 目录

```text
include/  # UDP、协议和远控会话头文件
src/      # 实现
proto/    # Cyber RT 控制指令和底盘状态消息定义
conf/     # 车端部署配置模板
apollo/   # Apollo Cyber RT 组件、DAG 和 launch
tests/    # 车端测试
```

`include/protocol/` 和 `src/protocol/` 是车端自己的协议实现。驾驶舱项目维护另一份
协议实现，两个项目不共享源码。

`RemoteDriveVehicleComponent` 是车端模块入口，负责初始化 UDP 通道、创建 Cyber
Writer、启动 UDP 工作线程并管理生命周期。底盘状态由 DAG reader 触发 `Proc()`
缓存，UDP 工作线程需要回传状态时读取最近一次缓存。`VehicleControlSession` 只负责
控制权仲裁、控制序号校验、远控退出和断联保护；它只判断控制指令是否允许转发，
不判断底盘是否真实执行成功。真实执行状态由后续底盘状态回传判断。

## 构建

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

## 运行

真实部署通过 Apollo Cyber RT 加载组件：

```bash
cyber_launch start apollo/remote_drive_vehicle.launch
```

组件通过 DAG 从车端运行目录加载配置：

```bash
/apollo/data/remote_drive/vehicle.pb.txt
```

仓库中的 `conf/vehicle.pb.txt.example` 是部署模板。每辆车使用相同的运行路径，
但配置内容属于当前车辆，至少需要设置唯一的 `vehicle_id`、UDP 监听端口和允许通信的
驾驶舱地址。可以在车辆宿主机上维护配置，再挂载到 Apollo 容器：

```text
/etc/remote-drive/vehicle.pb.txt
    -> /apollo/data/remote_drive/vehicle.pb.txt
```

如果所有车辆允许同一组驾驶舱接入，各车辆的 `cockpits` 可以相同；如果控制权限
不同，则为每辆车配置对应的驾驶舱子集。修改配置后需要重启组件生效。

当前 CMake 构建不链接 Apollo Cyber RT，只编译协议、UDP 通道和会话规则的本地测试。
