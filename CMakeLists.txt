cmake_minimum_required(VERSION 3.16)

project(remote_drive_vehicle LANGUAGES CXX)

include(CTest)

find_package(Protobuf REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/remote_drive.proto)

add_library(vehicle_protocol
    ${PROTO_SRCS}
    src/protocol/udp_codec.cpp
)

target_include_directories(vehicle_protocol PUBLIC
    include
    ${CMAKE_CURRENT_BINARY_DIR}
    ${Protobuf_INCLUDE_DIRS}
)
target_link_libraries(vehicle_protocol PUBLIC ${Protobuf_LIBRARIES})

add_library(vehicle_core
    src/network/udp_channel.cpp
    src/vehicle/cyber_chassis_gateway.cpp
    src/vehicle/vehicle.cpp
    src/vehicle/vehicle_control_session.cpp
)

target_include_directories(vehicle_core PUBLIC
    include
)
target_link_libraries(vehicle_core PUBLIC vehicle_protocol)

add_executable(vehicle apps/main.cpp)
target_link_libraries(vehicle PRIVATE vehicle_core)

if(BUILD_TESTING)
  add_executable(udp_codec_test
      tests/protocol/udp_codec_test.cpp
  )
  target_link_libraries(udp_codec_test PRIVATE vehicle_core)
  add_test(NAME udp_codec_test COMMAND udp_codec_test)

  add_executable(udp_channel_test
      tests/network/udp_channel_test.cpp
  )
  target_link_libraries(udp_channel_test PRIVATE vehicle_core)
  add_test(NAME udp_channel_test COMMAND udp_channel_test)
endif()
