cmake_minimum_required(VERSION 3.16)

project(remote_drive_vehicle LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

add_library(vehicle_protocol
    src/protocol/binary_codec.cpp
)

target_include_directories(vehicle_protocol PUBLIC
    include
)

add_library(vehicle_core
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
  add_executable(binary_protocol_test
      tests/protocol/binary_protocol_test.cpp
  )
  target_link_libraries(binary_protocol_test PRIVATE vehicle_core)
  add_test(NAME binary_protocol_test COMMAND binary_protocol_test)
endif()
