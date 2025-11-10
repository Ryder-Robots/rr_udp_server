cmake_minimum_required(VERSION 3.8)
project(rr_udp_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rr_interfaces REQUIRED)
find_package(udp_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Protobuf REQUIRED)

set(PROTO_FILES proto/inbound.proto)

# Generate source and header from proto files
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# Include directories for protobuf headers and generated headers
include_directories(${Protobuf_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

add_library(rr_udp_server_lib 
  src/rr_udp_server_lib.cpp 
  src/rr_udp_server_node.cpp 
  src/deserializer_fact.cpp
  ${PROTO_SRCS} 
  ${PROTO_HDRS})

add_library(rr_udp_server::rr_udp_server_lib ALIAS rr_udp_server_lib)
target_compile_features(rr_udp_server_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(rr_udp_server_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  rr_udp_server_lib
  "rclcpp"
  "udp_msgs"
  "std_msgs"
  "rr_interfaces"
  "Protobuf"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rr_udp_server_lib PRIVATE "RR_UDP_SERVER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS rr_udp_server_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(rr_udp_server_node src/main.cpp)
target_include_directories(rr_udp_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(rr_udp_server_node rr_udp_server_lib)

install(TARGETS rr_udp_server_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  rr_udp_server_lib
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
