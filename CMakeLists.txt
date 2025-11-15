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
find_package(rr_common_base REQUIRED)

set(PROTO_FILES proto/inbound.proto)

# # Generate source and header from proto files 
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS 
  ${PROTO_FILES}
  PROTOC_OUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/generated"
)

# # Include directories for protobuf headers and generated headers
include_directories(${Protobuf_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} )

add_library(${PROJECT_NAME}_lib STATIC
  src/rr_udp_server_node.cpp 
  src/deserializer_fact.cpp
  src/joystick_deserializer.cpp
  ${PROTO_SRCS} 
  ${PROTO_HDRS})


ament_target_dependencies(
  ${PROJECT_NAME}_lib
  "rclcpp"
  "udp_msgs"
  "std_msgs"
  "rr_interfaces"
  "Protobuf"
  "rr_common_base"
)

# add_library(rr_udp_server::rr_udp_server_lib ALIAS rr_udp_server_lib)
# target_compile_features(rr_udp_server_lib PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)

install(TARGETS 
  # ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_package()
