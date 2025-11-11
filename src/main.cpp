#include "rr_udp_server/rr_udp_server_node.hpp"

using namespace rr_udp_server;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<RrUdpServerLib>());
  rclcpp::shutdown();
  return 0;
}
