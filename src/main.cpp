#include "rr_udp_server/rr_udp_server_node.hpp"

using namespace rr_udp_server;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<RrUdpServerNode> node = std::make_shared<RrUdpServerNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
