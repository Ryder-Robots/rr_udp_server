#include "rr_udp_server/rr_udp_server_node.hpp"


using namespace rr_udp_server;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<RrUdpServerNode> node = std::make_shared<RrUdpServerNode>();
  std::shared_ptr<RrPublisherFact> fact = std::make_shared<RrPublisherFact>();
  rclcpp::executors::SingleThreadedExecutor executor;
  node->init(fact);
  executor.add_node(node);

  // return nodes from factory, initlize, and add them to executor.
  for (std::shared_ptr<RrPublisherInterface> publisher : fact->get_publishers()) {
    executor.add_node(publisher->get_node());
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
