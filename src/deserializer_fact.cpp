#include "rr_udp_server/deserializer_fact.hpp"

using namespace rr_udp_server;

std::shared_ptr<RrUdpDeserializer> RrDeserializerFact::get_deserializer_key(
    const udp_msgs::msg::UdpPacket udp_packet, bool& available, int& key) {
  // TODO: fill in gaps set all values to default, this is to check code
  available = false;
  key = 0;



  return nullptr;
}

/**
 * @fn get_deserializers
 * @brief returns back deserializers.
 */
std::vector<rclcpp::ClientBase::SharedPtr>
RrDeserializerFact::get_deserializers(std::shared_ptr<rclcpp::Node> node) {
  RCLCPP_INFO(node->get_logger(), "creating deserializers");
  std::vector<rclcpp::ClientBase::SharedPtr> rv = {
    static_cast<rclcpp::ClientBase::SharedPtr>(node->create_client<rr_interfaces::srv::Joy>(rr_constants_state_mgr::STATE_JOY_REQ.c_str()))
  };

  return rv;
}