#include "rr_udp_server/deserializer_fact.hpp"

using namespace rr_udp_server;

std::shared_ptr<RrUdpDeserializer> RrDeserializerFact::get_deserializer_key(
    const udp_msgs::msg::UdpPacket udp_packet, bool& available, int& key) {

  available = false;
  key = -1;
  std::shared_ptr<RrUdpDeserializer> deserializer = nullptr;
  
  InboundMessage packet;
  packet.ParseFromArray(udp_packet.data.data(), udp_packet.data.size());

  if (packet.has_joystick()) {
    key = 0;
    available = true;
    deserializer = deserializers_[key];
  }

  return deserializer;
}

/**
 * @fn get_deserializers
 * @brief returns back deserializers.
 */
std::vector<rclcpp::ClientBase::SharedPtr>
RrDeserializerFact::get_deserializers(std::shared_ptr<rclcpp::Node> node) {
  RCLCPP_INFO(node->get_logger(), "creating deserializers");

  // add deserializers to the list, note that they will need to be in the same order as the returned clients above
  std::vector<rclcpp::ClientBase::SharedPtr> rv = {
    // key = 0
    static_cast<rclcpp::ClientBase::SharedPtr>(node->create_client<rr_interfaces::srv::Joy>(rr_constants_state_mgr::STATE_JOY_REQ.c_str()))
  };

  return rv;
}