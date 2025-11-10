#include "rr_udp_server/deserializer_fact.hpp"

using namespace rr_udp_server;

std::shared_ptr<RrUdpDeserializer> RrDeserializerFact::get_deserializer_key(
    const udp_msgs::msg::UdpPacket udp_packet, bool& available, int& key) {

  //TODO: fill in gaps set all values to default, this is to check code      
  available = false;
  key = 0;
  return nullptr;
}

std::vector<rclcpp::ClientBase::SharedPtr>
RrDeserializerFact::get_deserializers(rclcpp::Node *node) {
  std::vector<rclcpp::ClientBase::SharedPtr> rv = {};

  // TODO: fill in gaps
  return rv;
}