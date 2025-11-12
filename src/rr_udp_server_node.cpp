#include "rr_udp_server/rr_udp_server_node.hpp"

using namespace rr_udp_server;

void RrUdpServerNode::init() {
  // get all the clients and create a reference
  // using safe approach
  rclcpp::Node* parent = static_cast<rclcpp::Node*>(this);
  clients_ = factory_.get_deserializers(parent);

  // TODO: link callback so that it gets during packet arrival
}

/**
 * called for each inbound packet, when packet recieved:
 *   1. Attempts to retrieve deserializer from the deserializer-factory
 *   2. if available, then
 *   3.    deserializer is reset.
 *   4.    packet is deserializecd
 *   5.    packet is sent to state service.
 */
void RrUdpServerNode::subscriber_cb(const udp_msgs::msg::UdpPacket packet) {
  tx_++;
  bool available = false;
  int key = -1;
  std::shared_ptr<rr_udp_server::RrUdpDeserializer> deserilizer =
      factory_.get_deserializer_key(packet, available, key);

  if (available) {
    deserilizer->reset();

    // check for integrity before updating the state
    uint8_t status = RrUdpDeserializer::OK();
    if ((status = deserilizer->deserialize(packet)) != RrUdpDeserializer::OK()) {
      if (status == RrUdpDeserializer::ERROR()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "dropping packet: integrity errors found in inbound packet: %s",
            deserilizer->err_str().c_str());
        err_++;
        return;
      } else {
        RCLCPP_WARN(
            this->get_logger(),
            "keeping packet: integrity warnings found in inbound packet: %s",
            deserilizer->err_str().c_str());
      }
    }

    // To allow ROS2 middleware to keep reference count to services within the
    // node, services are sent back to the node via factory. Therefore they need
    // to be sent to the deserializer to send request to state service
    deserilizer->update_state(clients_[key]);
    rx_++;
  } else {
    RCLCPP_ERROR(
        this->get_logger(),
        "dropping packet: no deserializer available for this type of packet");
  }
}
