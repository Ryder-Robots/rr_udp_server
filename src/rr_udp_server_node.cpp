#include "rr_udp_server/rr_udp_server_node.hpp"

using namespace rr_udp_server;

void RrUdpServerNode::init(std::shared_ptr<RrPublisherFact>  factory) {
  RCLCPP_INFO(this->get_logger(), "creating udp_server node");
  // get all the clients and create a reference
  // using safe approach
  RCLCPP_INFO(this->get_logger(), "adding factory");
  factory_ = factory;

  RCLCPP_INFO(this->get_logger(), "creating subscriptions");
  rclcpp::SubscriptionOptions options;
  auto topic_callback =
      std::bind(&RrUdpServerNode::subscriber_cb, this, std::placeholders::_1);
  subscription_ = this->create_subscription<udp_msgs::msg::UdpPacket>(
      TOPIC_SUBSCRIBE, rclcpp::SensorDataQoS(), topic_callback, options);

  RCLCPP_DEBUG(this->get_logger(), "subscription created");
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
  RCLCPP_DEBUG(this->get_logger(), "recieved packet");
  tx_++;
  bool available = false;
  int key = -1;
  std::shared_ptr<rr_udp_server::RrPublisherInterface> publisher = factory_->get_publisher(packet, available);

  if (available) {
    std::shared_ptr<RrUdpDeserializer> deserilizer = publisher->get_deserializer();
    deserilizer->reset();

    // check for integrity before updating the state
    uint8_t status = RrUdpDeserializer::OK();
    if ((status = deserilizer->deserialize(packet)) !=
        RrUdpDeserializer::OK()) {
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


    RCLCPP_DEBUG(this->get_logger(), "attempting to send packet");
    publisher->update_state();
    rx_++;
    RCLCPP_DEBUG(this->get_logger(), "%ld packet publsihed", rx_);
  } else {
    RCLCPP_ERROR(
        this->get_logger(),
        "dropping packet: no deserializer available for this type of packet");
  }
}
