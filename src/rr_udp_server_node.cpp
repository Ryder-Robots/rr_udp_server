#include "rr_udp_server/rr_udp_server_node.hpp"

using namespace rr_udp_server;

void RrUdpServerNode::init() {

    // get all the clients and create a reference
    // using safe approach
    rclcpp::Node* parent =  static_cast<rclcpp::Node*>(this);
    clients_ = factory_.get_deserializers(parent);

    // link callback so that it gets during packet arrival
}


void RrUdpServerNode::subscriber_cb(const udp_msgs::msg::UdpPacket packet) {
    bool available = false;
    int key = -1;
    std::shared_ptr<rr_udp_server::RrUdpDeserializer>  deserilizer = factory_.get_deserializer_key(packet, available, key);

    //TODO: perform some checking, make sure key is >= 0, and available is true
    deserilizer->clear();
    deserilizer->deserialize(packet);
    deserilizer->update_state(clients_[key]);
}
