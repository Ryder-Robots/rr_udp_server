#include "rr_udp_server/rr_udp_server_lib.hpp"

namespace rr_udp_server
{

void RrUdpServerLib::init() {
    auto topic_callback =  std::bind(&RrUdpServerLib::subscriber, this, std::placeholders::_1);
    subscription_ = this->create_subscription<RR_UDP_SVR_MSG_CLASS>(RR_UDP_SVR_READ_TOPIC, 10, topic_callback);
}

RrUdpServerLib::~RrUdpServerLib() {}

/**
 * Read udp messages
 */
void RrUdpServerLib::subscriber(const udp_msgs::msg::UdpPacket::SharedPtr packet) {
    char* char_ptr = reinterpret_cast<char*>(packet->data.data());
    RCLCPP_DEBUG(logger_, "received message UDP msg: %s", char_ptr);
}

void RrUdpServerLib::publisher() {}

}  // namespace rr_udp_server
