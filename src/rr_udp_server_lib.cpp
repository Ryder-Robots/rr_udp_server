#include "rr_udp_server/rr_udp_server_lib.hpp"

namespace rr_udp_server
{

void RrUdpServerLib::init() {

}

RrUdpServerLib::~RrUdpServerLib() {}

/**
 * Read udp messages
 */
void RrUdpServerLib::subscriber(const udp_msgs::msg::UdpPacket::SharedPtr packet) {
    RCLCPP_DEBUG(logger_, "received message image msg");
}

void RrUdpServerLib::publisher() {}

}  // namespace rr_udp_server
