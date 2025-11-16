#include "rr_udp_server/rr_publisher_fact.hpp"

using namespace rr_udp_server;

/*
 * Uses protobuf to define the map of serializers that it should be getting.
 * Note that order of the publishers is very important.
 */
std::shared_ptr<RrPublisherInterface> RrPublisherFact::get_publisher(
    const udp_msgs::msg::UdpPacket udp_packet, bool& available) {
  available = false;
  std::shared_ptr<RrPublisherInterface> publisher = nullptr;

  InboundMessage packet;
  packet.ParseFromArray(udp_packet.data.data(), udp_packet.data.size());

  if (packet.has_joystick()) {
    available = true;
    publisher = publishers_[0];  // key for Joystick must be 0
  }

  return publisher;
}

/*
 * returns constant reference to the publishers. The main class should be able to interate over
 * this and add them to the executor.
 */
const std::vector<std::shared_ptr<RrPublisherInterface>> RrPublisherFact::get_publishers() {
    return publishers_;
}