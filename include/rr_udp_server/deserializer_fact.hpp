#ifndef DESEIALIZER_FACT_HPP
#define DESEIALIZER_FACT_HPP

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rr_udp_server/deserializer.hpp"
#include "udp_msgs/msg/udp_packet.hpp"

namespace rr_udp_server {
class RrDeserializerFact {
 public:
  ~RrDeserializerFact() = default;

  /**
   * @fn get_deserializer_key
   * @param udp_packet inbound packet
   * @param true if key is available
   * @param key, reference of key to use with deserializder
   * @return key to with deserilizer vector
   */
  std::shared_ptr<RrUdpDeserializer> get_deserializer_key(
      const udp_msgs::msg::UdpPacket udp_packet, bool& available, int& key);

  /**
   * @fn get_deserializers
   * @param node - reference to "this" from calling class
   * @return list of service clients, these will submit the message to the
   * service
   *
   * Called during node initilzation, provides list of clients, so that state of
   * each client remains during life cycle of node.
   */
  std::vector<rclcpp::ClientBase::SharedPtr> get_deserializers(
      std::shared_ptr<rclcpp::Node> node);
};
} // namespace rr_udp_server

#endif