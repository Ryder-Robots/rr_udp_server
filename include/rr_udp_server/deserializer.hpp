#ifndef DESERIALIZER_HPP
#define DESERIALIZER_HPP

#include "rclcpp/rclcpp.hpp"
#include "udp_msgs/msg/udp_packet.hpp"

namespace rr_udp_server {

/**
 * @class RrUdpDeserializer
 * @brief interface for command deserializers
 */
class RrUdpDeserializer {
 public:
  /**
   * @fn reset
   * @brief clears internal buffer.
   *
   * This command should be called first or last, after update_state().
   */
  virtual void clear() = 0;

  /**
   * @fn init
   * @param udp_packet inbound packet to deserialize
   * @brief deserialize udp_packet
   */
  virtual void deserialize(const udp_msgs::msg::UdpPacket udp_packet) = 0;

  /**
   * @fn update_state
   * @param client as derived by factory used to send the request
   * @brief send message to state service.
   * @return true if state updated, otherwise return false.
   */
  virtual bool update_state(rclcpp::ClientBase::SharedPtr client) = 0;
};
} // namespace rr_udp_server

#endif // DESERIALIZER_HPP