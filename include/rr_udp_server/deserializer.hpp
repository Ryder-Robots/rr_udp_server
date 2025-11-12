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
   * @brief resets internal buffer.
   *
   * This command should be called first or last, after update_state().
   */
  virtual void reset() = 0;

  /**
   * @fn init
   * @param udp_packet inbound packet to deserialize
   * @brief deserialize udp_packet
   * @return returns status code, if code is not 0 then state will be set, but
   * warning message will be generated.
   */
  virtual uint8_t deserialize(const udp_msgs::msg::UdpPacket udp_packet) = 0;

  /**
   * @fn update_state
   * @param client as derived by factory used to send the request
   * @brief send message to state service.
   */
  virtual uint8_t update_state(rclcpp::ClientBase::SharedPtr client,
                               std::shared_ptr<rclcpp::Node> node) = 0;

  /**
   * @fn err_str
   * @brief returns string containing error in the event that deserialize
   * returns false.
   */
  virtual const std::string err_str() = 0;

  static uint8_t OK() { return 0; }
  static uint8_t WARN() { return 1; }
  static uint8_t ERROR() { return 2; }
};
} // namespace rr_udp_server

#endif // DESERIALIZER_HPP