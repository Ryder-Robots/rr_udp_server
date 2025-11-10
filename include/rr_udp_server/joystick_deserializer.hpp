#ifndef JOYSTICK_DESERIALIZER_HPP
#define JOYSTICK_DESERIALIZER_HPP

#include <vector>

#include "rr_udp_server/deserializer.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace rr_udp_server {

/**
 * @class JoystickDeserializer
 * @brief deserialize and publish USP packet that has joystick message
 */
class JoystickDeserializer : public RrUdpDeserializer {
 public:
  /**
   * @fn clear
   * @brief reset buttons and axes vectors.
   */
  void clear() override;

  /**
   * @fn deserialize
   * @brief deserialize udp_packet data section and update axes, and buttons
   * vectors.
   */
  void deserialize(const udp_msgs::msg::UdpPacket udp_packet) override;

  /**
   * @fn update_state
   * @brief send request to state manager joy service.
   *
   * convert vectors into sensor_msg::msg::Joy message and submit to state
   * service.
   */
  bool update_state(rclcpp::ClientBase::SharedPtr client) override;

 private:
  /**
   * check that all values submitted by inbound request fit into clamped
   * parameters.
   */
  bool validate();

  std::vector<float> axes_;   // inbound axes definition. 
  std::vector<int> buttons_;  // inbound buttons.
};
} // namespace rr_udp_server

#endif // JOYSTICK_DESERIALIZER_HPP