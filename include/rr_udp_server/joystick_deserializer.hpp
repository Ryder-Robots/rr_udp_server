#ifndef JOYSTICK_DESERIALIZER_HPP
#define JOYSTICK_DESERIALIZER_HPP

#include <vector>

#include "rr_udp_server/deserializer.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace rr_udp_server {

/**
 * @class JoystickDeserializer
 * @brief deserialize and publish USP packet that has joystick message
 *
 * reference:
 * https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md
 */
class JoystickDeserializer : public RrUdpDeserializer {
 public:
  JoystickDeserializer();

  ~JoystickDeserializer() = default;

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

  // constants
  const size_t BUTTONS_SZ = 20;
  const size_t AXES_SZ = 5;
  const int MAX_BUTTON = 1;
  const int MIN_BUTTON = 0;
  const float MAX_AXES = 1;
  const float MIN_AXES = -1;

 private:
  /**
   * check that all values submitted by inbound request fit into clamped
   * parameters.
   */
  bool validate();

  std::vector<float> axes_;  // inbound axes definition.
  std::vector<int> buttons_; // inbound buttons.
};
} // namespace rr_udp_server

#endif // JOYSTICK_DESERIALIZER_HPP