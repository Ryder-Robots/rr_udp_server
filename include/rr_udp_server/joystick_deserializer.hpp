#ifndef JOYSTICK_DESERIALIZER_HPP
#define JOYSTICK_DESERIALIZER_HPP

#include <vector>

#include "rr_udp_server/generated/inbound.pb.h"
#include "rr_udp_server/deserializer.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rr_interfaces/srv/joy.hpp"

namespace rr_udp_server {

/**
 * @class JoystickDeserializer
 * @brief deserialize and publish USP packet that has joystick message
 *
 * reference:
 * https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md
 */
class RrJoystickDeserializer : public RrUdpDeserializer {
 public:
  RrJoystickDeserializer();

  ~RrJoystickDeserializer() = default;

  /**
   * @fn reset
   * @brief reset buttons and axes vectors.
   */
  void reset() override;

  /**
   * @fn deserialize
   * @brief deserialize udp_packet data section and update axes, and buttons
   * vectors.
   */
  u_int8_t deserialize(const udp_msgs::msg::UdpPacket udp_packet) override;

  /**
   * @fn err_str
   * @brief this message is set during input validation of the object in the event that something has gone wrong.
   * 
   * If deserialize returns false, controller will call this method to update log with perinent information.
   */
  const std::string err_str() override;

  /**
   * @fn update_state
   * @brief send request to state manager joy service.
   *
   * convert vectors into sensor_msg::msg::Joy message and submit to state
   * service.
   */
  uint8_t update_state(rclcpp::ClientBase::SharedPtr client, std::shared_ptr<rclcpp::Node> node) override;

  // constants
  const int BUTTONS_SZ = 20;
  const int AXES_SZ = 5;
  const int MAX_BUTTON = 1;
  const int MIN_BUTTON = 0;
  const float MAX_AXES = 1;
  const float MIN_AXES = -1;

  /**
   * @fn get_axes
   * @brief returns axes vector, so that it can be checked.
   *
   * NOTE this is ony expected for debugging and testing purposes, it shoulds
   * not be called directly by the controller.
   */
  const std::vector<float> get_axes();

  /**
   * @fn get_buttons
   * @brief return the button vector.
   *
   * NOTE this is only expected for testing purposes, it should not be called
   * directly by the controller.
   */
  const std::vector<int> get_buttons();

  // This should be moved to a common base, but it will need to be templated.
  // std::vector<rclcpp::Client<rr_interfaces::srv::Joy>::FutureAndRequestId> futures_;

 private:
  std::vector<float> axes_;  // inbound axes definition.
  std::vector<int> buttons_; // inbound buttons.
  std::string err_ = "";

  uint8_t FT_RSLT_ERR_ = RrUdpDeserializer::OK();
};
} // namespace rr_udp_server

#endif // JOYSTICK_DESERIALIZER_HPP