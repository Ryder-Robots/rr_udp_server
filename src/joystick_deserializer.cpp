#include "rr_udp_server/joystick_deserializer.hpp"

using namespace rr_udp_server;

RrJoystickDeserializer::RrJoystickDeserializer() {
  // reserve memory for the vectors.
  axes_.reserve(AXES_SZ);
  buttons_.reserve(BUTTONS_SZ);

  clear();
}

/*
 * erase all elements and set them to default size. values are all initally
 * set as zero, 
 */
void RrJoystickDeserializer::clear() {
  // erase all elements
  axes_.erase(axes_.begin(), axes_.end());
  buttons_.erase(buttons_.begin(), buttons_.end());

  // resize vectors back to defaults.
  axes_.resize(AXES_SZ);
  buttons_.resize(BUTTONS_SZ);

  // set default to zero
  std::fill(axes_.begin(), axes_.end(), 0);
  std::fill(buttons_.begin(), buttons_.end(), 0);
}

void RrJoystickDeserializer::deserialize(
    const udp_msgs::msg::UdpPacket udp_packet) {}

bool RrJoystickDeserializer::update_state(rclcpp::ClientBase::SharedPtr client) {}

const std::vector<float> RrJoystickDeserializer::get_axes() {
  return axes_;
}

const std::vector<int> RrJoystickDeserializer::get_buttons() {
  return buttons_;
}