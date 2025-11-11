#include "rr_udp_server/joystick_deserializer.hpp"

using namespace rr_udp_server;

RrJoystickDeserializer::RrJoystickDeserializer() {}

void RrJoystickDeserializer::clear() {}

void RrJoystickDeserializer::deserialize(
    const udp_msgs::msg::UdpPacket udp_packet) {}

bool RrJoystickDeserializer::update_state(rclcpp::ClientBase::SharedPtr client) {}

const std::vector<float> RrJoystickDeserializer::get_axes() {
  return axes_;
}

const std::vector<int> RrJoystickDeserializer::get_buttons() {
  return buttons_;
}