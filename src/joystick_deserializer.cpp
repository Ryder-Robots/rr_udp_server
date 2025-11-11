#include "rr_udp_server/joystick_deserializer.hpp"

using namespace rr_udp_server;

JoystickDeserializer::JoystickDeserializer() {}

void JoystickDeserializer::clear() {}

void JoystickDeserializer::deserialize(
    const udp_msgs::msg::UdpPacket udp_packet) {}

bool JoystickDeserializer::update_state(rclcpp::ClientBase::SharedPtr client) {}