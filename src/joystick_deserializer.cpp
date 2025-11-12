#include "rr_udp_server/joystick_deserializer.hpp"

using namespace rr_udp_server;

RrJoystickDeserializer::RrJoystickDeserializer() {
  // reserve memory for the vectors.
  axes_.reserve(AXES_SZ);
  buttons_.reserve(BUTTONS_SZ);

  reset();
}

/*
 * erase all elements and set them to default size. values are all initally
 * set as zero,
 */
void RrJoystickDeserializer::reset() {
  // erase all elements
  axes_.erase(axes_.begin(), axes_.end());
  buttons_.erase(buttons_.begin(), buttons_.end());
  err_ = "";

  // resize vectors back to defaults.
  axes_.resize(AXES_SZ);
  buttons_.resize(BUTTONS_SZ);

  // set default to zero
  std::fill(axes_.begin(), axes_.end(), 0);
  std::fill(buttons_.begin(), buttons_.end(), 0);
}

/**
 * @fn err_str
 * @brief returns last error code, this will be reset once reset is called.
 */
const std::string RrJoystickDeserializer::err_str() {
  return err_;
}

/**
 * @fn deserialize
 * @brief deserialize the packet and load the buffers.
 *
 * Verify that axes, and buttons do not exceed speicifications that are
 * documented in
 * https://github.com/ros-drivers/joystick_drivers/blob/ros2/joy/README.md
 *
 *
 * @param udp_packet inbound packet.
 */
uint8_t RrJoystickDeserializer::deserialize(
    const udp_msgs::msg::UdpPacket udp_packet) {
  uint8_t status = FT_RSLT_ERR_;
  InboundMessage packet;
  packet.ParseFromArray(udp_packet.data.data(), udp_packet.data.size());
  if (!packet.has_joystick()) {
    err_ = "inbound packet does not contain joystick packet";
    return ERROR();
  }

  Joystick* joystick_data = packet.mutable_joystick();
  if (joystick_data->axes_size() > AXES_SZ) {
    err_ =
        "axes size is greater than max AXES_SZ, ignoring values excess values";
    status = WARN();
  }

  if (joystick_data->buttons_size() > BUTTONS_SZ) {
    err_ =
        "buttons size is greater than max AXES_SZ, ignoring values excess "
        "values";
    status = WARN();
  }

  // validate max and min values
  for (float i : joystick_data->axes()) {
    if (i > MAX_AXES || i < MIN_AXES) {
      err_ = "axes is not within threshold range";
      return ERROR();
    }
  }

  // validate max and min values
  for (int i : joystick_data->buttons()) {
    if (i > MAX_BUTTON || i < MIN_BUTTON) {
      err_ = "button is not within threshold range";
      return ERROR();
    }
  }

  // for remaining data populate the buffers, keep the data in the same order,
  // or we could see the robot do some strange and surprising things ;)
  int n = (joystick_data->axes_size() > AXES_SZ) ? AXES_SZ
                                                 : joystick_data->axes_size();
  for (int i = 0; i < n; i++) {
    axes_[i] = joystick_data->axes()[i];
  }

  n = (joystick_data->buttons_size() > BUTTONS_SZ)
          ? BUTTONS_SZ
          : joystick_data->buttons_size();
  for (int i = 0; i < n; i++) {
    buttons_[i] = joystick_data->buttons()[i];
  }

  return status;
}

// TODO: this looks like a candidate for a base class.
uint8_t RrJoystickDeserializer::update_state(
    rclcpp::ClientBase::SharedPtr client_ptr, std::shared_ptr<rclcpp::Node> node) {
  // retrieve the client
  rclcpp::Client<rr_interfaces::srv::Joy>::SharedPtr client =
      std::static_pointer_cast<rclcpp::Client<rr_interfaces::srv::Joy>>(
          client_ptr);

  auto request = std::make_shared<rr_interfaces::srv::Joy::Request>();
  auto ticks = std::chrono::duration<int, std::milli>(100);

  while (!client->wait_for_service(ticks)) {
    if (!rclcpp::ok()) {
      err_ = "interrupted while waiting for the service.";
      return ERROR();
    }
  }

  // store the future, if a timeout is thrown, then we can pick it up
  // in the errors servers
  auto result = client->async_send_request(request);
  auto f_code = rclcpp::FutureReturnCode::SUCCESS;
  if ((f_code = rclcpp::spin_until_future_complete(node, result)) != rclcpp::FutureReturnCode::SUCCESS) {
    switch (f_code) {
      case rclcpp::FutureReturnCode::INTERRUPTED:
        err_ = "state update was interrupted";
        break;
      default: // rclcpp::FutureReturnCode::TIMEOUT
        err_ = "attempt to update timed out";
    }
    return ERROR();
  }

  return OK();
}

const std::vector<float> RrJoystickDeserializer::get_axes() {
  return axes_;
}

const std::vector<int> RrJoystickDeserializer::get_buttons() {
  return buttons_;
}