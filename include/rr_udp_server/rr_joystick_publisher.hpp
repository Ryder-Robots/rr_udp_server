#ifndef RR_JOYSTICK_PUBLISHER_HPP
#define RR_JOYSTICK_PUBLISHER_HPP

#include <memory>
#include "rr_udp_server/joystick_deserializer.hpp"
#include "rr_udp_server/rr_publisher_interface.hpp"
#include "sensor_msgs/msg/joy.h"

namespace rr_udp_server {

class RrJoyStickPublisherNode : public rclcpp::Node {
 public:
  RrJoyStickPublisherNode() : rclcpp::Node("rr_udp_server_joy_node") {}
  void init();
  const std::string topic_ = "/state/joy";
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
};

class RrJoyStickPublisher : public RrPublisherInterface {
 public:
  /**
   * @fn get_node
   * @brief returns associated node.
   */
  virtual std::shared_ptr<rclcpp::Node> get_node() override;

  /**
   * @fn get_deserializer
   * @brief returns the associated deserializer
   */
  std::shared_ptr<RrUdpDeserializer> get_deserializer() override;

  /**
   * @fn init
   * @brief performs any required initlization.
   */
  void init() override;

  /**
   * @fn update_state
   * @brief publishes message to topic
   */
  void update_state() override;

 private:
  std::shared_ptr<RrJoyStickPublisherNode> node_;
  std::shared_ptr<RrJoystickDeserializer> deserializer_;
};
} // namespace rr_udp_server

#endif // RR_JOYSTICK_PUBLISHER_HPP