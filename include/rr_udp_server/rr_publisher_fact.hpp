#ifndef RR_PUBLISHER_FACT_HPP
#define RR_PUBLISHER_FACT_HPP

#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rr_udp_server/rr_joystick_publisher.hpp"
#include "rr_udp_server/rr_publisher_interface.hpp"
#include "udp_msgs/msg/udp_packet.hpp"

namespace rr_udp_server {
/**
 * @class RrPublisherFact
 * @brief factory service for publsiher nodes
 *
 * Mangages and creates nodes taht respond to inbound commands.
 */
class RrPublisherFact {
 public:
  /**
   * @fn get_interface
   * @brief retrieves interface that matches the inbound packet.
   */
  std::shared_ptr<RrPublisherInterface> get_publisher(
      const udp_msgs::msg::UdpPacket udp_packet, bool& available);

  /**
   * @fn get_publishers
   * @brief returns all the publishers.
   *
   * This routine is used for initlize, and to register the publisher nodes with
   * ROS2 middleware.
   */
  const std::vector<std::shared_ptr<RrPublisherInterface>> get_publishers();

 private:
  std::vector<std::shared_ptr<RrPublisherInterface>> publishers_ = {

      // key = 0
      std::static_pointer_cast<RrPublisherInterface>(
          std::make_shared<RrJoyStickPublisher>())};
};

} // namespace rr_udp_server

#endif // RR_PUBLISHER_FACT_HPP