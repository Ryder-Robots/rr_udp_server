#include "rclcpp/rclcpp.hpp"
#include "rr_udp_server/rr_publisher_fact.hpp"
#include "udp_msgs/msg/udp_packet.hpp"

namespace rr_udp_server {
class RrUdpServerNode : public rclcpp::Node {
 public:
  RrUdpServerNode() : rclcpp::Node("rr_udp_server_node") {}
  ~RrUdpServerNode() = default;
  /**
   * @fn init
   * @brief initilize subscriber
   */
  void init(std::shared_ptr<RrPublisherFact>  factory);

 private:
  /**
   * @fn subscriber_cb
   * @param inbound request
   * @brief callback that is used for inbound requests.
   */
  void subscriber_cb(const udp_msgs::msg::UdpPacket packet);
  std::shared_ptr<RrPublisherFact> factory_;

  // Create a subscription to UDP bridge
  rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr subscription_;

  // TODO: Add message topic for health checks.

  // following values are kept for heartbeat health messages
  long rx_ = 0;
  long tx_ = 0;
  long err_ = 0;

  const std::string TOPIC_SUBSCRIBE = "/udp_read";
  const std::string TOPIC_MSG = "/udp_server_msg";
};
} // namespace rr_udp_server