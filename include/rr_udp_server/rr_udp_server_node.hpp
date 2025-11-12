#include "rclcpp/rclcpp.hpp"
#include "udp_msgs/msg/udp_packet.hpp"
#include "rr_udp_server/deserializer_fact.hpp"

namespace rr_udp_server {
class RrUdpServerNode : public rclcpp::Node {
 public:
  RrUdpServerNode() : rclcpp::Node("rr_udp_server_node") { init(); }
  ~RrUdpServerNode() = default;

 private:

  /**
   * @fn init
   * @brief initilize subscriber
   */
  void init();

  /**
   * @fn subscriber_cb
   * @param inbound request
   * @brief callback that is used for inbound requests.
   */
  void subscriber_cb(const udp_msgs::msg::UdpPacket packet);

  // list of clients that are created by factory are returned back to node.
  std::vector<rclcpp::ClientBase::SharedPtr> clients_;

  RrDeserializerFact factory_;

  // following values are kept for heartbeat health messages
  long rx_ = 0;
  long tx_ = 0;
  long err_ = 0;
  
};
} // namespace rr_udp_server