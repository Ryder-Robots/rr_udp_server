#ifndef RR_UDP_SERVER__RR_UDP_SERVER_LIB_HPP_
#define RR_UDP_SERVER__RR_UDP_SERVER_LIB_HPP_

#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "rr_udp_server/visibility_control.h"
#include "udp_msgs/msg/udp_packet.hpp"

namespace rr_udp_server
{

#define RR_UDP_SVR_MSG_CLASS udp_msgs::msg::UdpPacket
#define RR_UDP_SVR_READ_TOPIC "/udp_read"

class RrUdpServerLib : public rclcpp::Node
{
 public:
  RrUdpServerLib() : Node("rr_udp_server_lib") { init(); }

  /**
   * @fn subscribe
   *
   * listens for
   */
  void subscriber(const RR_UDP_SVR_MSG_CLASS::SharedPtr packet);

  void publisher();

  virtual ~RrUdpServerLib();

 private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<RR_UDP_SVR_MSG_CLASS>::SharedPtr publisher_;
  rclcpp::Subscription<RR_UDP_SVR_MSG_CLASS>::SharedPtr subscription_;
  rclcpp::Logger logger_ = rclcpp::get_logger("rr_udp_server_lib");

  // private methods
  void init();
};

}  // namespace rr_udp_server

#endif  // RR_UDP_SERVER__RR_UDP_SERVER_LIB_HPP_
