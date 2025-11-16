#ifndef RR_PUBLSHER_INTERFACE_HPP
#define RR_PUBLSHER_INTERFACE_HPP

#include <memory>
#include "udp_msgs/msg/udp_packet.hpp"
#include "rclcpp/node.hpp"
#include "rr_udp_server/deserializer.hpp"
#include "rr_common_base/rr_sensor_constants.hpp"
// #include "rclcpp/node_interfaces/get_node_base_interface.hpp"

namespace rr_udp_server {
    
    /**
     * @class RrPublisherInterface
     * @brief provides interface for publsher nodes to write state servixcve
     */
    class RrPublisherInterface {
        public:
            /**
             * @fn get_node
             * @brief retrive the attached publsiher node.
             */
            virtual std::shared_ptr<rclcpp::Node> get_node() = 0;

            /**
             * @fn init perform any initlization.
             */
            virtual void init() = 0;

            /**
             * @fn get_deserializer
             * @brief retrieve the des
             */
            virtual std::shared_ptr<RrUdpDeserializer> get_deserializer() = 0;

            virtual void update_state() = 0;
    };
}

#endif // RR_PUBLSHER_INTERFACE_HPP