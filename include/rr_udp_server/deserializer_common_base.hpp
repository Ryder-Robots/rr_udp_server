#ifndef DESERIALIZER_COMMON_BASE_HPP
#define DESERIALIZER_COMMON_BASE_HPP

#include "rr_udp_server/deserializer.hpp"

namespace rr_udp_server {

    /**
     * @class RrDeserializerCommonBase
     * @brief Provides common base functions to deserializers.
     */
    class RrDeserializerCommonBase : public RrUdpDeserializer {
        protected:
       // TODO implement T uint8_t RrJoystickDeserializer::update_state(rclcpp::ClientBase::SharedPtr client_ptr, std::shared_ptr<rclcpp::Node> node)
       //
       // also errors, and success can go here.
    };
}

#endif