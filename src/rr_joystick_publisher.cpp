#include "rr_udp_server/rr_joystick_publisher.hpp"

using namespace rr_udp_server;

/**
 * standard getter, if locking is needed add it here.
 */
std::shared_ptr<rclcpp::Node> RrJoyStickPublisher::get_node() {
    return node_;
}


/**
 * returns associated deserializer
 */
std::shared_ptr<RrUdpDeserializer> RrJoyStickPublisher::get_deserializer() {
    return deserializer_;
}

/**
 * creates a node with middleware, and creates the deserializer.
 * 
 * Note that the node will need to be added to the executor by a maoin class.
 */
void RrJoyStickPublisher::init() {
    node_ = std::make_shared<RrJoyStickPublisherNode>();
    node_->publisher_ = node_->create_publisher<sensor_msgs::msg::Joy>(node_->topic_, 10);

    // create deserializer
    deserializer_ = std::make_shared<RrJoystickDeserializer>();
}

/*
 * should be called after deserialization only, and state of deserializer must be checked.
 */
void RrJoyStickPublisher::update_state() {
    sensor_msgs::msg::Joy msg;
    msg.axes = deserializer_->get_axes();
    msg.buttons = deserializer_->get_buttons();
    msg.header.stamp = node_->now();
    msg.header.frame_id = rr_constants::LINK_JOY_PS4;

    node_->publisher_->publish(msg);
}