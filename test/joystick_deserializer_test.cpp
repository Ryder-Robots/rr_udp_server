#include "rr_udp_server/joystick_deserializer.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "rr_udp_server/generated/inbound.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "udp_msgs/msg/udp_packet.hpp"

using namespace rr_udp_server;

class TestController : public testing::Test {
 protected:
  TestController() {}

  ~TestController() override {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  void SetUp() override { rclcpp::init(0, nullptr); }

  void TearDown() override { rclcpp::shutdown(); }
  std::shared_ptr<RrUdpDeserializer> deserializer_ =
      std::make_shared<RrJoystickDeserializer>();
};

TEST_F(TestController, clear) {
  // reset the buffers fist and check that size is correct.
  deserializer_->reset();

  std::shared_ptr<RrJoystickDeserializer> jd =
      std::static_pointer_cast<RrJoystickDeserializer>(deserializer_);
  EXPECT_EQ(jd->get_axes().size(), jd->AXES_SZ);
  EXPECT_EQ(jd->get_buttons().size(), jd->BUTTONS_SZ);
  for (float value : jd->get_axes()) {
    EXPECT_EQ(0, value);
  }

  for (int value : jd->get_buttons()) {
    EXPECT_EQ(0, value);
  }
}

TEST_F(TestController, deserialize1) {
  InboundMessage packet;
  packet.clear_data();
  Joystick* joystick_data = packet.mutable_joystick();

  joystick_data->add_axes(0.5f);
  joystick_data->add_axes(-0.7f);

  // Initialize joystick buttons (example values)
  joystick_data->add_buttons(1);
  joystick_data->add_buttons(0);

  // Add UDP packet
  // attach inbound message to packet
  udp_msgs::msg::UdpPacket udp_packet;
  rclcpp::Clock clock;
  auto current_time = clock.now();
  udp_packet.header.frame_id = "joy_ps4";
  udp_packet.header.stamp = clock.now();
  udp_packet.address = "127.0.0.1";
  udp_packet.src_port = 57410;

  std::vector<uint8_t> buffer(packet.ByteSizeLong());
  packet.SerializeToArray(buffer.data(),
                          static_cast<int>(packet.ByteSizeLong()));
  udp_packet.data = buffer;
  EXPECT_EQ(deserializer_->deserialize(udp_packet), RrUdpDeserializer::OK());

  std::shared_ptr<RrJoystickDeserializer> jd =
      std::static_pointer_cast<RrJoystickDeserializer>(deserializer_);

  EXPECT_EQ(jd->get_axes()[0], 0.5f);
  EXPECT_EQ(jd->get_axes()[1], -0.7f);
  EXPECT_EQ(jd->get_buttons()[0], 1);
  EXPECT_EQ(jd->get_buttons()[1], 0);
}

TEST_F(TestController, deserialize2) {
  InboundMessage packet;
  packet.clear_data();
  Joystick* joystick_data = packet.mutable_joystick();

  joystick_data->add_axes(1.5f);
  joystick_data->add_axes(-1.7f);

  // Initialize joystick buttons (example values)
  joystick_data->add_buttons(1);
  joystick_data->add_buttons(-1);

  // Add UDP packet
  // attach inbound message to packet
  udp_msgs::msg::UdpPacket udp_packet;
  rclcpp::Clock clock;
  auto current_time = clock.now();
  udp_packet.header.frame_id = "joy_ps4";
  udp_packet.header.stamp = clock.now();
  udp_packet.address = "127.0.0.1";
  udp_packet.src_port = 57410;

  std::vector<uint8_t> buffer(packet.ByteSizeLong());
  packet.SerializeToArray(buffer.data(),
                          static_cast<int>(packet.ByteSizeLong()));
  udp_packet.data = buffer;
  EXPECT_EQ(deserializer_->deserialize(udp_packet), RrUdpDeserializer::ERROR());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}