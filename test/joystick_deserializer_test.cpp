#include "rr_udp_server/joystick_deserializer.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

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
  // clear the buffers fist and check that size is correct.
  deserializer_->clear();

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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}