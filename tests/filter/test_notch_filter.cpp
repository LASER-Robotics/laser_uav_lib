#include <gtest/gtest.h>
#include <laser_uav_lib/filter/notch_filter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <random>

using namespace laser_uav_lib;

class NotchFilterTest : public ::testing::Test
{
protected:
    rclcpp::Logger logger = rclcpp::get_logger("NotchFilterTest");

    // Setup: Inicializa o ROS2, se necessário
    void SetUp() override
    {
        if (!rclcpp::ok())
            rclcpp::init(0, nullptr);
    }

    // Cleanup: Finaliza o ROS2, se necessário
    void TearDown() override
    {
        if (!rclcpp::ok())
            rclcpp::shutdown();
    }
};
