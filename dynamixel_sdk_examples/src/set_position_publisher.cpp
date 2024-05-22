#include <memory>
#include <chrono>
#include <functional>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"

using namespace std::chrono_literals;

class SetPositionPublisher : public rclcpp::Node
{
public:
    SetPositionPublisher() : Node("set_position_publisher")
    {
        publisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>(
            "set_position", 10);

        timer_ = this->create_wall_timer(3000ms, std::bind(&SetPositionPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        rclcpp::Time now = this->get_clock()->now();
        double x = now.seconds() * (M_PI/4);
        auto message = std::make_shared<dynamixel_sdk_custom_interfaces::msg::SetPosition>();
        // Generate angles or retrieve them from wherever they are stored
        int angle1 = 20*sin(x);
        int angle2 = 50*cos(x);

        // Fill in the message
        message->angle_1 = angle1;
        message->angle_2 = angle2;

        RCLCPP_INFO(this->get_logger(), "Publishing set position: %d, %d", angle1, angle2);
        publisher_->publish(*message);
    }

    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SetPositionPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
