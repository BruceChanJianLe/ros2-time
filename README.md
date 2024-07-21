# ROS2 Time

This repository aims to demonstrate the usage of time in ROS2.

## std_msgs::msg::Header
link: https://docs.ros2.org/latest/api/std_msgs/msg/Header.html
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

class TimePublisher : public rclcpp::Node
{
public:
    TimePublisher() : Node("time_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Header>("time_topic", 10);

        // Create a timer to publish the current time periodically (every second in this example)
        timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
            auto header_msg = std_msgs::msg::Header();
            header_msg.stamp = this->get_clock()->now(); // Get the current time

            // Publish the header message
            publisher_->publish(header_msg);
        });
    }

private:
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TimePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

##  builtin_interfaces::msg::Time
link: https://docs.ros2.org/latest/api/builtin_interfaces/msg/Time.html
```cpp
#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/time.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create a node to access time utilities
    auto node = std::make_shared<rclcpp::Node>("time_publisher");

    // Get the current time
    auto now = node->now();

    // Convert the time to seconds and nanoseconds
    auto seconds = now.seconds();
    auto nanoseconds = now.nanoseconds();

    // Create a builtin_interfaces/Time message and set the current time
    builtin_interfaces::msg::Time current_time;
    current_time.sec = seconds;
    current_time.nanosec = nanoseconds;

    // Print the current time
    RCLCPP_INFO(node->get_logger(), "Current timestamp: %ld.%09u", current_time.sec, current_time.nanosec);

    rclcpp::shutdown();
    return 0;
}
```
