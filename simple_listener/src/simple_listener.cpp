#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>
#include <functional>

// Using for bind callback function.
using std::placeholders::_1;

class SimpleListener :  public rclcpp::Node {
public:
    SimpleListener() : rclcpp::Node("simple_listener") {
        // Create subscriber which subscribe messages in "topic". 
        // When `subsciber_` subscribe message, a function `topic_callback` is called. 
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "topic", // topic name
            10, // Quality of Service. The larger value is, the higher communication quality is among nodes, topics and services in exchange for communication speed.
            std::bind(&SimpleListener::topic_callback, this, _1) // Callback function
        );
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        // Print message to terminal.
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleListener>());
  rclcpp::shutdown();
  return 0;
}
