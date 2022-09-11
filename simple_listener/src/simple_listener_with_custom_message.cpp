#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <my_messages/msg/my_string.hpp>

#include <memory>
#include <string>
#include <functional>

using std::placeholders::_1;

class SimpleListener :  public rclcpp::Node {
public:
    SimpleListener() : rclcpp::Node("simple_listener_with_custom_msg") {
        // Create subscriber which subscribe
        subscriber_ = this->create_subscription<my_messages::msg::MyString>(
            "topic", 10, std::bind(&SimpleListener::topic_callback, this, _1)
        );
    }

private:
    void topic_callback(const my_messages::msg::MyString::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s' as `%d`th message", msg->string.data.c_str(), msg->count);
    }

    rclcpp::Subscription<my_messages::msg::MyString>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleListener>());
  rclcpp::shutdown();
  return 0;
}
