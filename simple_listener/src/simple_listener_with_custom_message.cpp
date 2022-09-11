#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <my_messages/msg/my_string.hpp>

#include <memory>
#include <string>
#include <functional>

using std::placeholders::_1;

class SimpleListener :  public rclcpp::Node {
public:
    SimpleListener(std::string name) : rclcpp::Node(name) {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&SimpleListener::topic_callback, this, _1)
        );
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleListener>("simple_listener"));
  rclcpp::shutdown();
  return 0;
}
