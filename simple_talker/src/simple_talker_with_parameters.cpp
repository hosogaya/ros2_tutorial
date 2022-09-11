#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <my_messages/msg/my_string.hpp>

#include <memory>
#include <string>
#include <functional>

// To simplify timer definition
using namespace std::chrono_literals;

class SimpleTalker : public rclcpp::Node {
public:
    SimpleTalker() : rclcpp::Node("simple_talker_with_parameters") {
        // declare parameters (<paramter_name>, <default_value>)
        declare_parameter("string", "Hello world");
        count_ = declare_parameter("count", 0);
        // Create topic and publisher
        publisher_ = this->create_publisher<my_messages::msg::MyString>("topic", 10);
        // Register callback function
        timer_ = this->create_wall_timer(
           500ms, std::bind(&SimpleTalker::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        // Create message
        auto message = my_messages::msg::MyString();
        // Set value from parameter
        message.string.data = get_parameter("string").as_string();
        message.count = count_++;
        // Print to terminal
        RCLCPP_INFO(this->get_logger(), "Publishing %dth message: '%s'", message.count, message.string.data.c_str());
        // Publish message
        publisher_->publish(message);
    }
    size_t count_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<my_messages::msg::MyString>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleTalker>());
    rclcpp::shutdown();

    return 0;
}