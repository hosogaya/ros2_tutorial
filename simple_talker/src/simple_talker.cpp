#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <my_messages/msg/my_string.hpp>

#include <memory>
#include <string>
#include <functional>

using namespace std::chrono_literals;

class SimpleTalker : public rclcpp::Node {
public:
    SimpleTalker(std::string name) : rclcpp::Node(name) {
        declare_parameter("string", "Hello world");
        count_ = declare_parameter("count", 0);
        publisher_ = this->create_publisher<my_messages::msg::MyString>("topic", 10);
        // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
           500ms, std::bind(&SimpleTalker::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        auto message = my_messages::msg::MyString();
        //  message.data = "Hello world!"+ std::to_string(count_++);
        // message.string.data = "Hello world!";
        message.string.data = get_parameter("string").as_string();
        // message.string.data = str_;
        message.count = count_++;
        RCLCPP_INFO(this->get_logger(), "Publishing %dth message: '%s'", message.count, message.string.data.c_str());
        publisher_->publish(message);
    }
    size_t count_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<my_messages::msg::MyString>::SharedPtr publisher_;
    // std::string str_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleTalker>("simple_talker"));
    rclcpp::shutdown();

    return 0;
}