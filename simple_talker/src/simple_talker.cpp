#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>
#include <functional>

// To simplify timer definition
using namespace std::chrono_literals;

class SimpleTalker : public rclcpp::Node {
public:
    SimpleTalker() : rclcpp::Node("simple_talker"), count_(0) {
       // Create topic and publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        // call function 'timer_callback' every 500ms.
        timer_ = this->create_wall_timer(
           500ms, std::bind(&SimpleTalker::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        // Create message
        auto message = std_msgs::msg::String();
        message.data = "Hello world!"+ std::to_string(count_++);
        // print message to terminal
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); 
        // publish message
        publisher_->publish(message);
    }
    size_t count_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleTalker>());
    rclcpp::shutdown();

    return 0;
}