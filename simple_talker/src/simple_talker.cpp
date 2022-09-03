#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>
#include <functional>

using namespace std::chrono_literals;

class SimpleTalker : public rclcpp::Node {
public:
    SimpleTalker(std::string name) : rclcpp::Node(name), count_(0) {
       // topicという名前のトピックを作成
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        // 500 msごとにtime_callback()を呼び出す
        timer_ = this->create_wall_timer(
           500ms, std::bind(&SimpleTalker::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello world!"+ std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str()); // ターミナルにプリントする
        publisher_->publish(message); // メッセージを送信
    }
    size_t count_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleTalker>("simple_talker"));
    rclcpp::shutdown();

    return 0;
}