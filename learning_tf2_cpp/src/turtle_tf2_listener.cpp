#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <turtlesim/srv/spawn.hpp>

#include <chrono>
#include <memory>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node 
{
public:
    FrameListener()
    : Node("turtle_tf2_frame_listener"),
      turtle_spawning_service_ready_(false),
      turtle_spawned_(false)
    {
        // This node publish geometry_msgs::msg::Twist message for turtle2 to chace <target_frame>.
        this->declare_parameter<std::string>("target_frame", "turtle1");
        this->get_parameter("target_frame", target_frame_);
        
        // Create buffer of tf and tf listener.
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create spawner for turtle2
        spawner_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        // Create publisher for Twist msg.
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle2/cmd_vel", 1);

        // Register callback function.
        timer_ = create_wall_timer(
            1s, std::bind(&FrameListener::on_timer, this)
        );
    }

private:
    void on_timer()
    {
        std::string fromFrameRel = target_frame_.c_str();
        std::string toFrameRel = "turtle2";

        if (turtle_spawning_service_ready_) {
            // services of turtlesim are launched. 
            if (turtle_spawned_) {
                // turtle2 already spawned.
                geometry_msgs::msg::TransformStamped transformStamped;

                try {
                    transformStamped = tf_buffer_->lookupTransform(
                        toFrameRel, fromFrameRel,
                        tf2::TimePointZero // get the latest transform
                    );
                }
                catch (tf2::TransformException& ex) {
                    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", 
                    toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
                    return;
                }
                // Create Twist msg.
                geometry_msgs::msg::Twist msg;

                static const double scaleRotationRate = 1.0;
                msg.angular.z = scaleRotationRate*atan2(
                    transformStamped.transform.translation.y,
                    transformStamped.transform.translation.x
                );

                static const double scaleForwardSpeed = 0.5;
                msg.linear.x = scaleForwardSpeed*sqrt(
                    pow(transformStamped.transform.translation.x, 2) + 
                    pow(transformStamped.transform.translation.y, 2)
                );
                // Publish Twist msg.
                publisher_->publish(msg);
            }
            else {
                RCLCPP_INFO(this->get_logger(), "Successfully spawned");
                turtle_spawned_ = true;
            }
        }
        else {
            if (spawner_->service_is_ready()) {
                // services of turtlesim are launched. 

                // Create a message for using service.
                auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
                request->x = 4.0;
                request->y = 2.0;
                request->theta = 0.0;
                request->name = "turtle2";

                // Create callback function
                using ServiceResponseFuture = rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture;
                auto response_received_callback = [this](ServiceResponseFuture future) {
                    auto result = future.get();
                    if (strcmp(result->name.c_str(), "turtle2") == 0.0) {
                        // turtle2 is spawned.
                        turtle_spawning_service_ready_ = true;
                    }
                    else {
                        RCLCPP_ERROR(this->get_logger(), "Service callback result mismatch");
                    }
                };
                // Call service. 
                // first argument is a message sending to the service
                // second argument is callback function which called when the service returns message.
                auto result = spawner_->async_send_request(request, response_received_callback);
            }
            else {
                RCLCPP_INFO(this->get_logger(), "Service is not ready");
            }
        }
    }
    bool turtle_spawning_service_ready_;
    bool turtle_spawned_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameListener>());
    rclcpp::shutdown();

    return 0;
}