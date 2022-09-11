
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <chrono>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

class StatePublisher : public rclcpp::Node
{
public:
    StatePublisher() : Node("state_publisher") 
    {
        // Create tf publisher 
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // Create joint state publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", 10);
        // Register callback function
        timer_ = this->create_wall_timer(
            100ms, std::bind(&StatePublisher::on_timer, this)
        );
    }   

private:
    void on_timer() {
        rclcpp::Time now = this->get_clock()->now();

        // Create joint state reference 
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = now;
        joint_state.name.resize(3);
        joint_state.name[0] = "swivel";
        joint_state.name[1] = "tilt";
        joint_state.name[2] = "periscope";
        joint_state.position.resize(3);
        joint_state.position[0] = swivel_;
        joint_state.position[1] = tilt_;
        joint_state.position[2] = height_;

        // Create tf (robot position on the world)
        geometry_msgs::msg::TransformStamped t;
        t.header.frame_id = "odom"; 
        t.child_frame_id = "axis"; 
        t.header.stamp = now;

        t.transform.translation.x = cos(angle_)*2.0;
        t.transform.translation.y = sin(angle_)*2.0;
        t.transform.translation.z = 0.7;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, angle_ + M_PI*0.5);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // publish joint state and tf
        publisher_->publish(joint_state);
        tf_broadcaster_->sendTransform(t);

        // update joint state reference
        tilt_ += tinc_;
        if (tilt_ < -0.5f || tilt_ > 0.0f) {
            tinc_ *= -1.0f;
        }
        height_ += hinc_;
        if (height_ > 0.2f || height_ < 0.0f) {
            hinc_ *= -1.0f;
        }
        swivel_ += degree_;
        angle_ += degree_/4.0f;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_{nullptr};
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    const float degree_ = M_PI/180.0f;
    float angle_ = 0.0f;
    float tilt_ = 0.0f;
    float tinc_ = degree_;
    float swivel_ = 0.0f;
    float height_ = 0.0f;
    float hinc_ = 0.005f;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatePublisher>());
    rclcpp::shutdown();
    return 0;
}