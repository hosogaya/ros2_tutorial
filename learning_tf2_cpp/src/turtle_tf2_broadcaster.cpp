#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/msg/pose.hpp>

#include <memory>
#include <string>

using std::placeholders::_1;

class FramePublisher : public rclcpp::Node 
{
public:
    FramePublisher() : Node("turtle_tf2_frame_publisher")
    {
        // This node subscribe tf of <turtlename>.
        this->declare_parameter<std::string>("turtlename", "turtle");
        this->get_parameter("turtlename", turtlename_); // substitute parameter to variable.

        // Create tf publisher
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // buffer of output string
        std::ostringstream stream;
        stream << "/" << turtlename_.c_str() << "/pose";
        std::string topic_name = stream.str();

        // Create subscriber for turtle's pose
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            topic_name, 10, 
            std::bind(&FramePublisher::handle_turtle_pose, this, _1)
        );
    }

private:
    // Transform turtlesim::msg::Pose to geometry_msgs::msg::TransformStamped message.
    void handle_turtle_pose(const turtlesim::msg::Pose& msg)
    {
        rclcpp::Time now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = now; // time
        t.header.frame_id = "world"; // parent frame
        t.child_frame_id = turtlename_.c_str(); // child frame

        // translation from parent to child frame in parent frame
        t.transform.translation.x = msg.x;
        t.transform.translation.y = msg.y;
        t.transform.translation.z = 0.0;

        // Create quaternion from roll, pitch, yaw angle.
        // This express posture of child frame in parent frame
        tf2::Quaternion q;
        q.setRPY(0, 0, msg.theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // publish tf message
        tf_broadcaster_->sendTransform(t); 
    }
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string turtlename_;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}