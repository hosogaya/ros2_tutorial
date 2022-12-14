#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>

using std::placeholders::_1;

class StaticFramdePublisher : public rclcpp::Node
{
public:
    explicit StaticFramdePublisher(char* transformation[]) : Node("static_turtle_tf2_broadcaster")
    {
        // Create static broadcaster of tf
        // This publish static transform such that robot COG to camera equiped on the robot.
        // So, this node publish tf only once.
        tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        this->make_transforms(transformation);
    }

private:
    void make_transforms(char* transformation[])
    {
        rclcpp::Time now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = now;
        t.header.frame_id = "world"; // parent frame
        t.child_frame_id = transformation[1]; // child frame
        
        // translation from parent to child frame in parent frame
        t.transform.translation.x = atof(transformation[2]);
        t.transform.translation.y = atof(transformation[3]);
        t.transform.translation.z = atof(transformation[4]);

        // Create quaternion from roll, pitch, yaw angle.
        // This express posture of child frame in parent frame
        tf2::Quaternion q;
        q.setRPY( 
            atof(transformation[5]),
            atof(transformation[6]),
            atof(transformation[7])
        );
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // publish tf
        tf_publisher_->sendTransform(t);
    }
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
};

int main(int argc, char* argv[]) {
    auto logger = rclcpp::get_logger("logger");

    // Obtain parameters from comand line arguments
    if (argc != 8) {
        RCLCPP_INFO(
            logger, "Invalid number of parameters\nusage: %d "
            "ros run learning_tf2_cpp static_turtle_tf2_broadcaster "
            "child_frame_name x y z roll pitch yaw"
            , argc
        );
        for (int i = 0; i < argc; ++i) {
            RCLCPP_INFO(logger, "%s", argv[i]);
        }
        return 1;
    }

    if (strcmp(argv[1], "world") == 0) {
        RCLCPP_INFO(logger, "Your static turtle name cannot be 'world'");
        return 1;
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticFramdePublisher>(argv));
    rclcpp::shutdown();
    
    return 0;
}