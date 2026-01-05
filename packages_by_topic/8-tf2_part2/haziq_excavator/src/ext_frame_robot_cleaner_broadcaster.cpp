#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class ExtRobotCleanerTFBroadcaster : public rclcpp::Node
{
public:
    ExtRobotCleanerTFBroadcaster()
    : Node("ext_robot_cleaner_tf_broadcaster")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(100ms, std::bind(&ExtRobotCleanerTFBroadcaster::broadcastTransforms, this));
        start_time_ = this->now();
    }

private:
    void broadcastTransforms()
    {
        rclcpp::Time now = this->now();
        double s = (now - start_time_).seconds();

        geometry_msgs::msg::TransformStamped t;
        tf2::Quaternion q;

        // motion & brush rotation
        double angle = s / 5.0;
        double brush_yaw = s * 5.0;
        double Lmop_yaw = s * 17.0;
        double Rmop_yaw = s * -17.0;

        // world -> home_base
        t.header.stamp = now;
        t.header.frame_id = "world";
        t.child_frame_id = "home_base";
        t.transform.translation.x = -2.0;
        t.transform.translation.y = 3.0;
        t.transform.translation.z = 0.0;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);

        // home_base -> body_link
        t.header.frame_id = "home_base";
        t.child_frame_id = "body_link";
        t.transform.translation.x = 1.0;
        t.transform.translation.y = 1.0;
        t.transform.translation.z = 0.0;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);

        // body_link -> Lmop (left mop)
        t.header.frame_id = "body_link";
        t.child_frame_id = "Lmop";
        t.transform.translation.x = -0.25;
        t.transform.translation.y = -0.35;
        t.transform.translation.z = -0.05;
        q.setRPY(0, 0, Lmop_yaw);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);

        // body_link -> Rmop (right mop)
        t.header.frame_id = "body_link";
        t.child_frame_id = "Rmop";
        t.transform.translation.x = 0.25;
        t.transform.translation.y = -0.35;
        t.transform.translation.z = -0.05;
        q.setRPY(0, 0, Rmop_yaw);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);

        // body_link -> Lwheel
        t.header.frame_id = "body_link";
        t.child_frame_id = "Lwheel";
        t.transform.translation.x = -0.35;
        t.transform.translation.y = -0.20;
        t.transform.translation.z = 0.0;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);

        // body_link -> Rwheel
        t.header.frame_id = "body_link";
        t.child_frame_id = "Rwheel";
        t.transform.translation.x = 0.35;
        t.transform.translation.y = -0.20;
        t.transform.translation.z = 0.0;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);

        // body_link -> Fwheel (front roller)
        t.header.frame_id = "body_link";
        t.child_frame_id = "Fwheel";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.35;
        t.transform.translation.z = -0.03;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);

        // body_link -> brush (rotating brush underneath front-left)
        t.header.frame_id = "body_link";
        t.child_frame_id = "brush";
        t.transform.translation.x = -0.15;
        t.transform.translation.y = 0.25;
        t.transform.translation.z = -0.05;
        q.setRPY(0, 0, brush_yaw);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);

        // body_link -> laser_camera (front top)
        t.header.frame_id = "body_link";
        t.child_frame_id = "laser_camera";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.45;
        t.transform.translation.z = 0.15;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExtRobotCleanerTFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
