#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class RobotCleanerTFBroadcaster : public rclcpp::Node
{
    public:
    RobotCleanerTFBroadcaster()
    : Node("robot_cleaner_tf_broadcaster")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        timer_ = this->create_wall_timer(100ms, std::bind(&RobotCleanerTFBroadcaster::broadcastTransforms, this));
        start_time_ = this->now();
    }

    private:
    void broadcastTransforms()
    {
        rclcpp::Time now = this->now();
        double s = (now - start_time_).seconds();

        geometry_msgs::msg::TransformStamped t;
        tf2::Quaternion q ;

        // motion equation
        double angle = s/5;
        double brush_yaw = std::sin(angle);
        //

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

        tf_broadcaster_->sendTransform(t);        

        // home_base -> body_link
        t.header.stamp = now;t.header.stamp = now;
        t.header.frame_id = "home_base";
        t.child_frame_id = "body_link";
        t.transform.translation.x = 1.0;
        t.transform.translation.y = 1.0;
        t.transform.translation.z = 0.0;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();

        tf_broadcaster_->sendTransform(t);

        // body_link -> Lmop
        t.header.stamp = now;
        t.header.frame_id = "body_link";
        t.child_frame_id = "Lmop";
        t.transform.translation.x = -0.10;
        t.transform.translation.y = -0.18;
        t.transform.translation.z = -0.03;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();

        tf_broadcaster_->sendTransform(t);

        // body_link -> Rmop
        t.header.stamp = now;
        t.header.frame_id = "body_link";
        t.child_frame_id = "Rmop";
        t.transform.translation.x = 0.10;
        t.transform.translation.y = -0.18;
        t.transform.translation.z = -0.03;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();

        tf_broadcaster_->sendTransform(t);

        // body_link -> Lwheel
        t.header.stamp = now;
        t.header.frame_id = "body_link";
        t.child_frame_id = "Lwheel";
        t.transform.translation.x = -0.12;
        t.transform.translation.y = -0.10;
        t.transform.translation.z = 0.0;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();

        tf_broadcaster_->sendTransform(t);

        // body_link -> Rwheel
        t.header.stamp = now;
        t.header.frame_id = "body_link";
        t.child_frame_id = "Rwheel";
        t.transform.translation.x = 0.12;
        t.transform.translation.y = -0.10;
        t.transform.translation.z = 0.0;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();

        tf_broadcaster_->sendTransform(t);

        // body_link -> Fwheel
        t.header.stamp = now;
        t.header.frame_id = "body_link";
        t.child_frame_id = "Fwheel";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.15;
        t.transform.translation.z = -0.02;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();

        tf_broadcaster_->sendTransform(t);

        // body_link -> brush
        t.header.stamp = now;
        t.header.frame_id = "body_link";
        t.child_frame_id = "brush";
        t.transform.translation.x = -0.08;
        t.transform.translation.y = 0.12;
        t.transform.translation.z = -0.03;
        q.setRPY(0, 0, brush_yaw);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();

        tf_broadcaster_->sendTransform(t);

        // body_link -> laser_camera
        t.header.stamp = now;
        t.header.frame_id = "body_link";
        t.child_frame_id = "laser_camera";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.3;
        t.transform.translation.z = 0.08;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();

        tf_broadcaster_->sendTransform(t);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotCleanerTFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}