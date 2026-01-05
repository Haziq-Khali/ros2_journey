#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class ExcavatorTFBroadcaster : public rclcpp::Node
{
public:
  ExcavatorTFBroadcaster()
  : Node("excavator_tf_broadcaster")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(100ms, std::bind(&ExcavatorTFBroadcaster::broadcastTransforms, this));
    start_time_ = this->now();
  }

private:
  void broadcastTransforms()
  {
    rclcpp::Time now = this->now();
    double t = (now - start_time_).seconds();

    // Simulated motion values
    double turret_yaw   = 0.5 * std::sin(t / 2.0);
    double arm_pitch    = 0.4 * std::sin(t);
    double elbow_pitch  = 0.6 * std::sin(t * 1.5);
    double hand_pitch   = 0.5 * std::sin(t * 2.0);
    double wheel_angle  = t;  // track wheel rotation

    // Base cicrcling the world
    double R = 5.0;
    double omega = 0.2;
    double x = R * std::cos(omega * t);
    double y = R * std::sin(omega * t);
    double base_yaw = omega * t + M_PI/2.0;
    // 

    geometry_msgs::msg::TransformStamped transform;
    tf2::Quaternion q;
    // ---------------------------
    // world -> base_link
    // ---------------------------
    transform.header.stamp = now;
    transform.header.frame_id = "world";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = 4.0;
    transform.transform.translation.y = 1.0;
    transform.transform.translation.z = 0.0;
    q.setRPY(0, 0, 0);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transform);

    // ---------------------------
    // base_link -> left_track_link
    // ---------------------------
    transform.header.stamp = now;
    transform.header.frame_id = "base_link";
    transform.child_frame_id = "left_track_link";
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.6;
    transform.transform.translation.z = 0.0;
    q.setRPY(0, 0, 0);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transform);

    // ---------------------------
    // base_link -> right_track_link
    // ---------------------------
    transform.header.stamp = now;
    transform.header.frame_id = "base_link";
    transform.child_frame_id = "right_track_link";
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = -0.6;
    transform.transform.translation.z = 0.0;
    q.setRPY(0, 0, 0);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transform);

    // ---------------------------
    // base_link -> turret_link (yaw)
    // ---------------------------
    transform.header.stamp = now;
    transform.header.frame_id = "base_link";
    transform.child_frame_id = "turret_link";
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.5;  // turret height
    q.setRPY(0, 0, turret_yaw);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transform);

    // ---------------------------
    // turret_link -> arm_link (pitch)
    // ---------------------------
    transform.header.stamp = now;
    transform.header.frame_id = "turret_link";
    transform.child_frame_id = "arm_link";
    transform.transform.translation.x = 0.5;  // pivot offset
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.3;
    q.setRPY(0, arm_pitch, 0);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transform);

    // ---------------------------
    // arm_link -> elbow_link (pitch)
    // ---------------------------
    transform.header.stamp = now;
    transform.header.frame_id = "arm_link";
    transform.child_frame_id = "elbow_link";
    transform.transform.translation.x = 0.8;  // arm length
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    q.setRPY(0, elbow_pitch, 0);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transform);

    // ---------------------------
    // elbow_link -> hand_link (pitch)
    // ---------------------------
    transform.header.stamp = now;
    transform.header.frame_id = "elbow_link";
    transform.child_frame_id = "hand_link";
    transform.transform.translation.x = 0.7;  // elbow length
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    q.setRPY(0, hand_pitch, 0);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(transform);
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExcavatorTFBroadcaster>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

