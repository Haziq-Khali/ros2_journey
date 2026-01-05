#include <memory>
#include <string>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class ExcavatorTFListener : public rclcpp::Node
{
public:
  ExcavatorTFListener(const std::string &parent_frame, const std::string &child_frame)
  : Node("excavator_tf_listener"),
    parent_frame_(parent_frame),
    child_frame_(child_frame),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    timer_ = this->create_wall_timer(500ms, std::bind(&ExcavatorTFListener::onTimer, this));

    RCLCPP_INFO(this->get_logger(),
                "Excavator TF Listener started.\nParent: %s\nChild: %s",
                parent_frame_.c_str(), child_frame_.c_str());
  }

private:
  void onTimer()
  {
    try
    {
      geometry_msgs::msg::TransformStamped transform =
        tf_buffer_.lookupTransform(parent_frame_, child_frame_, tf2::TimePointZero);

      // --- Extract translation ---
      double x = transform.transform.translation.x;
      double y = transform.transform.translation.y;
      double z = transform.transform.translation.z;

      // --- Extract rotation (quaternion -> RPY) ---
      tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      // --- Print output ---
      RCLCPP_INFO(this->get_logger(),
        "\n[%s -> %s]\n"
        "Translation: x=%.2f, y=%.2f, z=%.2f\n"
        "Rotation (radians): roll=%.2f, pitch=%.2f, yaw=%.2f\n"
        "Rotation (degrees): roll=%.1f°, pitch=%.1f°, yaw=%.1f°",
        parent_frame_.c_str(), child_frame_.c_str(),
        x, y, z,
        roll, pitch, yaw,
        roll * 180.0 / M_PI,
        pitch * 180.0 / M_PI,
        yaw * 180.0 / M_PI);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform %s -> %s: %s",
                  parent_frame_.c_str(), child_frame_.c_str(), ex.what());
    }
  }

  std::string parent_frame_;
  std::string child_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::string parent_frame = "world";
  std::string child_frame = "hand_link";

  // Support command line arguments: e.g.
  // ros2 run excavator_tf excavator_tf_listener world hand_link
  if (argc >= 3) {
    parent_frame = argv[1];
    child_frame = argv[2];
  }

  auto node = std::make_shared<ExcavatorTFListener>(parent_frame, child_frame);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
