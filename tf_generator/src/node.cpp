#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <functional>
#include <memory>
#include <string>

const rclcpp::QoS qos_keep_all = rclcpp::SensorDataQoS().keep_all().reliable();

class TFGenerator : public rclcpp::Node
{
public:
  TFGenerator()
  : Node("tf_generator"),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)),
    subscription_(this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "input_pose", qos_keep_all,
        std::bind(&TFGenerator::PublishMapToBaseLink, this, std::placeholders::_1)))
  {
  }

private:
  void PublishMapToBaseLink(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg)
  {
    Eigen::Isometry3d isometry3d;
    tf2::fromMsg(pose_msg->pose, isometry3d);

    geometry_msgs::msg::TransformStamped transform = tf2::eigenToTransform(isometry3d);
    transform.header = pose_msg->header;
    transform.child_frame_id = "base_link";

    tf_broadcaster_->sendTransform(transform);
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFGenerator>());
  rclcpp::shutdown();
  return 0;
}

