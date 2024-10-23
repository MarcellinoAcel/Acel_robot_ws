#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <iostream>
#include <math.h>

class PosePublisher : public rclcpp::Node
{
public:
  PosePublisher() : Node("initial_pose")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
  }

  void publish_pose(double x, double y, double yaw)
  {
    auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "map";

    message.pose.pose.position.x = x;
    message.pose.pose.position.y = y;
    message.pose.pose.position.z = 0.0;
    float q[4];
    euler_to_quat(0, 0, yaw, q);
    message.pose.pose.orientation.x=(double)q[1];
    message.pose.pose.orientation.y=(double)q[2];
    message.pose.pose.orientation.z=(double)q[3];
    message.pose.pose.orientation.w=(double)q[0];

    for (size_t i = 0; i < 36; ++i)
    {
      message.pose.covariance[i] = 0.0;
    }
    message.pose.covariance[0] = 0.1;
    message.pose.covariance[7] = 0.1;
    message.pose.covariance[35] = 0.1;

    RCLCPP_INFO(this->get_logger(), "Publishing 2D Pose Estimate: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
    publisher_->publish(message);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  const void euler_to_quat(float roll, float pitch, float yaw, float *q)
  {
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = sy * cp * sr + cy * sp * cr;
    q[3] = sy * cp * cr - cy * sp * sr;
  }
};

float toRad(float degree)
{
  return degree * M_PI / 180;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PosePublisher>();

  auto start_time = std::chrono::steady_clock::now();
  auto end_time = start_time + std::chrono::seconds(2);

  while (rclcpp::ok())
  {
    auto current_time = std::chrono::steady_clock::now();

    if (current_time >= end_time)
    {
      break;
    }

    node->publish_pose(1.7689088370349864, -1.5561960613724437, toRad(90.0));

    rclcpp::spin_some(node);

    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
  return 0;
}
