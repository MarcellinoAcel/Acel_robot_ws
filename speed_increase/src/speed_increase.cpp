#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
using std::placeholders::_1;
class SpeedIncrease : public rclcpp::Node
{
public:
  SpeedIncrease() : Node("speed_increase")
  {
    publisher_axis_auto = this->create_publisher<geometry_msgs::msg::Twist>("beyond/cmd_vel", 10);
    subscriber_button = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "button", 10, std::bind(&SpeedIncrease::button_callback, this, _1));
    subscriber_axis_auto = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&SpeedIncrease::topic_callback, this, _1));
  }

private:
  float speed = 0.0;
  bool button6_pressed = false;
  bool button7_pressed = false;
  void topic_callback(const geometry_msgs::msg::Twist &msg)
  {
    auto increased_speed = geometry_msgs::msg::Twist();
    increased_speed.linear.x = msg.linear.x * speed;
    increased_speed.linear.y = msg.linear.x * speed;
    increased_speed.angular.z = msg.angular.z * speed;
    publisher_axis_auto->publish(increased_speed);
  }
  void button_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data[6] == 1 && !button6_pressed)
    {
      speed -= 0.2;
      button6_pressed = true;
    }
    else if (msg->data[6] == 0)
    {
      button6_pressed = false;
    }

    if (msg->data[7] == 1 && !button7_pressed)
    {
      speed += 0.2;
      button7_pressed = true;
    }
    else if (msg->data[7] == 0)
    {
      button7_pressed = false;
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_axis_auto;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscriber_button;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_axis_auto;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpeedIncrease>());
  rclcpp::shutdown();
  return 0;
}
