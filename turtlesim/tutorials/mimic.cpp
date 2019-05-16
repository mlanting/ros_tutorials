#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

class Mimic
{
public:
  Mimic();
  rclcpp::Node::SharedPtr node;

private:
  void poseCallback(const turtlesim::msg::Pose::ConstSharedPtr pose);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
};

Mimic::Mimic()
{
  node = rclcpp::Node::make_shared("turtle_mimic");
  auto input_node = node->create_sub_node("input");
  auto output_node = node->create_sub_node("output");
  twist_pub_ = output_node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  pose_sub_ = input_node->create_subscription<turtlesim::msg::Pose>("pose", 1, std::bind(&Mimic::poseCallback, this, _1));
}

void Mimic::poseCallback(const turtlesim::msg::Pose::ConstSharedPtr pose)
{
  auto twist = std::make_shared<geometry_msgs::msg::Twist>();
  twist->angular.z = pose->angular_velocity;
  twist->linear.x = pose->linear_velocity;
  twist_pub_->publish(*twist);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  Mimic mimic;

  rclcpp::spin(mimic.node);
}

