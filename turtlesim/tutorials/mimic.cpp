/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.  *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <memory>

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
  /* rclcpp::shutdown(); */
  /* rclcpp::init(argc, argv); */

  Mimic mimic;

  rclcpp::spin_some(mimic.node);
  /* rclcpp::spin(mimic.node); */
  /* rclcpp::shutdown(); */
  rclcpp::init(argc, argv);

  mimic = Mimic();

  rclcpp::spin(mimic.node);
}

