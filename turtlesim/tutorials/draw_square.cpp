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

#include <boost/bind.hpp>
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>
#include <cmath>
#include <memory>
#include <utility>

using namespace std::chrono_literals;

turtlesim::msg::Pose::SharedPtr g_pose = NULL;
auto g_goal = std::make_shared<turtlesim::msg::Pose>();
rclcpp::Node::SharedPtr nh;

enum State
{
  FORWARD,
  STOP_FORWARD,
  TURN,
  STOP_TURN,
};

State g_state = FORWARD;
State g_last_state = FORWARD;
bool g_first_goal_set = false;

#define PI 3.141592

void poseCallback(turtlesim::msg::Pose::SharedPtr pose)
{
  g_pose = pose;
}

bool hasReachedGoal()
{
  return fabsf(g_pose->x - g_goal->x) < 0.1 && fabsf(g_pose->y - g_goal->y) < 0.1 && fabsf(g_pose->theta - g_goal->theta) < 0.01;
}

bool hasStopped()
{
  return g_pose->angular_velocity < 0.0001 && g_pose->linear_velocity < 0.0001;
}

void printGoal()
{
  RCLCPP_INFO(nh->get_logger(), "New goal [%f, %f, %f]", g_goal->x, g_goal->y, g_goal->theta);
}

void commandTurtle(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub,
                   float linear,
                   float angular)
{
  auto twist = std::make_shared<geometry_msgs::msg::Twist>();
  twist->linear.x = linear;
  twist->angular.z = angular;
  twist_pub->publish(*twist);
}

void stopForward(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub)
{
  if (hasStopped())
  {
    RCLCPP_INFO(nh->get_logger(), "Reached goal");
    g_state = TURN;
    g_goal->x = g_pose->x;
    g_goal->y = g_pose->y;
    g_goal->theta = fmod(g_pose->theta + PI/2.0, 2*PI);
    // wrap g_goal.theta to [-pi, pi)
    if (g_goal->theta >= PI) g_goal->theta -= 2 * PI;
    printGoal();
  } else {
    commandTurtle(twist_pub, 0, 0);
  }
}

void stopTurn(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub)
{
  if (hasStopped())
  {
    RCLCPP_INFO(nh->get_logger(), "Reached goal");
    g_state = FORWARD;
    g_goal->x = std::cos(g_pose->theta) * 2 + g_pose->x;
    g_goal->y = std::sin(g_pose->theta) * 2 + g_pose->y;
    g_goal->theta = g_pose->theta;
    printGoal();
  } else {
    commandTurtle(twist_pub, 0, 0);
  }
}


void forward(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub)
{
  if (hasReachedGoal())
  {
    g_state = STOP_FORWARD;
    commandTurtle(twist_pub, 0, 0);
  } else {
    commandTurtle(twist_pub, 1.0, 0.0);
  }
}

void turn(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub)
{
  if (hasReachedGoal())
  {
    g_state = STOP_TURN;
    commandTurtle(twist_pub, 0, 0);
  } else {
    commandTurtle(twist_pub, 0.0, 0.4);
  }
}

void timerCallback(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub)
{
  if (!g_pose)
  {
    return;
  }

  if (!g_first_goal_set)
  {
    g_first_goal_set = true;
    g_state = FORWARD;
    g_goal->x = std::cos(g_pose->theta) * 2 + g_pose->x;
    g_goal->y = std::sin(g_pose->theta) * 2 + g_pose->y;
    g_goal->theta = g_pose->theta;
    printGoal();
  }

  if (g_state == FORWARD)
  {
    forward(twist_pub);
  } else if (g_state == STOP_FORWARD) {
    stopForward(twist_pub);
  } else if (g_state == TURN) {
    turn(twist_pub);
  } else if (g_state == STOP_TURN) {
    stopTurn(twist_pub);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("draw_square");
  auto pose_sub = nh->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 1, &poseCallback);
  auto twist_pub = nh->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
  auto reset = nh->create_client<std_srvs::srv::Empty>("reset");
  auto timer = nh->create_wall_timer(0.016s, [twist_pub]() { timerCallback(twist_pub); });  // std::bind(&timerCallback, twist_pub));

  auto empty = std::make_shared<std_srvs::srv::Empty::Request>();
  reset->async_send_request(empty);

  rclcpp::spin(nh);
}
