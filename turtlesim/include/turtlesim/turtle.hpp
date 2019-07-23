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

#ifndef TURTLESIM__TURTLE_HPP_
#define TURTLESIM__TURTLE_HPP_

# include <rclcpp/rclcpp.hpp>

# include <turtlesim/msg/pose.hpp>
# include <turtlesim/msg/color.hpp>
# include <geometry_msgs/msg/twist.hpp>
# include <turtlesim/srv/set_pen.hpp>
# include <turtlesim/srv/teleport_relative.hpp>
# include <turtlesim/srv/teleport_absolute.hpp>

#include <QImage>
#include <QPainter>
#include <QPen>
#include <QPointF>

# include <memory>
# include <vector>

#define PI 3.14159265

namespace turtlesim
{

class Turtle
{
public:
  Turtle(const std::shared_ptr<rclcpp::Node> nh, const QImage& turtle_image, const QPointF& pos, float orient);

  bool update(double dt, QPainter& path_painter, const QImage& path_image, qreal canvas_width, qreal canvas_height);
  void paint(QPainter &painter);
private:
  void velocityCallback(geometry_msgs::msg::Twist::SharedPtr vel);
  bool setPenCallback(
          const std::shared_ptr<turtlesim::srv::SetPen::Request>,
          const std::shared_ptr<turtlesim::srv::SetPen::Response>);
  bool teleportRelativeCallback(
          const std::shared_ptr<turtlesim::srv::TeleportRelative::Request>,
          const std::shared_ptr<turtlesim::srv::TeleportRelative::Response>);
  bool teleportAbsoluteCallback(
          const std::shared_ptr<turtlesim::srv::TeleportAbsolute::Request>,
          const std::shared_ptr<turtlesim::srv::TeleportAbsolute::Response>);

  void rotateImage();

  std::shared_ptr<rclcpp::Node> nh_;

  QImage turtle_image_;
  QImage turtle_rotated_image_;

  QPointF pos_;
  qreal orient_;

  qreal lin_vel_;
  qreal ang_vel_;
  bool pen_on_;
  QPen pen_;

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> velocity_sub_;
  std::shared_ptr<rclcpp::Publisher<turtlesim::msg::Pose>> pose_pub_;
  std::shared_ptr<rclcpp::Publisher<turtlesim::msg::Color>> color_pub_;
  std::shared_ptr<rclcpp::Service<turtlesim::srv::SetPen>> set_pen_srv_;
  std::shared_ptr<rclcpp::Service<turtlesim::srv::TeleportRelative>> teleport_relative_srv_;
  std::shared_ptr<rclcpp::Service<turtlesim::srv::TeleportAbsolute>> teleport_absolute_srv_;

  builtin_interfaces::msg::Time last_command_time_;

  float meter_;

  struct TeleportRequest
  {
    TeleportRequest(float x, float y, qreal _theta, qreal _linear, bool _relative)
    : pos(x, y)
    , theta(_theta)
    , linear(_linear)
    , relative(_relative)
    {}

    QPointF pos;
    qreal theta;
    qreal linear;
    bool relative;
  };
  typedef std::vector<TeleportRequest> V_TeleportRequest;
  V_TeleportRequest teleport_requests_;
};
typedef std::shared_ptr<Turtle> TurtlePtr;

}  // namespace turtlesim

#endif  // TURTLESIM__TURTLE_HPP_
