/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
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

#ifndef TURTLESIM__TURTLE_FRAME_HPP_
#define TURTLESIM__TURTLE_FRAME_HPP_

#include <rclcpp/rclcpp.hpp>

#include "turtle.hpp"
#include <std_srvs/srv/empty.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/kill.hpp>

#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>

#include <string>
#include <memory>
#include <map>

namespace turtlesim
{

class TurtleFrame : public QFrame
{
  Q_OBJECT
public:
  TurtleFrame(rclcpp::Node::SharedPtr nh, QWidget* parent = 0, Qt::WindowFlags f = 0);
  ~TurtleFrame();

  std::string spawnTurtle(const std::string& name, float x, float y, float angle);
  std::string spawnTurtle(const std::string& name, float x, float y, float angle, size_t index);

protected:
  void paintEvent(QPaintEvent* event);

private slots:
  void onUpdate();

private:
  void updateTurtles();
  void clear();
  bool hasTurtle(const std::string& name);

  bool clearCallback(
          const std::shared_ptr<std_srvs::srv::Empty::Request>,
          const std::shared_ptr<std_srvs::srv::Empty::Response>);
  bool resetCallback(
          const std::shared_ptr<std_srvs::srv::Empty::Request>,
          const std::shared_ptr<std_srvs::srv::Empty::Response>);
  bool spawnCallback(
          const std::shared_ptr<turtlesim::srv::Spawn::Request>,
          const std::shared_ptr<turtlesim::srv::Spawn::Response>);
  bool killCallback(
          const std::shared_ptr<turtlesim::srv::Kill::Request>,
          const std::shared_ptr<turtlesim::srv::Kill::Response>);

  std::shared_ptr<rclcpp::Node> nh_;
  QTimer* update_timer_;
  QImage path_image_;
  QPainter path_painter_;

  uint64_t frame_count_;

  rclcpp::Time last_turtle_update_;

  std::shared_ptr<rclcpp::Service<std_srvs::srv::Empty>> clear_srv_;
  std::shared_ptr<rclcpp::Service<std_srvs::srv::Empty>> reset_srv_;
  std::shared_ptr<rclcpp::Service<turtlesim::srv::Spawn>> spawn_srv_;
  std::shared_ptr<rclcpp::Service<turtlesim::srv::Kill>> kill_srv_;

  typedef std::map<std::string, TurtlePtr> M_Turtle;
  M_Turtle turtles_;
  uint32_t id_counter_;

  QVector<QImage> turtle_images_;

  float meter_;
  float width_in_meters_;
  float height_in_meters_;
};

}  // namespace turtlesim

#endif  // TURTLESIM__TURTLE_FRAME_HPP_
