/*
 * Copyright (C) 2016, Bielefeld University, CITEC
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
 *
 * Author: Robert Haschke <rhaschke@techfak.uni-bielefeld.de>
 */

#pragma once

#include <QObject>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Geometry>

/** QObject wrapper for tf2_ros::StaticTransformBroadcaster
 *  to allow for signal-slot interaction
 */
class TransformBroadcaster : public QObject
{
  Q_OBJECT
public:
  explicit TransformBroadcaster(const QString &parent_frame="",
                                const QString &child_frame="",
                                QObject *parent = 0);

  const geometry_msgs::TransformStamped& value() const;
  void setValue(const geometry_msgs::TransformStamped &tf);
  void setPose(const geometry_msgs::Pose &pose);

  bool enabled() const;

public slots:
  void setEnabled(bool bEnabled=true);
  void setDisabled(bool bDisabled=true);

  void setParentFrame(const QString &frame);
  void setChildFrame(const QString &frame);

  void setPosition(const Eigen::Vector3d &p);
  void setQuaternion(const Eigen::Quaterniond &q);

  void setPosition(double x, double y, double z);
  void setQuaternion(double x, double y, double z, double w);

protected:
  void send();
  void check();

private:
  tf2_ros::StaticTransformBroadcaster broadcaster_;
  geometry_msgs::TransformStamped msg_;
  bool valid_;
  bool enabled_;
};
