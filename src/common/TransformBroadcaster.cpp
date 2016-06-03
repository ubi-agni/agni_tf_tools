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

#include "TransformBroadcaster.h"

TransformBroadcaster::TransformBroadcaster(const QString &parent_frame, const QString &child_frame, QObject *parent) :
  QObject(parent), valid_(false), enabled_(false)
{
  setPosition(0,0,0);
  setQuaternion(0,0,0,1);

  setParentFrame(parent_frame);
  setChildFrame(child_frame);

  enabled_ = true;
  check(); send();
}

const geometry_msgs::TransformStamped &TransformBroadcaster::value() const
{
  return msg_;
}

void TransformBroadcaster::setValue(const geometry_msgs::TransformStamped &tf)
{
  msg_ = tf;
  check(); send();
}

void TransformBroadcaster::setPose(const geometry_msgs::Pose &pose)
{
  bool old = enabled_;
  enabled_ = false;

  const geometry_msgs::Point &p = pose.position;
  setPosition(p.x, p.y, p.z);

  const geometry_msgs::Quaternion &q = pose.orientation;
  setQuaternion(q.x, q.y, q.z, q.w);

  enabled_ = old;
  send();
}

bool TransformBroadcaster::enabled() const
{
  return enabled_;
}

void TransformBroadcaster::setEnabled(bool bEnabled)
{
  enabled_ = bEnabled;
  check(); send();
}

void TransformBroadcaster::setDisabled(bool bDisabled)
{
  setEnabled(!bDisabled);
}

void TransformBroadcaster::setParentFrame(const QString &frame)
{
  msg_.header.frame_id = frame.toStdString();
  check(); send();
}

void TransformBroadcaster::setChildFrame(const QString &frame)
{
  msg_.child_frame_id = frame.toStdString();
  check(); send();
}

void TransformBroadcaster::setPosition(const Eigen::Vector3d &p)
{
  setPosition(p.x(), p.y(), p.z());
}

void TransformBroadcaster::setQuaternion(const Eigen::Quaterniond &q)
{
  setQuaternion(q.x(), q.y(), q.z(), q.w());
}

void TransformBroadcaster::setPosition(double x, double y, double z)
{
  msg_.transform.translation.x = x;
  msg_.transform.translation.y = y;
  msg_.transform.translation.z = z;
  send();
}

void TransformBroadcaster::setQuaternion(double x, double y, double z, double w)
{
  msg_.transform.rotation.x = x;
  msg_.transform.rotation.y = y;
  msg_.transform.rotation.z = z;
  msg_.transform.rotation.w = w;
  send();
}

void TransformBroadcaster::send()
{
  if (enabled_ && valid_) {
    msg_.header.stamp = ros::Time::now();
    ++msg_.header.seq;
    broadcaster_.sendTransform(msg_);
    ros::spinOnce();
  }
}

void TransformBroadcaster::check()
{
  valid_ =
      !msg_.header.frame_id.empty() &&
      !msg_.child_frame_id.empty() &&
      msg_.header.frame_id != msg_.child_frame_id;
}
