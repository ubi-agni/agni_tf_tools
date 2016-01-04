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

#include <rviz/display.h>
#include <ros/ros.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

namespace rviz
{
class Property;
class StringProperty;
class BoolProperty;
class FloatProperty;
class VectorProperty;
class TfFrameProperty;

class InteractiveMarker;
}

class TransformBroadcaster;

namespace agni_tf_tools
{

class RotationProperty;

class TransformPublisherDisplay : public rviz::Display
{
  Q_OBJECT

public:
  TransformPublisherDisplay();
  ~TransformPublisherDisplay();

  void reset();

protected:
  void onInitialize();
  void onEnable();
  void onDisable();
  void update(float wall_dt, float ros_dt);

  virtual visualization_msgs::InteractiveMarker createInteractiveMarker() const;

protected Q_SLOTS:
  void onFramesChanged();
  void onTransformChanged();
  void onMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback);
  void onBroadcastChanged();

protected:
  rviz::VectorProperty *translation_property_;
  RotationProperty *rotation_property_;
  rviz::BoolProperty *broadcast_property_;
  rviz::TfFrameProperty *parent_frame_property_;
  rviz::TfFrameProperty *child_frame_property_;
  TransformBroadcaster *tf_pub_;

  // interactive marker stuff
  rviz::InteractiveMarker *imarker_;
  bool ignore_updates_ ;
};

} // namespace rviz_cbf_plugin