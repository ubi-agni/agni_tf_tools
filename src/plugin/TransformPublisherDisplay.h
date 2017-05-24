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
#include <geometry_msgs/PoseStamped.h>

// forward declarations of classes
namespace rviz
{
class Property;
class StringProperty;
class BoolProperty;
class FloatProperty;
class VectorProperty;
class TfFrameProperty;
class EnumProperty;

class InteractiveMarker;
}

class TransformBroadcaster;

namespace agni_tf_tools
{
// needed because rviz::InteractiveMarker::statusUpdate is declared without rviz namespace
using rviz::StatusProperty;

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

  void addFrameControls(visualization_msgs::InteractiveMarker &im, double scale, bool interactive);
  void add6DOFControls(visualization_msgs::InteractiveMarker &im);
  bool createInteractiveMarker(int type);
  bool fillPoseStamped(std_msgs::Header &header, geometry_msgs::Pose &pose);

protected Q_SLOTS:
  void setStatus(int level, const QString &name, const QString &text);
  void setStatusStd(StatusProperty::Level, const std::string &name, const std::string &text);
  void onRefFrameChanged();
  void onAdaptTransformChanged();
  void onFramesChanged();
  void onTransformChanged();
  void onMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback);
  void onBroadcastEnableChanged();
  void onMarkerTypeChanged();
  void onMarkerScaleChanged();

private:
  // properties
  rviz::VectorProperty *translation_property_;
  RotationProperty *rotation_property_;
  rviz::BoolProperty *broadcast_property_;
  rviz::TfFrameProperty *parent_frame_property_;
  rviz::BoolProperty *adapt_transform_property_;
  std::string prev_parent_frame_;
  rviz::TfFrameProperty *child_frame_property_;
  rviz::EnumProperty *marker_property_;
  rviz::FloatProperty *marker_scale_property_;

  // tf publisher
  TransformBroadcaster *tf_pub_;

  // interactive marker stuff
  boost::shared_ptr<rviz::InteractiveMarker> imarker_;
  Ogre::SceneNode *marker_node_;
  bool ignore_updates_ ;
};

} // namespace rviz_cbf_plugin
