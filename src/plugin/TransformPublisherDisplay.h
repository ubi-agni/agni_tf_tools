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

#include <rviz_common/rviz_common/display.hpp>
#include <rviz_common/rviz_common/frame_manager_iface.hpp>
#include <rviz_default_plugins/rviz_default_plugins/displays/interactive_markers/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// forward declarations of classes
namespace rviz_common {
namespace properties {
class Property;
class StringProperty;
class BoolProperty;
class FloatProperty;
class VectorProperty;
class TfFrameProperty;
class EnumProperty;
} // namespace properties
} // namespace rviz_common

class TransformBroadcaster;

namespace agni_tf_tools {

class RotationProperty;

class TransformPublisherDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  TransformPublisherDisplay();
  ~TransformPublisherDisplay() override;

  void reset() override;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

  void addFrameControls(visualization_msgs::msg::InteractiveMarker& im, double scale, bool interactive);
  void add6DOFControls(visualization_msgs::msg::InteractiveMarker& im);
  bool createInteractiveMarker(int type);
  bool fillPoseStamped(std_msgs::msg::Header& header, geometry_msgs::msg::Pose& pose);

protected Q_SLOTS:
  void onRefFrameChanged();
  void onAdaptTransformChanged();
  void onFramesChanged();
  void onTransformChanged();
  void onMarkerFeedback(visualization_msgs::msg::InteractiveMarkerFeedback& feedback);
  void onBroadcastEnableChanged();
  void onMarkerTypeChanged();
  void onMarkerScaleChanged();

private:
  // properties
  rviz_common::properties::VectorProperty* translation_property_;
  RotationProperty* rotation_property_;
  rviz_common::properties::BoolProperty* broadcast_property_;
  rviz_common::properties::TfFrameProperty* parent_frame_property_;
  rviz_common::properties::BoolProperty* adapt_transform_property_;
  std::string prev_parent_frame_;
  rviz_common::properties::TfFrameProperty* child_frame_property_;
  rviz_common::properties::EnumProperty* marker_property_;
  rviz_common::properties::FloatProperty* marker_scale_property_;

  // tf publisher
  TransformBroadcaster* tf_pub_;

  // interactive marker stuff
  std::shared_ptr<rviz_default_plugins::displays::InteractiveMarker> imarker_;
  bool ignore_updates_;
};

} // namespace agni_tf_tools
