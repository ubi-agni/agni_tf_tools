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

#include "TransformPublisherDisplay.h"
#include "TransformBroadcaster.h"
#include "rotation_property.h"

#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/display_factory.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/default_plugin/interactive_markers/interactive_marker.h>
#include <tf/transform_listener.h>

namespace vm = visualization_msgs;
const std::string MARKER_NAME = "marker";

namespace agni_tf_tools
{

void static updatePose(geometry_msgs::Pose &pose,
                       const Eigen::Quaterniond &q,
                       Ogre::Vector3 p = Ogre::Vector3::ZERO)
{
  pose.orientation.w = q.w();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();

  pose.position.x = p.x;
  pose.position.y = p.y;
  pose.position.z = p.z;
}


TransformPublisherDisplay::TransformPublisherDisplay()
  : rviz::Display()
  , imarker_(NULL)
  , ignore_updates_(false)
{
  translation_property_ = new rviz::VectorProperty("translation", Ogre::Vector3::ZERO, "", this);
  rotation_property_ = new RotationProperty(this, "rotation");

  parent_frame_property_ = new rviz::TfFrameProperty(
        "parent frame", rviz::TfFrameProperty::FIXED_FRAME_STRING, "", this,
        0, true, SLOT(onFramesChanged()), this);
  broadcast_property_ = new rviz::BoolProperty("publish transform", true, "", this,
                                               SLOT(onBroadcastChanged()), this);
  child_frame_property_ = new rviz::TfFrameProperty(
        "child frame", "", "", broadcast_property_,
        0, false, SLOT(onFramesChanged()), this);

  connect(translation_property_, SIGNAL(changed()), this, SLOT(onTransformChanged()));
  connect(rotation_property_, SIGNAL(quaternionChanged(Eigen::Quaterniond)), this, SLOT(onTransformChanged()));
  tf_pub_ = new TransformBroadcaster("", "", this);
}

TransformPublisherDisplay::~TransformPublisherDisplay()
{
}

void TransformPublisherDisplay::onInitialize()
{
  Display::onInitialize();
  parent_frame_property_->setFrameManager(context_->getFrameManager());
  child_frame_property_->setFrameManager(context_->getFrameManager());

  // show some children by default
  this->expand();
  broadcast_property_->expand();
}

void TransformPublisherDisplay::reset()
{
  Display::reset();
}

void TransformPublisherDisplay::onEnable()
{
  Display::onEnable();
}

void TransformPublisherDisplay::onDisable()
{
  Display::onDisable();
}

void TransformPublisherDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);
  // create marker if not yet done
  if (!imarker_ && !createInteractiveMarker())
    setStatusStd(StatusProperty::Warn, MARKER_NAME, "Waiting for tf");
  else
    imarker_->update(wall_dt); // get online marker updates
}


static vm::Marker createArrowMarker(double scale,
                                    const Eigen::Vector3d &dir,
                                    const QColor &color) {
  vm::Marker marker;

  marker.type = vm::Marker::ARROW;
  marker.scale.x = scale;
  marker.scale.y = 0.1*scale;
  marker.scale.z = 0.1*scale;

  updatePose(marker.pose,
             Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), dir));

  marker.color.r = color.redF();
  marker.color.g = color.greenF();
  marker.color.b = color.blueF();
  marker.color.a = color.alphaF();

  return marker;
}

bool TransformPublisherDisplay::createInteractiveMarker()
{
  float scale = 0.2;

  vm::InteractiveMarker im;
  im.name = MARKER_NAME;
  im.scale = scale;
  if (!fillPoseStamped(im.header, im.pose)) return false;

  vm::InteractiveMarkerControl ctrl;
  ctrl.orientation_mode = vm::InteractiveMarkerControl::VIEW_FACING;
  ctrl.interaction_mode = vm::InteractiveMarkerControl::MOVE_ROTATE_3D;
  ctrl.independent_marker_orientation = true;
  ctrl.always_visible = true;
  ctrl.name = "drag control";

  ctrl.markers.push_back(createArrowMarker(scale, Eigen::Vector3d::UnitX(), QColor("red")));
  ctrl.markers.push_back(createArrowMarker(scale, Eigen::Vector3d::UnitY(), QColor("green")));
  ctrl.markers.push_back(createArrowMarker(scale, Eigen::Vector3d::UnitZ(), QColor("blue")));

  im.controls.push_back(ctrl);


  if (imarker_) delete imarker_;
  imarker_ = new rviz::InteractiveMarker(getSceneNode(), context_);
  connect(imarker_, SIGNAL(userFeedback(visualization_msgs::InteractiveMarkerFeedback&)),
          this, SLOT(onMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback&)));
  connect(imarker_, SIGNAL(statusUpdate(StatusProperty::Level,std::string,std::string)),
          this, SLOT(setStatusStd(StatusProperty::Level,std::string,std::string)));

  setStatusStd(rviz::StatusProperty::Ok, MARKER_NAME, "Ok");
  imarker_->processMessage(im);
  imarker_->setShowVisualAids(false);
  imarker_->setShowAxes(false);
  imarker_->setShowDescription(false);

  return true;
}


bool TransformPublisherDisplay::fillPoseStamped(std_msgs::Header &header,
                                                geometry_msgs::Pose &pose)
{
  const std::string &parent_frame = parent_frame_property_->getFrameStd();
  std::string error;
  if (context_->getFrameManager()->transformHasProblems(parent_frame, ros::Time(), error))
  {
    setStatusStd(StatusProperty::Error, MARKER_NAME, error);
    return false;
  }
  setStatusStd(StatusProperty::Ok, MARKER_NAME, "");

  const Eigen::Quaterniond &q = rotation_property_->getQuaternion();
  const Ogre::Vector3 &p = translation_property_->getVector();
  updatePose(pose, q, p);
  header.frame_id = parent_frame;
  // frame-lock marker to update marker pose with frame updates
  header.stamp = ros::Time();
  return true;
}

void TransformPublisherDisplay::setStatusStd(rviz::StatusProperty::Level level,
                                             const std::string &name, const std::string &text)
{
  Display::setStatusStd(level, name, text);
}

void TransformPublisherDisplay::onFramesChanged()
{
  if (ignore_updates_) return;

  // update marker pose
  vm::InteractiveMarkerPose marker_pose;
  fillPoseStamped(marker_pose.header, marker_pose.pose);
  if (imarker_) imarker_->processMessage(marker_pose);

  // broadcast transform
  geometry_msgs::TransformStamped tf;
  tf.header = marker_pose.header;
  tf.child_frame_id = child_frame_property_->getFrameStd();
  tf.transform.translation.x = marker_pose.pose.position.x;
  tf.transform.translation.y = marker_pose.pose.position.y;
  tf.transform.translation.z = marker_pose.pose.position.z;
  tf.transform.rotation = marker_pose.pose.orientation;
  tf_pub_->setValue(tf);
}

void TransformPublisherDisplay::onTransformChanged()
{
  if (ignore_updates_) return;

  vm::InteractiveMarkerPose marker_pose;
  fillPoseStamped(marker_pose.header, marker_pose.pose);

  // update marker pose + broadcaster pose
  if (imarker_) imarker_->processMessage(marker_pose);
  tf_pub_->setPose(marker_pose.pose);
}

void TransformPublisherDisplay::onMarkerFeedback(vm::InteractiveMarkerFeedback &feedback)
{
  if (ignore_updates_) return;
  if (feedback.event_type != vm::InteractiveMarkerFeedback::POSE_UPDATE) return;

  // convert to parent frame
  const geometry_msgs::Point &p_in = feedback.pose.position;
  const geometry_msgs::Quaternion &q_in = feedback.pose.orientation;

  tf::Stamped<tf::Pose> pose_in(tf::Transform(tf::Quaternion(q_in.x, q_in.y, q_in.z, q_in.w),
                                              tf::Vector3(p_in.x, p_in.y, p_in.z)),
                                feedback.header.stamp, feedback.header.frame_id);
  tf::Stamped<tf::Pose> pose_out;
  try {
    context_->getTFClient()->transformPose(parent_frame_property_->getFrameStd(),
                                           pose_in, pose_out);
  } catch(const std::runtime_error &e) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s': %s",
              feedback.header.frame_id.c_str(),
              parent_frame_property_->getFrameStd().c_str(),
              e.what());
    return;
  }

  const tf::Vector3 &p = pose_out.getOrigin();
  const tf::Quaternion &q = pose_out.getRotation();

  ignore_updates_ = true;
  translation_property_->setVector(Ogre::Vector3(p.x(), p.y(), p.z()));
  rotation_property_->setQuaternion(Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()));
  ignore_updates_ = false;

  updatePose(feedback.pose, rotation_property_->getQuaternion(),
             translation_property_->getVector());
  tf_pub_->setPose(feedback.pose);
}

void TransformPublisherDisplay::onBroadcastChanged()
{
  tf_pub_->setEnabled(broadcast_property_->getBool());
}

} // namespace agni_tf_tools
