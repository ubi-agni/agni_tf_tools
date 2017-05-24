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
#include <rviz/properties/float_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/enum_property.h>

#include <rviz/display_factory.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/default_plugin/interactive_markers/interactive_marker.h>
#include <interactive_markers/tools.h>
#include <tf/transform_listener.h>
#include <QDebug>

namespace vm = visualization_msgs;
const std::string MARKER_NAME = "marker";

enum MARKER_TYPE { NONE, FRAME, IFRAME, DOF6 };

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
  , ignore_updates_(false)
{
  translation_property_ = new rviz::VectorProperty("translation", Ogre::Vector3::ZERO, "", this);
  rotation_property_ = new RotationProperty(this, "rotation");

  parent_frame_property_ = new rviz::TfFrameProperty(
        "parent frame", rviz::TfFrameProperty::FIXED_FRAME_STRING, "", this,
        0, true, SLOT(onRefFrameChanged()), this);
  adapt_transform_property_ = new rviz::BoolProperty(
        "adapt transformation", false,
        "Adapt transformation when changing the parent frame? "
        "If so, the marker will not move.", this,
        SLOT(onAdaptTransformChanged()), this);
  onAdaptTransformChanged();

  broadcast_property_ = new rviz::BoolProperty("publish transform", true, "", this,
                                               SLOT(onBroadcastEnableChanged()), this);
  child_frame_property_ = new rviz::TfFrameProperty(
        "child frame", "", "", broadcast_property_,
        0, false, SLOT(onFramesChanged()), this);

  connect(translation_property_, SIGNAL(changed()), this, SLOT(onTransformChanged()));
  connect(rotation_property_, SIGNAL(quaternionChanged(Eigen::Quaterniond)), this, SLOT(onTransformChanged()));
  connect(rotation_property_, SIGNAL(statusUpdate(int,QString,QString)),
          this, SLOT(setStatus(int,QString,QString)));
  tf_pub_ = new TransformBroadcaster("", "", this);

  marker_property_ = new rviz::EnumProperty("marker type", "interactive frame", "Choose which type of interactive marker to show",
                                            this, SLOT(onMarkerTypeChanged()), this);
  marker_property_->addOption("none", NONE);
  marker_property_->addOption("static frame", FRAME);
  marker_property_->addOption("interactive frame", IFRAME);
  marker_property_->addOption("6 DoF handles", DOF6);

  marker_scale_property_ = new rviz::FloatProperty("marker scale", 0.2, "", marker_property_,
                                                   SLOT(onMarkerScaleChanged()), this);
  marker_property_->hide(); // only show when marker is created
}

TransformPublisherDisplay::~TransformPublisherDisplay()
{
}

void TransformPublisherDisplay::onInitialize()
{
  Display::onInitialize();
  parent_frame_property_->setFrameManager(context_->getFrameManager());
  child_frame_property_->setFrameManager(context_->getFrameManager());
  marker_node_ = getSceneNode()->createChildSceneNode();

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
  tf_pub_->setEnabled(true);
}

void TransformPublisherDisplay::onDisable()
{
  Display::onDisable();
  tf_pub_->setEnabled(false);
  createInteractiveMarker(NONE);
}

void TransformPublisherDisplay::update(float wall_dt, float ros_dt)
{
  if (!this->isEnabled()) return;

  Display::update(wall_dt, ros_dt);
  // create marker if not yet done
  if (!imarker_ && marker_property_->getOptionInt() != NONE &&
      !createInteractiveMarker(marker_property_->getOptionInt()))
    setStatusStd(StatusProperty::Warn, MARKER_NAME, "Waiting for tf");
  else if (imarker_)
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

inline void setOrientation(geometry_msgs::Quaternion &q, double w, double x, double y, double z) {
  q.w = w;
  q.x = x;
  q.y = y;
  q.z = z;
}

void TransformPublisherDisplay::addFrameControls(vm::InteractiveMarker &im, double scale, bool interactive)
{
  vm::InteractiveMarkerControl ctrl;
  setOrientation(ctrl.orientation, 1, 0,0,0);
  ctrl.always_visible = true;
  if (interactive) {
    ctrl.orientation_mode = vm::InteractiveMarkerControl::VIEW_FACING;
    ctrl.independent_marker_orientation = true;
    ctrl.interaction_mode = vm::InteractiveMarkerControl::MOVE_ROTATE_3D;
  }
  ctrl.name = "frame";

  ctrl.markers.push_back(createArrowMarker(im.scale * scale, Eigen::Vector3d::UnitX(), QColor("red")));
  ctrl.markers.push_back(createArrowMarker(im.scale * scale, Eigen::Vector3d::UnitY(), QColor("green")));
  ctrl.markers.push_back(createArrowMarker(im.scale * scale, Eigen::Vector3d::UnitZ(), QColor("blue")));

  im.controls.push_back(ctrl);
}

void TransformPublisherDisplay::add6DOFControls(vm::InteractiveMarker &im) {
  vm::InteractiveMarkerControl ctrl;
  ctrl.always_visible = false;

  setOrientation(ctrl.orientation, 1, 1,0,0);
  ctrl.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
  ctrl.name = "x pos";
  im.controls.push_back(ctrl);
  ctrl.interaction_mode = vm::InteractiveMarkerControl::ROTATE_AXIS;
  ctrl.name = "x rot";
  im.controls.push_back(ctrl);

  setOrientation(ctrl.orientation, 1, 0,1,0);
  ctrl.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
  ctrl.name = "y pos";
  im.controls.push_back(ctrl);
  ctrl.interaction_mode = vm::InteractiveMarkerControl::ROTATE_AXIS;
  ctrl.name = "y rot";
  im.controls.push_back(ctrl);

  setOrientation(ctrl.orientation, 1, 0,0,1);
  ctrl.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
  ctrl.name = "z pos";
  im.controls.push_back(ctrl);
  ctrl.interaction_mode = vm::InteractiveMarkerControl::ROTATE_AXIS;
  ctrl.name = "z rot";
  im.controls.push_back(ctrl);
}

bool TransformPublisherDisplay::createInteractiveMarker(int type)
{
  if (type == NONE) {
    if (imarker_)
      imarker_.reset();
    return true;
  }

  float scale = marker_scale_property_->getFloat();

  vm::InteractiveMarker im;
  im.name = MARKER_NAME;
  im.scale = scale;
  if (!fillPoseStamped(im.header, im.pose)) return false;

  if (type == FRAME || type == IFRAME)
    addFrameControls(im, 1.0, type == IFRAME);
  else if (type == DOF6) {
    addFrameControls(im, 0.5, type == IFRAME);
    add6DOFControls(im);
  }

  imarker_.reset(new rviz::InteractiveMarker(marker_node_, context_));
  connect(imarker_.get(), SIGNAL(userFeedback(visualization_msgs::InteractiveMarkerFeedback&)),
          this, SLOT(onMarkerFeedback(visualization_msgs::InteractiveMarkerFeedback&)));
  connect(imarker_.get(), SIGNAL(statusUpdate(StatusProperty::Level,std::string,std::string)),
          this, SLOT(setStatusStd(StatusProperty::Level,std::string,std::string)));

  setStatusStd(rviz::StatusProperty::Ok, MARKER_NAME, "Ok");

  // fill in default controls
  interactive_markers::autoComplete(im, true);

  imarker_->processMessage(im);
  imarker_->setShowVisualAids(false);
  imarker_->setShowAxes(false);
  imarker_->setShowDescription(false);

  marker_property_->show();
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

void TransformPublisherDisplay::setStatus(int level, const QString &name, const QString &text)
{
  if (level == rviz::StatusProperty::Ok && text.isEmpty()) {
    Display::setStatus(static_cast<rviz::StatusProperty::Level>(level), name, text);
    Display::deleteStatus(name);
  } else
    Display::setStatus(static_cast<rviz::StatusProperty::Level>(level), name, text);
}

void TransformPublisherDisplay::setStatusStd(rviz::StatusProperty::Level level,
                                             const std::string &name, const std::string &text)
{
  setStatus(level, QString::fromStdString(name), QString::fromStdString(text));
}

static bool getTransform(rviz::FrameManager &fm, const std::string &frame, Eigen::Affine3d &tf)
{
  Ogre::Vector3 p = Ogre::Vector3::ZERO;
  Ogre::Quaternion q = Ogre::Quaternion::IDENTITY;

  bool success = fm.getTransform(frame, ros::Time(), p, q);
  tf = Eigen::Translation3d(p.x, p.y, p.z) * Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  return success || frame == rviz::TfFrameProperty::FIXED_FRAME_STRING.toStdString();
}

void TransformPublisherDisplay::onRefFrameChanged()
{
  // update pose to be relative to new reference frame
  Eigen::Affine3d prevRef, nextRef;
  rviz::FrameManager &fm = *context_->getFrameManager();
  if (getTransform(fm, prev_parent_frame_, prevRef) &&
      getTransform(fm, parent_frame_property_->getFrameStd(), nextRef)) {
    const Ogre::Vector3 &p = translation_property_->getVector();
    Eigen::Affine3d curPose = Eigen::Translation3d(p.x, p.y, p.z) * rotation_property_->getQuaternion();
    Eigen::Affine3d newPose = nextRef.inverse() * prevRef * curPose;
    Eigen::Vector3d t = newPose.translation();
    ignore_updates_ = true;
    translation_property_->setVector(Ogre::Vector3(t.x(), t.y(), t.z()));
    rotation_property_->setQuaternion(Eigen::Quaterniond(newPose.rotation()));
    ignore_updates_ = false;
  }
  onAdaptTransformChanged();
  onFramesChanged();
}

void TransformPublisherDisplay::onAdaptTransformChanged()
{
  if (adapt_transform_property_->getBool())
    prev_parent_frame_ = parent_frame_property_->getFrameStd();
  else
    prev_parent_frame_ = "";
}

void TransformPublisherDisplay::onFramesChanged()
{
  // update marker pose
  vm::InteractiveMarkerPose marker_pose;
  fillPoseStamped(marker_pose.header, marker_pose.pose);
  if (imarker_) imarker_->processMessage(marker_pose);

  // prepare transform for broadcasting
  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = parent_frame_property_->getFrameStd();
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

  // update marker pose + broadcast pose
  ignore_updates_ = true;
  if (imarker_) imarker_->processMessage(marker_pose);
  ignore_updates_ = false;
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

void TransformPublisherDisplay::onBroadcastEnableChanged()
{
  tf_pub_->setEnabled(broadcast_property_->getBool());
}

void TransformPublisherDisplay::onMarkerTypeChanged()
{
  createInteractiveMarker(marker_property_->getOptionInt());
}

void TransformPublisherDisplay::onMarkerScaleChanged()
{
  if (marker_scale_property_->getFloat() <= 0)
    marker_scale_property_->setFloat(0.2);
  createInteractiveMarker(marker_property_->getOptionInt());
}

} // namespace agni_tf_tools
