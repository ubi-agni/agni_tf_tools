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
#include "euler_property.h"

#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/display_factory.h>
#include <rviz/display_context.h>

namespace agni_tf_tools
{

TransformPublisherDisplay::TransformPublisherDisplay() :
  rviz::Display()
{
  translation_property_ = new rviz::VectorProperty("translation", Ogre::Vector3::ZERO, "", this);
  rviz::Property *rot = new rviz::Property("rotation", QVariant(), "", this);
  quaternion_property_ = new rviz::QuaternionProperty("quaternion", Ogre::Quaternion::IDENTITY, "", rot);
  new rviz::EulerProperty("Euler angles", Eigen::Quaterniond::Identity(), rot);

  parent_frame_property_ = new rviz::TfFrameProperty("parent frame", "", "", this, 0, false,
                                                     SLOT(updateFrames()), this);
  broadcast_property_ = new rviz::BoolProperty("publish transform", true, "", this);
  child_frame_property_ = new rviz::TfFrameProperty("child frame", "", "", broadcast_property_, 0, false,
                                                    SLOT(updateFrames()), this);

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

void TransformPublisherDisplay::updateFrames()
{
  tf_pub_->setParentFrame(parent_frame_property_->getValue().toString());
  tf_pub_->setChildFrame(child_frame_property_->getValue().toString());
}

} // namespace agni_tf_tools
