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

#include "rotation_property.h"

using namespace rviz;

namespace agni_tf_tools
{

RotationProperty::RotationProperty(Property* parent, const QString& name,
                                   const Eigen::Quaterniond& value,
                                   const char *changed_slot,
                                   QObject* receiver)
   : StringProperty(name, "",
                    "Orientation specification using Euler angles or a quaternion.",
                    parent, changed_slot, receiver)
   , ignore_quaternion_property_updates_(false)
   , show_euler_string_(true)
{
  euler_property_ = new EulerProperty(this, "Euler angles", value);
  quaternion_property_ = new rviz::QuaternionProperty("quaternion",
                                                      Ogre::Quaternion(value.w(), value.x(), value.y(), value.z()),
                                                      "order: x, y, z, w", this);
  connect(euler_property_, SIGNAL(changed()), this, SLOT(updateFromEuler()));
  connect(quaternion_property_, SIGNAL(changed()), this, SLOT(updateFromQuaternion()));
  // forward status signal from EulerProperty
  connect(euler_property_, SIGNAL(statusUpdate(int,QString,QString)),
          this, SIGNAL(statusUpdate(int,QString,QString)));
  // forward quaternion updates
  connect(euler_property_, SIGNAL(quaternionChanged(Eigen::Quaterniond)),
          this, SIGNAL(quaternionChanged(Eigen::Quaterniond)));
  updateString();
}

Eigen::Quaterniond RotationProperty::getQuaternion() const
{
  return euler_property_->getQuaternion();
}

void RotationProperty::setQuaternion(const Eigen::Quaterniond& q)
{
  Eigen::Quaterniond qn = q.normalized();
  if (getQuaternion().isApprox(qn)) return;
  // EulerProperty is considered "master".
  // EulerProperty update will trigger QuaternionProperty update too
  euler_property_->setQuaternion(qn);
}

void RotationProperty::updateFromEuler()
{
  const Eigen::Quaterniond q = euler_property_->getQuaternion();
  // do not update QuaternionProperty
  if (!ignore_quaternion_property_updates_) {
    quaternion_property_->setQuaternion(Ogre::Quaternion(q.w(), q.x(), q.y(), q.z()));
  }
  show_euler_string_ = true;
  updateString();
}

void RotationProperty::updateFromQuaternion()
{
  // protect from infinite update loop
  if (ignore_quaternion_property_updates_) return;

  const Ogre::Quaternion &q = quaternion_property_->getQuaternion();
  Eigen::Quaternion<Ogre::Real> eigen_q(q.w, q.x, q.y, q.z);

  // only update if changes are within accuracy range
  if (eigen_q.isApprox(getQuaternion().cast<Ogre::Real>())) return;

  ignore_quaternion_property_updates_ = true;
  setQuaternion(eigen_q.cast<double>());
  ignore_quaternion_property_updates_ = false;

  show_euler_string_ = false;
  updateString();
}

void RotationProperty::setEulerAngles(double euler[], bool normalize)
{
  euler_property_->setEulerAngles(euler, normalize);
}

void RotationProperty::setEulerAngles(double e1, double e2, double e3, bool normalize)
{
  euler_property_->setEulerAngles(e1, e2, e3, normalize);
}

void RotationProperty::setEulerAxes(const QString &axes_spec)
{
  euler_property_->setEulerAxes(axes_spec);
}

bool RotationProperty::setValue(const QVariant& value)
{
  // forward parsing to either Quaternion- or EulerProperty
  const QRegExp quatSpec("\\s*(quat:)?([^;]+;){3}");
  QString s = value.toString();
  if (quatSpec.indexIn(s) != -1)
  {
    s = s.mid(quatSpec.cap(1).length());
    return quaternion_property_->setValue(s);
  }
  return euler_property_->setValue(value);
}

void RotationProperty::updateString()
{
  QString euler = euler_property_->getValue().toString();
  QString quat  = QString("quat: ") + quaternion_property_->getValue().toString();
  QString s = show_euler_string_ ? euler : quat;
  if (getString() != s) {
    Q_EMIT aboutToChange();
    value_ = s;
    Q_EMIT changed();
  }
}

void RotationProperty::load(const Config& config)
{
  // save/load from EulerProperty. This handles both, quaternion and euler axes.
  euler_property_->load(config);
}

void RotationProperty::save(Config config) const
{
  // save/load from EulerProperty. This handles both, quaternion and euler axes.
  euler_property_->save(config);
}

void RotationProperty::setReadOnly(bool read_only)
{
  euler_property_->setReadOnly(read_only);
  quaternion_property_->setReadOnly(read_only);
}

} // end namespace rviz
