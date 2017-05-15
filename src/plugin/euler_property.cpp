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

#include <QStringList>
#include <rviz/properties/float_property.h>
#include <rviz/properties/status_property.h>
#include <angles/angles.h>
#include <boost/format.hpp>
#include <boost/assign/list_of.hpp>
#include "euler_property.h"

namespace rviz
{

EulerProperty::invalid_axes::invalid_axes(const std::string &msg) :
  std::invalid_argument(msg)
{
}

EulerProperty::EulerProperty(Property* parent, const QString& name,
                             const Eigen::Quaterniond& value,
                             const char *changed_slot,
                             QObject* receiver)
  : Property(name, QVariant(),
             "Angles specified in degrees.\n"
             "Choose axes with spec like xyz, zxz, or rpy.\n"
             "Composition w.r.t. the static or rotating frame\n"
             "is selected by prefixing with 's' or 'r' (default).",
             parent, changed_slot, receiver)
  , quaternion_(value)
  , ignore_child_updates_(false)
  , angles_read_only_(false)
  , update_string_(true)
{
  euler_[0] = new FloatProperty("", 0, "rotation angle about first axis", this);
  euler_[1] = new FloatProperty("", 0, "rotation angle about second axis", this);
  euler_[2] = new FloatProperty("", 0, "rotation angle about third axis", this);
  setEulerAxes("rpy");

  for (int i=0; i < 3; ++i) {
    connect(euler_[i], SIGNAL(aboutToChange()), this, SLOT(emitAboutToChange()));
    connect(euler_[i], SIGNAL(changed()), this, SLOT(updateFromChildren()));
  }
}

void EulerProperty::setQuaternion(const Eigen::Quaterniond& q)
{
  if (quaternion_.isApprox(q)) return;
  updateAngles(q); // this will also emit changed signals (in setEulerAngles)
}

void EulerProperty::setEulerAngles(double euler[], bool normalize)
{
  Eigen::Quaterniond q;
  if (fixed_)
    q = Eigen::AngleAxisd(euler[2], Eigen::Vector3d::Unit(axes_[2])) *
        Eigen::AngleAxisd(euler[1], Eigen::Vector3d::Unit(axes_[1])) *
        Eigen::AngleAxisd(euler[0], Eigen::Vector3d::Unit(axes_[0]));
  else
    q = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::Unit(axes_[0])) *
        Eigen::AngleAxisd(euler[1], Eigen::Vector3d::Unit(axes_[1])) *
        Eigen::AngleAxisd(euler[2], Eigen::Vector3d::Unit(axes_[2]));

  if (normalize) setQuaternion(q);
  else {
    for (int i=0; i < 3; ++i) {
      float deg = angles::to_degrees(euler[i]);
      if (!Eigen::internal::isApprox(deg, euler_[i]->getFloat())) {
        update_string_ = true;
        if (!ignore_child_updates_)
          euler_[i]->setValue(deg);
      }
    }

    if (!quaternion_.isApprox(q)) {
      Q_EMIT aboutToChange();
      quaternion_ = q;
      Q_EMIT quaternionChanged(q);
    } else if (update_string_) {
      Q_EMIT aboutToChange();
    } else { // there is nothing to update at all
      return;
    }
    updateString();
    Q_EMIT changed();
  }
}

void EulerProperty::setEulerAngles(double e1, double e2, double e3, bool normalize)
{
  double euler[3] = {e1,e2,e3};
  setEulerAngles(euler, normalize);
}

void EulerProperty::setEulerAxes(const QString &axes_spec)
{
  static const std::vector<QString> xyzNames = boost::assign::list_of("x")("y")("z");
  static const std::vector<QString> rpyNames = boost::assign::list_of("roll")("pitch")("yaw");
  const std::vector<QString> *names = &xyzNames;

  if (axes_string_ == axes_spec) return;
  QString sAxes = axes_spec;
  if (sAxes == "rpy") {
    sAxes = "sxyz";
    names = &rpyNames;
  } else if (sAxes == "ypr") {
    sAxes = "rzyx";
    names = &rpyNames;
  }

  // static or rotated frame order?
  QString::iterator pc = sAxes.begin();
  bool fixed;
  if (*pc == 's') fixed = true;
  else if (*pc == 'r') fixed = false;
  else {fixed = false; --pc;}
  ++pc; // advance to first axis char

  // need to have 3 axes specs
  if (sAxes.end() - pc != 3)
    throw invalid_axes((boost::format("Invalid axes spec: %s. Expecting 3 chars from [xyz]")
                        % axes_spec.toStdString()).str());

  // parse axes specs into indexes
  uint axes[3];
  for (int i=0; i < 3; ++i, ++pc) {
    int idx = pc->toLatin1() - 'x';
    if (idx < 0 || idx > 2)
      throw invalid_axes((boost::format("invalid axis char: %c (only xyz allowed)") % pc->unicode()).str());
    if (i > 0 && axes[i-1] == idx)
      throw invalid_axes("consecutive axes need to be different");
    axes[i] = idx;
  }

  // everything OK: accept changes
  axes_string_ = axes_spec;
  fixed_ = fixed;
  for (int i=0; i < 3; ++i) {
    axes_[i] = axes[i];
    euler_[i]->setName((*names)[axes[i]]);
  }

  // finally compute euler angles matching the new axes
  update_string_ = true;
  updateAngles(quaternion_);
}

bool EulerProperty::setValue(const QVariant& value)
{
  static const QString statusAxes ("Euler axes");
  static const QString statusAngles ("Euler angles");

  const QRegExp axesSpec("\\s*([a-z]+)\\s*:?");
  QString s = value.toString();

  // parse axes spec
  if (axesSpec.indexIn(s) != -1) {
    try {
      setEulerAxes(axesSpec.cap(1));
      Q_EMIT statusUpdate(StatusProperty::Ok, statusAxes, axes_string_);
    } catch (const invalid_axes &e) {
      Q_EMIT statusUpdate(StatusProperty::Warn, statusAxes, e.what());
      return false;
    }
    s = s.mid(axesSpec.matchedLength());
  }

  // in read-only mode only allow to change axes, but not angles
  if (angles_read_only_) {
    Q_EMIT statusUpdate(StatusProperty::Warn, statusAngles, "read-only");
    return true;
  }

  if (s.trimmed().isEmpty())
    return true; // allow change of axes only

  // parse angles
  QStringList strings = s.split(';');
  double euler[3];
  bool ok = true;
  for (int i=0; i < 3 && ok; ++i)
  {
    if (i < strings.size())
      euler[i] = angles::from_degrees(strings[i].toDouble(&ok));
    else // providing a single value will set all angles to this (1st) value
      euler[i] = euler[0];
  }
  if (!ok)
  {
    Q_EMIT statusUpdate(StatusProperty::Warn, statusAngles,
                        "failed to parse angle value");
    return false;
  }
  if (strings.size() != 3 && strings.size() != 1) {
    Q_EMIT statusUpdate(StatusProperty::Warn, statusAngles,
                        "expecting 3 semicolon-separated values");
    return false;
  }

  Q_EMIT statusUpdate(StatusProperty::Ok, statusAngles, "");
  setEulerAngles(euler, false);
}

void EulerProperty::updateFromChildren()
{
  if (ignore_child_updates_) return;
  double euler[3];
  for (int i = 0; i < 3; ++i)
    euler[i] = angles::from_degrees(euler_[i]->getValue().toFloat());

  ignore_child_updates_ = true;
  setEulerAngles(euler, false);
  ignore_child_updates_ = false;
}

void EulerProperty::emitAboutToChange()
{
  if (ignore_child_updates_) return;
  Q_EMIT aboutToChange();
}

void EulerProperty::updateAngles(const Eigen::Quaterniond &q)
{
  Eigen::Vector3d e;
  if (fixed_) {
    e = q.matrix().eulerAngles(axes_[2], axes_[1], axes_[0]);
    std::swap(e[0], e[2]);
  } else
    e = q.matrix().eulerAngles(axes_[0], axes_[1], axes_[2]);
  setEulerAngles(e.data(), false);
}

void EulerProperty::updateString()
{
  QString s = QString("%1: %2; %3; %4")
      .arg(axes_string_)
      .arg(euler_[0]->getFloat(), 0, 'f', 1)
      .arg(euler_[1]->getFloat(), 0, 'f', 1)
      .arg(euler_[2]->getFloat(), 0, 'f', 1);
  value_ = s.replace(".0", "");
  update_string_ = false; // reset update flag
}

void EulerProperty::load(const Config& config)
{
  QString axes;
  float euler[3];
  if (config.mapGetString("axes", &axes) &&
      config.mapGetFloat("e1", euler+0 ) &&
      config.mapGetFloat("e2", euler+1 ) &&
      config.mapGetFloat("e3", euler+2 ))
  {
    // Setting the value once is better than letting the Property class
    // load all parameters independently, which would result in 4 update calls
    setEulerAxes(axes);
    for (int i=0; i < 3; ++i)
      euler[i] = angles::from_degrees(euler[i]);
    setEulerAngles(euler[0], euler[1], euler[2], false);
  }
}

void EulerProperty::save(Config config) const
{
  // Saving the child values explicitly avoids having Property::save()
  // save the summary string version of the property.
  config.mapSetValue("axes", axes_string_);
  config.mapSetValue("e1", euler_[0]->getValue());
  config.mapSetValue("e2", euler_[1]->getValue());
  config.mapSetValue("e3", euler_[2]->getValue());
}

void EulerProperty::setReadOnly(bool read_only)
{
  // read-only mode should allow for changing axes, but not angles
  angles_read_only_ = read_only;
  // do not pass read-only to base class, but only to children
  for (int i=0; i < 3; ++i)
    euler_[i]->setReadOnly(read_only);
}

} // end namespace rviz
