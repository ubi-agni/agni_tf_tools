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

#ifndef ROTATION_PROPERTY_H
#define ROTATION_PROPERTY_H

#include <Eigen/Geometry>
#include <stdexcept>

#include <rviz/properties/string_property.h>
#include <rviz/properties/quaternion_property.h>
#include "euler_property.h"

namespace agni_tf_tools
{

class RotationProperty: public rviz::StringProperty
{
Q_OBJECT
public:
  RotationProperty(Property* parent = 0,
                   const QString& name = QString(),
                   const Eigen::Quaterniond& value = Eigen::Quaterniond::Identity(),
                   const char *changed_slot = 0,
                   QObject* receiver = 0);

  Eigen::Quaterniond getQuaternion() const;
  virtual bool setValue(const QVariant& value);

  /** @brief Load the value of this property and/or its children from the given Config node. */
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

  /** @brief Overridden from Property to propagate read-only-ness to children. */
  virtual void setReadOnly(bool read_only);

public Q_SLOTS:
  void setQuaternion(const Eigen::Quaterniond &q);
  void setEulerAngles(double euler[3], bool normalize);
  void setEulerAngles(double e1, double e2, double e3, bool normalize);
  void setEulerAxes(const QString &axes);

private Q_SLOTS:
  void updateFromEuler();
  void updateFromQuaternion();

Q_SIGNALS:
  /** signal emitted when quaternion value has changed */
  void quaternionChanged(Eigen::Quaterniond q);
  /** signal emitted to indicate error status, e.g. to a rviz::Display */
  void statusUpdate(int, const QString&, const QString&);

private:
  void updateString();

  rviz::EulerProperty *euler_property_;
  rviz::QuaternionProperty *quaternion_property_;
  bool ignore_quaternion_property_updates_;
  bool show_euler_string_;
};

} // end namespace agni_tf_tools

#endif // ROTATION_PROPERTY_H
