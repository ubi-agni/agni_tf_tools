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

#ifndef EULER_PROPERTY_H
#define EULER_PROPERTY_H

#include <Eigen/Geometry>
#include <stdexcept>

#include "rviz/properties/property.h"

namespace rviz
{

class FloatProperty;

class EulerProperty: public Property
{
  Q_OBJECT
public:
  class invalid_axes : public std::invalid_argument {
  public:
    invalid_axes(const std::string &msg);
  };

  EulerProperty(Property* parent = 0,
                const QString& name = QString(),
                const Eigen::Quaterniond& value = Eigen::Quaterniond::Identity(),
                const char *changed_slot = 0,
                QObject* receiver = 0);

  Eigen::Quaterniond getQuaternion() const {return quaternion_;}
  virtual bool setValue(const QVariant& value);

  /** @brief Load the value of this property and/or its children from
   * the given Config node. */
  virtual void load(const Config& config);
  virtual void save(Config config) const;

  /** @brief Overridden from Property to propagate read-only-ness to children. */
  virtual void setReadOnly(bool read_only);
  bool getAnglesReadOnly() {return angles_read_only_;}

public Q_SLOTS:
  void setQuaternion(const Eigen::Quaterniond &q);
  void setEulerAngles(double euler[3], bool normalize);
  void setEulerAngles(double e1, double e2, double e3, bool normalize);
  /** select Euler axes from string, allowed values are "rpy", "ypr", x,y,z
   *  r or s in front of x,y,z chooses application order,
   *  i.e. with respect to rotated or fixed frame
   */
  void setEulerAxes(const QString &axes_spec);

private Q_SLOTS:
  void updateFromChildren();
  void emitAboutToChange();

Q_SIGNALS:
  /** signal emitted when quaternion value has changed */
  void quaternionChanged(Eigen::Quaterniond q);
  /** signal emitted when there was an error, e.g. with Euler axes */
  void statusUpdate(int, const QString&, const QString&);

private:
  void updateAngles(const Eigen::Quaterniond &q);
  void updateString();

  Eigen::Quaterniond quaternion_;
  QString   axes_string_;
  uint      axes_[3]; // unit axis index for i-th rotation
  bool      fixed_;
  FloatProperty* euler_[3];
  bool ignore_child_updates_;
  bool angles_read_only_;
  bool update_string_; // do we have any changes triggering an updateString()?
};

} // end namespace rviz

#endif // EULER_PROPERTY_H
