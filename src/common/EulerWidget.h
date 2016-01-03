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

#include <QWidget>
#include <Eigen/Geometry>

namespace Ui {
class EulerWidget;
}

class EulerWidget : public QWidget
{
  Q_OBJECT
public:
  enum Axis {X = 0, Y = 1, Z = 2};

  explicit EulerWidget(QWidget *parent = 0);

  const Eigen::Quaterniond &value() const;

  /// retrieve indices of axes selected in GUI
  void getGuiAxes(uint a[3]) const;
  /// retrieve angles from GUI
  void getGuiAngles(double e[]) const;

signals:
  /// quaternion value has changed
  void valueChanged(const Eigen::Quaterniond &q);
  /// euler axis selection changed
  void axesChanged(uint a1, uint a2, uint a3);

public slots:
  void setValue(const Eigen::Quaterniond &q);
  void setEulerAngles(double e1, double e2, double e3, bool normalize);
  void setEulerAxes(uint a1, uint a2, uint a3);

protected slots:
  void axisChanged(int axis);
  void angleChanged(double angle);

private slots:
  void updateAngles();

private:
  Eigen::Quaterniond q_;
  Ui::EulerWidget *ui_;
};
