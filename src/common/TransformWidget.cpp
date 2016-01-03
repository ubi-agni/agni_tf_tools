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

#include "TransformWidget.h"
#include "EulerWidget.h"

#include "ui_transform.h"

#include <QMetaType>

TransformWidget::TransformWidget(QWidget *parent) :
  QWidget(parent), ui_(new Ui::TransformWidget)
{
  qRegisterMetaType<Eigen::Quaterniond>("Eigen::Vector3d");
  qRegisterMetaType<Eigen::Quaterniond>("Eigen::Quaterniond");
  pos_.setZero();

  ui_->setupUi(this);

  connect(ui_->pos_x, SIGNAL(valueChanged(double)), this, SLOT(changePos(double)));
  connect(ui_->pos_y, SIGNAL(valueChanged(double)), this, SLOT(changePos(double)));
  connect(ui_->pos_z, SIGNAL(valueChanged(double)), this, SLOT(changePos(double)));
  connect(ui_->euler_widget_, SIGNAL(valueChanged(Eigen::Quaterniond)),
          this, SIGNAL(quaternionChanged(Eigen::Quaterniond)));
}

const Eigen::Vector3d &TransformWidget::position() const
{
  return pos_;
}

const Eigen::Quaterniond &TransformWidget::quaternion() const
{
  return ui_->euler_widget_->value();
}

void TransformWidget::setPosition(const Eigen::Vector3d &p)
{
  if (pos_.isApprox(p)) return;
  pos_ = p;

  // do not trigger posChanged() signals
  ui_->pos_x->blockSignals(true);
  ui_->pos_y->blockSignals(true);
  ui_->pos_z->blockSignals(true);

  ui_->pos_x->setValue(p.x());
  ui_->pos_y->setValue(p.y());
  ui_->pos_z->setValue(p.z());

  ui_->pos_x->blockSignals(false);
  ui_->pos_y->blockSignals(false);
  ui_->pos_z->blockSignals(false);

  emit positionChanged(pos_);
}

void TransformWidget::setQuaternion(const Eigen::Quaterniond &q)
{
  ui_->euler_widget_->setValue(q);
}

void TransformWidget::changePos(double value)
{
  QDoubleSpinBox *s = qobject_cast<QDoubleSpinBox*>(sender());
  if (s == ui_->pos_x) changePos(0, value);
  if (s == ui_->pos_y) changePos(1, value);
  if (s == ui_->pos_z) changePos(2, value);
}

void TransformWidget::changePos(unsigned int i, double value)
{
  if (Eigen::internal::isApprox(pos_[i], value)) return;
  pos_[i] = value;
  emit positionChanged(pos_);
}
