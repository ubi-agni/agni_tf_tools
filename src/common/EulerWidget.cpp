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

#include "EulerWidget.h"
#include "ui_euler.h"

#include <angles/angles.h>
#include <QStandardItemModel>
#include <iostream>

// ensure different axes for consecutive operations
static void disableAxis(QComboBox *w, unsigned int axis) {
  const QStandardItemModel* model = qobject_cast<const QStandardItemModel*>(w->model());
  for (unsigned int i=0; i < 3; ++i) {
    QStandardItem* item = model->item(i);
    if (i == axis) {
      item->setFlags(item->flags() & ~Qt::ItemIsEnabled);
      if (w->currentIndex() == axis) w->setCurrentIndex((axis+1) % 3);
    } else {
      item->setFlags(item->flags() | Qt::ItemIsEnabled);
    }
  }
}

EulerWidget::EulerWidget(QWidget *parent) :
  QWidget(parent), ui_(new Ui::EulerWidget)
{
  qRegisterMetaType<Eigen::Quaterniond>("Eigen::Quaterniond");

  ui_->setupUi(this);
  ui_->a1->setCurrentIndex(0);
  ui_->a2->setCurrentIndex(1); disableAxis(ui_->a2, 0);
  ui_->a3->setCurrentIndex(2); disableAxis(ui_->a3, 1);

  q_ = Eigen::Quaterniond::Identity();
  updateAngles();

  // react to axis changes
  connect(ui_->a1, SIGNAL(currentIndexChanged(int)),
          this, SLOT(axisChanged(int)));
  connect(ui_->a2, SIGNAL(currentIndexChanged(int)),
          this, SLOT(axisChanged(int)));
  connect(ui_->a3, SIGNAL(currentIndexChanged(int)),
          this, SLOT(axisChanged(int)));

  // react to angle changes
  connect(ui_->e1, SIGNAL(valueChanged(double)),
          this, SLOT(angleChanged(double)));
  connect(ui_->e2, SIGNAL(valueChanged(double)),
          this, SLOT(angleChanged(double)));
  connect(ui_->e3, SIGNAL(valueChanged(double)),
          this, SLOT(angleChanged(double)));
}

void EulerWidget::getGuiAxes(uint a[]) const {
  a[0] = ui_->a1->currentIndex();
  a[1] = ui_->a2->currentIndex();
  a[2] = ui_->a3->currentIndex();
}

void EulerWidget::getGuiAngles(double e[3]) const {
  e[0] = angles::from_degrees(ui_->e1->value());
  e[1] = angles::from_degrees(ui_->e2->value());
  e[2] = angles::from_degrees(ui_->e3->value());
}


void EulerWidget::axisChanged(int axis) {
  bool bFirstCall = !this->signalsBlocked();
  this->blockSignals(true);

  // ensure different axes for consecutive operations
  QComboBox* origin = dynamic_cast<QComboBox*>(sender());
  if (origin == ui_->a1) disableAxis(ui_->a2, axis);
  if (origin == ui_->a2) disableAxis(ui_->a3, axis);

  if (bFirstCall) {
    updateAngles();
    this->blockSignals(false);

    emit axesChanged(ui_->a1->currentIndex(),
                     ui_->a2->currentIndex(),
                     ui_->a3->currentIndex());
  }
}

void EulerWidget::angleChanged(double angle) {
  double e[3]; getGuiAngles(e);
  setEulerAngles(e[0], e[1], e[2], false);
}

void EulerWidget::setEulerAngles(double e1, double e2, double e3, bool normalize) {
  uint a[3]; getGuiAxes(a);
  Eigen::Quaterniond q =
      Eigen::AngleAxisd(e1, Eigen::Vector3d::Unit(a[0])) *
      Eigen::AngleAxisd(e2, Eigen::Vector3d::Unit(a[1])) *
      Eigen::AngleAxisd(e3, Eigen::Vector3d::Unit(a[2]));
  if (normalize)
    setValue(q);
  else {
    // do not trigger angleChanged() again
    ui_->e1->blockSignals(true);
    ui_->e2->blockSignals(true);
    ui_->e3->blockSignals(true);

    ui_->e1->setValue(angles::to_degrees(e1));
    ui_->e2->setValue(angles::to_degrees(e2));
    ui_->e3->setValue(angles::to_degrees(e3));

    ui_->e1->blockSignals(false);
    ui_->e2->blockSignals(false);
    ui_->e3->blockSignals(false);

    if (q_.isApprox(q)) return;
    q_ = q;
    emit valueChanged(q);
  }
}

void EulerWidget::setEulerAxes(uint a1, uint a2, uint a3)
{
  if (a1 > 2 || a2 > 2 || a3 > 2) return;
  if (a1 == ui_->a1->currentIndex() &&
      a2 == ui_->a2->currentIndex() &&
      a3 == ui_->a3->currentIndex()) return;

  this->blockSignals(true);
  ui_->a3->setCurrentIndex(a3);
  ui_->a2->setCurrentIndex(a2);
  ui_->a1->setCurrentIndex(a1);
  this->blockSignals(false);
  updateAngles();

  emit axesChanged(a1, a2, a3);
}


void EulerWidget::setValue(const Eigen::Quaterniond &q) {
  if (q_.isApprox(q)) return;
  q_ = q;
  updateAngles();
  emit valueChanged(q);
}

const Eigen::Quaterniond& EulerWidget::value() const {
  return q_;
}


void EulerWidget::updateAngles() {
  // ensure different axes for consecutive operations
  uint a[3]; getGuiAxes(a);
  Eigen::Vector3d e = q_.matrix().eulerAngles(a[0], a[1], a[2]);
  setEulerAngles(e[0], e[1], e[2], false);
}
