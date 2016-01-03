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

#include "FramesWidget.h"
#include "ui_frames.h"

FramesWidget::FramesWidget(const QString &parent_frame, const QString &child_frame, QWidget *parent) :
   QWidget(parent), ui_(new Ui::FramesWidget)
{
  ui_->setupUi(this);
  setParentFrame(parent_frame);
  setChildFrame(child_frame);

  connect(ui_->parent, SIGNAL(editingFinished()),
          this, SLOT(parentEdited()));
  connect(ui_->child, SIGNAL(editingFinished()),
          this, SLOT(childEdited()));
}

QString FramesWidget::parentFrame() const
{
  return ui_->parent->text();
}

QString FramesWidget::childFrame() const
{
  return ui_->child->text();
}

void FramesWidget::setParentFrame(const QString &frame)
{
  if (ui_->parent->text() == frame) return;
  ui_->parent->setText(frame);
  emit parentFrameChanged(frame);
}

void FramesWidget::setChildFrame(const QString &frame)
{
  if (ui_->child->text() == frame) return;
  ui_->child->setText(frame);
  emit childFrameChanged(frame);
}

void FramesWidget::parentEdited()
{
  emit parentFrameChanged(ui_->parent->text());
}

void FramesWidget::childEdited()
{
  emit childFrameChanged(ui_->child->text());
}
