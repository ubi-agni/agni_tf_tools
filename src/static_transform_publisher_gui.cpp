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

#include <ros/ros.h>
#include <QApplication>
#include <QVBoxLayout>

#include "FramesWidget.h"
#include "TransformWidget.h"
#include "TransformBroadcaster.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "static_transform_publisher_gui",
            ros::init_options::AnonymousName ||
            ros::init_options::NoSigintHandler);
  QApplication app(argc, argv);

  QWidget *main = new QWidget();
  QVBoxLayout *l = new QVBoxLayout();

  FramesWidget *frames = new FramesWidget();
  TransformWidget *tf_widget = new TransformWidget();
  l->addWidget(frames);
  l->addWidget(tf_widget);
  main->setLayout(l);

  TransformBroadcaster *tf_pub = new TransformBroadcaster(frames->parentFrame(),
                                                          frames->childFrame(), main);
  QObject::connect(frames, SIGNAL(parentFrameChanged(QString)),
                   tf_pub, SLOT(setParentFrame(QString)));
  QObject::connect(frames, SIGNAL(childFrameChanged(QString)),
                   tf_pub, SLOT(setChildFrame(QString)));

  QObject::connect(tf_widget, SIGNAL(positionChanged(Eigen::Vector3d)),
                   tf_pub, SLOT(setPosition(Eigen::Vector3d)));
  QObject::connect(tf_widget, SIGNAL(quaternionChanged(Eigen::Quaterniond)),
                   tf_pub, SLOT(setQuaternion(Eigen::Quaterniond)));

  main->setWindowTitle("static transform publisher");
  main->show();
  int ret = app.exec();
  delete main;
  return ret;
}
