/**
 * @file SampleView.cpp
 *
 * Implementation of classes SampleView & SampleWidget
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <QFileDialog>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QSlider>
#include <QString>
#include <QVBoxLayout>
#include <opencv2/imgproc/imgproc.hpp>
#include "SampleView.h"
#include "TeachView.h"
#include "Core/System/File.h"
#include "Core/System/Thread.h"
#include "Controller/ConsoleRoboCupCtrl.h"
#include "Controller/Views/TeachView/PixmapTile.h"
#include "Controller/Views/TeachView/Sampler.h"

SampleView::SampleView(const QString& fullName, ConsoleRoboCupCtrl& console, TeachView& teachView)
    : fullName(fullName), icon(":/Icons/tag_green.png"), console(console), teachView(teachView) {}

SimRobot::Widget* SampleView::createWidget() {
  return new SampleWidget(*this);
}

SampleWidget::SampleWidget(SampleView& sampleView)
    : sampleView(sampleView), teachView(sampleView.teachView), lesson(sampleView.teachView.lesson),
      session(sampleView.teachView.session) {
  setFocusPolicy(Qt::StrongFocus);

  QVBoxLayout* layout = new QVBoxLayout(this);

  QSlider* columnsSlider;
  QHBoxLayout* topLayout = new QHBoxLayout();
  {
    columnsSlider = new QSlider(Qt::Horizontal, this);
    {
      columnsSlider->setInvertedAppearance(true);
      columnsSlider->setMaximum(25);
      columnsSlider->setMinimum(1);
      columnsSlider->setPageStep(1);
      columnsSlider->setSingleStep(1);
      columnsSlider->setValue(25);
      topLayout->addWidget(columnsSlider);
    }

    sampleCountLabel = new QLabel(tr("0 samples"), this);
    {
      sampleCountLabel->setAlignment(Qt::AlignRight);
      topLayout->addWidget(sampleCountLabel);
    }

    layout->addLayout(topLayout);
  }

  sampleScene = new QGraphicsScene(this);
  sampleGrid = new TileGridView(sampleScene, this);
  {
    sampleGrid->setAspectRatio(1.0);
    sampleGrid->setColumns(columnsSlider->value());
    sampleGrid->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    sampleGrid->setSpacing(5.0);
    sampleGrid->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
    layout->addWidget(sampleGrid);

    connect(columnsSlider, SIGNAL(valueChanged(int)), sampleGrid, SLOT(setColumns(int)));
  }

  connect(&session, SIGNAL(selectedAnnotationsChanged(const std::unordered_set<Annotation*>)), this, SLOT(startSampler()));

  this->setLayout(layout);
}

SampleWidget::~SampleWidget() {
  samplerThread.quit();
  samplerThread.wait();
}

QWidget* SampleWidget::getWidget() {
  return this;
}

void SampleWidget::contextMenuEvent(QContextMenuEvent* event) {
  QMenu menu;

  event->accept();

  QAction* action = menu.addAction(tr("Save %1 samples (YCrCb) to folder...").arg(samples.size()));
  if (samples.empty()) {
    action->setEnabled(false);
  }
  connect(action, SIGNAL(triggered()), this, SLOT(saveToFolder()));

  menu.exec(event->globalPos());
}

void SampleWidget::startSampler() {
  if (samplerThread.isRunning()) {
    // Skip if thread already running; yes we'll miss out on the most recent selections,
    // but most of the time the sampler shouldn't already be running
    return;
  }

  std::vector<Annotation*> annotations(session.selectedAnnotations.begin(), session.selectedAnnotations.end());

  // TODO Make sampler user-selectable, and save user choice in Lesson or TeachSession
  Sampler* sampler = new SinglePatchSampler(annotations);
  // Sampler* sampler = new RandomPatchSampler(annotations);

  sampler->moveToThread(&samplerThread);

  // These all get disconnected when sampler is destroyed
  connect(&samplerThread, SIGNAL(finished()), sampler, SLOT(deleteLater()));
  connect(&samplerThread, SIGNAL(started()), sampler, SLOT(run()));
  connect(sampler, SIGNAL(finished()), this, SLOT(showSamplerResults()));

  samplerThread.start();

  std::cout << "launched sampler thread for " << annotations.size() << " annotations" << std::endl;
}

void SampleWidget::showSamplerResults() {
  Sampler* sampler = qobject_cast<Sampler*>(sender());
  if (!sampler) {
    std::cerr << "SampleWidget::showSamplerResults: sampler is null" << std::endl;
    return;
  }

  samples = sampler->giveResults();
  std::cout << "showSamplerResults: " << samples.size() << " samples" << std::endl;

  samplerThread.quit();

  // Update sample drawings view
  sampleCountLabel->setText(tr(samples.size() == 1 ? "%1 sample" : "%1 samples").arg(samples.size()));
  sampleScene->clear();
  for (const auto& i : samples) {
    MatSample* sample = dynamic_cast<MatSample*>(i.get());
    cv::Mat rgbMat;
    cv::cvtColor(sample->mat, rgbMat, cv::COLOR_YCrCb2RGB);
    PixmapTile* tile = PixmapTile::fromCVMat(rgbMat);
    sampleScene->addItem(tile);
  }
}

// Generate a random name
std::string SampleWidget::generateName() {
  const std::string alphabet = "-0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ_abcdefghijklmnopqrstuvwxyz";
  const int L = alphabet.size();

  std::string name;

  // 8 characters with a 64-letter alphabet guarantees a 0.2% chance of collision at 10e6 calls
  for (int i = 0; i < 8; ++i) {
    name += alphabet[rand() % L];
  }

  return name;
}

void SampleWidget::saveToFolder() {
  QString qdir = QFileDialog::getExistingDirectory(
    this, tr("Save %1 samples to folder").arg(samples.size()), QString(File::getBHDir()), QFileDialog::ShowDirsOnly);

  std::string dir = qdir.toUtf8().constData();

  auto startTime = std::chrono::high_resolution_clock::now();

  int i = 0;
  const int N = samples.size();

  for (const auto& sample : samples) {
    std::string path = dir + "/" + generateName() + ".bmp";

    try {
      sample->save(path);
    } catch (std::runtime_error& e) {
      std::cerr << "SampleWidget::saveToFolder: Could not save sample to " << path << ": " << e.what() << std::endl;
    }

    ++i;
    if (i % 10000 == 0) {
      const float elapsed = (std::chrono::high_resolution_clock::now() - startTime).count() / 1e9f; // seconds
      std::cout << i << " / " << N << " (" << int(elapsed) << "s elapsed, " << int(elapsed * (N - i) / i) << "s left)"
                << std::endl;
    }
  }

  const float elapsed = (std::chrono::high_resolution_clock::now() - startTime).count() / 1e9f; // seconds
  std::cout << "Done in " << elapsed << "s" << std::endl;
}
