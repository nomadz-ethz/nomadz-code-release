/**
 * @file Sampler.h
 *
 * Declaration of Sampler classes. Samplers turn a bunch of Annotations into a bunch of Samples.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include <memory>
#include <vector>
#include <QObject>
#include <opencv2/core/core.hpp>

#include "Controller/Representations/Annotation.h"
#include "Representations/Infrastructure/Image.h"

// TODO Not sure how this class is going to play out yet... I just don't want to force everything to be a cv::Mat.
//      But this class just looks kind of useless.
class Sample {
public:
  Sample(Annotation* annotation) : annotation(annotation) {}

  // Declare virtual destructor so that Sample* can be dynamic_cast<>'d to one of its subclasses
  virtual ~Sample(){};

  virtual bool save(const std::string&) = 0;

  Annotation* annotation;
};

class MatSample : public Sample {
public:
  MatSample(cv::Mat mat, Annotation* annotation) : mat(mat), Sample(annotation) {}

  virtual bool save(const std::string&);

  static cv::Mat cropImage(const Image*, const cv::Rect&);

  static cv::Mat squeezeImage(const Image*, const cv::Point2f&, const cv::Point2f&, const cv::Point2f&, int);

  cv::Mat mat;
};

class Sampler : public QObject {
public:
  Sampler(std::vector<Annotation*> annotations) : inputs(annotations) {}
  virtual ~Sampler(){};

  virtual std::vector<std::unique_ptr<Sample>> giveResults() = 0;

public slots:
  virtual void run() = 0;

signals:
  void finished();
  void progressed(int, int);

private:
  Q_OBJECT

protected:
  std::vector<Annotation*> inputs;
  std::vector<std::unique_ptr<Sample>> results;
};

/**
 * @class SinglePatchSampler
 *
 * The SinglePatchSampler turns each Annotation into zero or one Sample by resizing it.
 */
class SinglePatchSampler : public Sampler {
public:
  SinglePatchSampler(std::vector<Annotation*> annotations) : Sampler(annotations) {}

  std::vector<std::unique_ptr<Sample>> giveResults() override;

  void setParameters();

  void run() override;

private:
  int patchSize = 24;
  float marginFactor = 0.f; // Extra margins that are 25% the radius (approx sqrt(2*pi)/2 - 1)
};

/**
 * @class RandomPatchSampler
 *
 * The RandomPatchSampler turns each Annotation into many Samples by taking many patches of fixed size around it.
 */

class RandomPatchSampler : public Sampler {
public:
  RandomPatchSampler(std::vector<Annotation*> annotations) : Sampler(annotations) {}

  std::vector<std::unique_ptr<Sample>> giveResults() override;

  void setParameters();

  void run() override;

private:
  int patchSize = 20;
  int numSamples = 50;

  MatSample* sampleCircle(Annotation* annotation) const;
};
