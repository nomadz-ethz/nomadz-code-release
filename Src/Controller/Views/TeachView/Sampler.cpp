/**
 * @file Sampler.cpp
 *
 * Implementation of Sampler classes
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include <iostream>
#include <random>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Sampler.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Tools/Geometry/Transformations.h"

bool MatSample::save(const std::string& path) {
  return cv::imwrite(path, mat);
}

cv::Mat MatSample::cropImage(const Image* image, const cv::Rect& region) {
  cv::Mat input(image->height,
                image->width,
                CV_8UC4,
                (void*)&((*image)[0][0].channels[0]),
                image->widthStep * sizeof(decltype(image->widthStep)));
  cv::Mat cropped = input(region);

  cv::Mat mat(cropped.rows, cropped.cols, CV_8UC3);
  cv::mixChannels({cropped},
                  {mat},
                  {
                    2,
                    0, // Pixel.y -> out[0]
                    3,
                    1, // Pixel.cr -> out[1]
                    1,
                    2 // Pixel.cb -> out[2]
                  });

  return mat;
}

cv::Mat MatSample::squeezeImage(
  const Image* image, const cv::Point2f& origin, const cv::Point2f& dir1, const cv::Point2f& dir2, int patchSize) {
  cv::Mat mat(patchSize, patchSize, CV_8UC3);

  const float normalize = 1.f / patchSize;

  for (int i = 0; i < patchSize; ++i) { // row (mat.y / dir2)
    uchar* matRow = mat.ptr(i);
    for (int j = 0; j < patchSize; ++j) { // col (mat.x / dir1)
      const cv::Point2f pt = origin + (dir1 * j + dir2 * i) * normalize;
      const int x = int(std::round(pt.x));
      const int y = int(std::round(pt.y));

      if (x >= 0 && x < image->width && y >= 0 && y < image->height) {
        const Image::Pixel& px = (*image)[y][x];
        matRow[3 * j + 0] = px.y;
        matRow[3 * j + 1] = px.cr;
        matRow[3 * j + 2] = px.cb;
      } else {
        matRow[3 * j + 0] = 0;
        matRow[3 * j + 1] = 128;
        matRow[3 * j + 2] = 0;
      }
    }
  }

  return mat;
}

void SinglePatchSampler::setParameters() {
  // TODO Store parameters inside class here
  return;
}

void SinglePatchSampler::run() {
  std::cout << "SinglePatchSampler::run" << std::endl;
  if (!results.empty()) {
    std::cerr << "SinglePatchSampler: cannot run: previous results not yet consumed" << std::endl;
    return;
  }

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dis(-1.f, 1.f);

  // LUT for gamma correction
  std::map<float, cv::Mat> gamma_luts;
  {
    std::vector<float> gammas = {1.f / 2.5f,
                                 1.f / 2.25f,
                                 1.f / 2.0f,
                                 1.f / 1.8f,
                                 1.f / 1.6f,
                                 1.f / 1.4f,
                                 1.f / 1.2f,
                                 1.2f,
                                 1.4f,
                                 1.6f,
                                 1.8f,
                                 2.0f,
                                 2.25f,
                                 2.5f};
    for (float gamma : gammas) {
      cv::Mat lut(1, 256, CV_8UC1);
      uchar* ptr = lut.ptr();
      for (int i = 0; i < 256; i++) {
        ptr[i] = (uchar)(int)(std::pow((double)i / 255.0, 1.0 / gamma) * 255.0);
      }
      gamma_luts.insert(std::make_pair(gamma, lut));
    }
  }

  int processed = 0;
  for (Annotation* annotation : inputs) {
    ++processed;

    Image* image = annotation->frame->get<Image>();
    if (!image) {
      continue;
    }

    if (annotation->info.type() == typeid(Annotation::Circle)) {
      const Annotation::Circle circle = boost::get<Annotation::Circle>(annotation->info);

      for (int i = 0; i < 10; ++i) {
        // Randomly disturb position by a bit just to shake things up
        const float dx = dis(gen) * circle.r / 6.f;
        const float dy = dis(gen) * circle.r / 6.f;

        // Calculate center coordinates, but make sure entire circle fits within image
        const float cr = circle.r * (1.0f + marginFactor);
        const float cx = std::max(cr, std::min(circle.x + dx, image->width - 1 - cr));
        const float cy = std::max(cr, std::min(circle.y + dy, image->height - 1 - cr));

        const int x1 = (int)std::floor(cx - cr);
        const int x2 = (int)std::floor(cx + cr);
        const int y1 = (int)std::floor(cy - cr);
        const int y2 = (int)std::floor(cy + cr);
        if (x1 < 0 || x2 >= image->width || y1 < 0 || y2 >= image->height) {
          continue;
        }

        cv::Mat cropped = MatSample::cropImage(image, cv::Rect(x1, y1, x2 - x1, y2 - y1));

        cv::Mat resized;
        cv::resize(cropped, resized, cv::Size(patchSize, patchSize), 0, 0, cv::INTER_NEAREST);

        // Random gamma disturbance
        const std::vector<float> gammas = {1.f, 1.f / 1.8f, 1.f / 1.6f, 1.f / 1.4f, 1.f / 1.2f, 1.2f, 1.4f, 1.6f, 1.8f};
        float gamma = gammas[rand() % gammas.size()];
        if (gamma != 1.f) {
          cv::cvtColor(resized, resized, cv::COLOR_YCrCb2RGB);
          LUT(resized, gamma_luts[gamma], resized);
          cv::cvtColor(resized, resized, cv::COLOR_RGB2YCrCb);
        }

        results.emplace_back(new MatSample(resized, annotation));
      }

    } else if (annotation->info.type() == typeid(Annotation::Polygon)) {
      const Annotation::Polygon line = boost::get<Annotation::Polygon>(annotation->info);

      // Can only handle lines for now (no polygons)
      if (line.points.size() != 2) {
        continue;
      }

      // Need to project on field to know size
      CameraMatrix* cameraMatrix = annotation->frame->get<CameraMatrix>();
      CameraInfo* cameraInfo = annotation->frame->get<CameraInfo>();
      if (!cameraMatrix || !cameraInfo) {
        continue;
      }

      const Vector2<> mid = (line.points[0] + line.points[1]) * 0.5f;
      Vector2<> midOnField;
      if (!Geometry::calculatePointOnField(mid, *cameraMatrix, *cameraInfo, midOnField)) {
        continue;
      }

      Vector2<int> slightlyCloserInImage;
      const float realThickness = 250.f; // mm
      midOnField.normalize(midOnField.abs() - realThickness);
      if (!Geometry::calculatePointInImage(midOnField, *cameraMatrix, *cameraInfo, slightlyCloserInImage)) {
        continue;
      }

      const Vector2<> slightlyCloser(slightlyCloserInImage.x,
                                     slightlyCloserInImage.y); // exactly slightlyCloserInImage, but with floats
      const float dir2Len = std::max(patchSize / 3.f, (mid - slightlyCloser).abs());
      Vector2<> dir1 = line.points[1] - line.points[0];
      Vector2<> dir2 = dir1.left() / dir1.abs() *
                       dir2Len; // Actually rotates right, because screen x-y rotation is not trigonometric rotation
      Vector2<> origin = line.points[0] - dir2 * 0.5f;

      const float extraMargin = 4.f; // px: extra pixels in image to take in on each side
      origin -= dir1 / dir1.abs() * extraMargin;
      origin -= dir2 / dir2.abs() * extraMargin;
      dir1.normalize(extraMargin + dir1.abs() + extraMargin);
      dir2.normalize(extraMargin + dir2.abs() + extraMargin);

      for (int i = 0; i < 5; ++i) {
        cv::Mat resized = MatSample::squeezeImage(
          image, cv::Point2f(origin.x, origin.y), cv::Point2f(dir1.x, dir1.y), cv::Point2f(dir2.x, dir2.y), patchSize);

        // Random gamma disturbance
        const std::vector<float> gammas = {1.f, 1.f / 1.8f, 1.f / 1.6f, 1.f / 1.4f, 1.f / 1.2f, 1.2f, 1.4f, 1.6f, 1.8f};
        float gamma = gammas[rand() % gammas.size()];
        if (gamma != 1.f) {
          cv::cvtColor(resized, resized, cv::COLOR_YCrCb2RGB);
          LUT(resized, gamma_luts[gamma], resized);
          cv::cvtColor(resized, resized, cv::COLOR_RGB2YCrCb);
        }

        results.emplace_back(new MatSample(resized, annotation));
      }
    }

    emit progressed(processed, inputs.size());
  }

  emit finished();
}

std::vector<std::unique_ptr<Sample>> SinglePatchSampler::giveResults() {
  std::cout << "SinglePatchSampler::giveResults " << results.size() << std::endl;

  // Make new empty vector
  std::vector<std::unique_ptr<Sample>> results2;

  // Put this->results into the empty vector results2, while emptying this->results
  std::swap(results, results2);

  // Return the actual results that are now in the no-longer-empty vector
  return results2;
}

void RandomPatchSampler::run() {
  std::cout << "RandomPatchSampler::run" << std::endl;
  if (!results.empty()) {
    std::cerr << "RandomPatchSampler: cannot run: previous results not yet consumed" << std::endl;
    return;
  }

  int processed = 0;
  for (Annotation* annotation : inputs) {
    ++processed;

    if (annotation->info.type() != typeid(Annotation::Circle)) {
      continue;
    }

    for (int i = 0; i < numSamples; ++i) {
      results.emplace_back(sampleCircle(annotation));
    }

    emit progressed(processed, inputs.size());
  }

  emit finished();
}

// Undefined behavior if annotation->info is not a Circle
MatSample* RandomPatchSampler::sampleCircle(Annotation* annotation) const {
  Annotation::Circle circle = boost::get<Annotation::Circle>(annotation->info);

  Image* image = annotation->frame->get<Image>();
  if (!image) {
    return nullptr;
  }

  int iterations = 50;
  while (iterations-- > 0) {
    // Rejection sampling in a circle
    // TODO Replace rand() with something from <random> -- rand() is not thread-safe
    const int dx = rand() % (int)(2 * circle.r) - (int)circle.r;
    const int dy = rand() % (int)(2 * circle.r) - (int)circle.r;
    if (dx * dy + dy * dy > (int)(circle.r * circle.r)) {
      continue;
    }

    // Cut out a little square (patchSize x patchSize) & make sure it's still in the image
    const int x1 = (int)std::floor(circle.x + dx - patchSize / 2);
    const int x2 = (int)std::floor(circle.x + dx + patchSize / 2);
    const int y1 = (int)std::floor(circle.y + dy - patchSize / 2);
    const int y2 = (int)std::floor(circle.y + dy + patchSize / 2);
    if (x1 < 0 || x2 >= image->width || y1 < 0 || y2 >= image->height) {
      continue;
    }

    cv::Mat patch = MatSample::cropImage(image, cv::Rect(x1, y1, x2 - x1, y2 - y1));
    return new MatSample(patch, annotation);
  }

  // Something didn't work out, apparently :(
  std::cerr << "RandomPatchSampler::sampleCircle timed out trying to sample circle (x: " << circle.x << ", y: " << circle.y
            << ", r: " << circle.r << ") in Image (w: " << image->width << ", h: " << image->height << ")" << std::endl;
  return nullptr;
}

std::vector<std::unique_ptr<Sample>> RandomPatchSampler::giveResults() {
  std::cout << "RandomPatchSampler::giveResults" << std::endl;

  // Make new empty vector
  std::vector<std::unique_ptr<Sample>> results2;

  // Put this->results into the empty vector results2, while emptying this->results
  std::swap(results, results2);

  // Return the actual results that are now in the no-longer-empty vector
  return results2;
}
