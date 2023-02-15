/**
 * @file Image.cpp
 *
 * Implementation of class Image.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#include <cstring>

#include "Image.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Core/System/BHAssert.h"

#include <opencv2/opencv.hpp>
using namespace cv;

Image::Image(bool initialize, int width, int height)
    : timeStamp(0), isReference(false), width(width), height(height), widthStep(width * 2) {
  // allocate full size image and keep it that way indepentent of resolution
  image = new Pixel[maxResolutionWidth * maxResolutionHeight * 2];
  if (initialize) {
    for (int y = 0; y < height; ++y) {
      for (Pixel *p = (*this)[y], *pEnd = p + width; p < pEnd; ++p) {
        p->color = 0x80008000;
      }
    }
  }
}

Image::Image(const Image& other) : isReference(true) {
  *this = other;
}

Image::~Image() {
  if (!isReference) {
    delete[] image;
  }
}

Image& Image::operator=(const Image& other) {
  height = other.height;
  width = other.width;
  widthStep = 2 * width;
  timeStamp = other.timeStamp;
  if (isReference) {
    // allocate full size image and keep it that way indepentent of resolution
    image = new Pixel[maxResolutionHeight * maxResolutionWidth * 2];
    isReference = false;
  }
  for (int y = 0; y < height; ++y) {
    memcpy((*this)[y], other[y], width * sizeof(Image::Pixel));
  }
  return *this;
}

void Image::setImage(const unsigned char* buffer) {
  if (!isReference) {
    delete[] image;
    isReference = true;
  }
  image = (Pixel*)buffer;
}

void Image::convertFromYCbCrToRGB(const Image& ycbcrImage) {
  height = ycbcrImage.height;
  width = ycbcrImage.width;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      ColorModelConversions::fromYCbCrToRGB(
        ycbcrImage[y][x].y, ycbcrImage[y][x].cb, ycbcrImage[y][x].cr, (*this)[y][x].r, (*this)[y][x].g, (*this)[y][x].b);
    }
  }
}

void Image::convertFromRGBToYCbCr(const Image& rgbImage) {
  height = rgbImage.height;
  width = rgbImage.width;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      ColorModelConversions::fromRGBToYCbCr(
        rgbImage[y][x].r, rgbImage[y][x].g, rgbImage[y][x].b, (*this)[y][x].y, (*this)[y][x].cb, (*this)[y][x].cr);
    }
  }
}

void Image::convertFromYCbCrToHSI(const Image& ycbcrImage) {
  height = ycbcrImage.height;
  width = ycbcrImage.width;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      ColorModelConversions::fromYCbCrToHSI(
        ycbcrImage[y][x].y, ycbcrImage[y][x].cb, ycbcrImage[y][x].cr, (*this)[y][x].h, (*this)[y][x].s, (*this)[y][x].i);
    }
  }
}

void Image::convertFromHSIToYCbCr(const Image& hsiImage) {
  height = hsiImage.height;
  width = hsiImage.width;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      ColorModelConversions::fromHSIToYCbCr(
        hsiImage[y][x].h, hsiImage[y][x].s, hsiImage[y][x].i, (*this)[y][x].y, (*this)[y][x].cb, (*this)[y][x].cr);
    }
  }
}

cv::Mat Image::convertToCVMat() const {
  Mat ret(height, width, CV_8UC3);
  for (int i = 0; i < height; i++) {
    uchar* row_ptr = ret.ptr(i);
    const Pixel* img_row = (*this)[i];
    for (int j = 0; j < width; j++) {
      const Image::Pixel& pix = img_row[j];
      row_ptr[3 * j + 0] = pix.y;
      row_ptr[3 * j + 1] = pix.cr;
      row_ptr[3 * j + 2] = pix.cb;
    }
  }
  return ret;
}

void Image::importFromCVMat(const cv::Mat& mat) {
  setResolution(mat.cols, mat.rows);
  setImage((unsigned char*)new Pixel[width * height * 2]);
  isReference = false;

  for (int i = 0; i < height; ++i) {
    Pixel* img_row = (*this)[i];
    const uchar* row_ptr = mat.ptr(i);
    for (int j = 0; j < width; ++j) {
      Image::Pixel& pix = img_row[j];
      pix.y = row_ptr[3 * j + 0];
      pix.cr = row_ptr[3 * j + 1];
      pix.cb = row_ptr[3 * j + 2];
    }
  }
}

cv::Mat Image::convertToCVMat_old() const {
  Mat ret(height, width, CV_8UC3);
  for (int i = 0; i < height; i++) {
    uchar* row_ptr = ret.ptr(i);
    const Pixel* img_row = (*this)[i];
    for (int j = 0; j < width; j++) {
      const Image::Pixel& pix = img_row[j];
      row_ptr[3 * j + 0] = pix.cb;
      row_ptr[3 * j + 1] = pix.y;
      row_ptr[3 * j + 2] = pix.cr;
    }
  }
  return ret;
}

cv::Mat Image::convertToCVMatYChan() const {
  Mat ret(height, width, CV_8UC1);
  for (int row = 0; row < height; ++row) {
    uchar* p = ret.ptr(row);
    for (int col = 0; col < width; ++col) {
      *(p++) = (*this)[row][col].y;
    }
  }
  return ret;
}

void Image::extractCVMatPatch(int x, int y, cv::Mat& patch) const {
  x = std::min(x, (width - patch.cols));
  y = std::min(y, (height - patch.rows));
  for (int i = 0; i < patch.rows; i++) {
    uchar* row_ptr = patch.ptr(i);
    const Pixel* img_row = (*this)[i + y];
    for (int j = 0; j < patch.cols; j++) {
      const Image::Pixel& pix = img_row[j + x];
      row_ptr[3 * j + 0] = pix.y;
      row_ptr[3 * j + 1] = pix.cr;
      row_ptr[3 * j + 2] = pix.cb;
    }
  }
}

void Image::extractCVMatPatchFromCenterPoint(Vector2<int>& center, cv::Mat& patch) const {
  int xS = std::max(0, std::min(center.x - patch.cols / 2, width - patch.cols));
  int yS = std::max(0, std::min(center.y - patch.rows / 2, height - patch.rows));

  center.x = xS + patch.cols / 2;
  center.y = yS + patch.rows / 2;

  for (int i = 0; i < patch.rows; i++) {
    uchar* row_ptr = patch.ptr(i);
    const Pixel* img_row = (*this)[i + yS];
    for (int j = 0; j < patch.cols; j++) {
      const Image::Pixel& pix = img_row[j + xS];
      row_ptr[3 * j + 0] = pix.y;
      row_ptr[3 * j + 1] = pix.cr;
      row_ptr[3 * j + 2] = pix.cb;
    }
  }
}

void Image::serialize(In* in, Out* out) {
  STREAM_REGISTER_BEGIN;
  STREAM(width);
  STREAM(height);
  STREAM(timeStamp);

  if (out) {
    for (int y = 0; y < height; ++y) {
      out->write((*this)[y], width * sizeof(Pixel));
    }
  } else {
    widthStep = width * 2;
    for (int y = 0; y < height; ++y) {
      in->read((*this)[y], width * sizeof(Pixel));
    }
  }

  STREAM_REGISTER_FINISH;
}

float Image::getColorDistance(const Image::Pixel& a, const Image::Pixel& b) {
  int dy = int(a.y) - b.y;
  int dcb = int(a.cb) - b.cb;
  int dcr = int(a.cr) - b.cr;
  dy *= dy;
  dcb *= dcb;
  dcr *= dcr;
  return std::sqrt(float(dy + dcb + dcr));
}
