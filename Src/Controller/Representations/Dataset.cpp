/**
 * @file Dataset.cpp
 *
 * Implementation of class Dataset
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <libgen.h>
#include <iostream>
#include <sstream>
#include <QDir>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sys/stat.h>

#include "Dataset.h"
#include "Controller/LogPlayer.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/BodyContour.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Core/Streams/OutStreams.h"

#if CV_MAJOR_VERSION >= 4
#ifndef CV_LOAD_IMAGE_COLOR
#define CV_LOAD_IMAGE_COLOR cv::IMREAD_COLOR
#endif
#endif

static std::string baseName(const std::string& path) {
  std::vector<char> pathCopy(path.c_str(), path.c_str() + path.size() + 1);
  return basename(pathCopy.data());
}

std::unique_ptr<Dataset> Dataset::load(const StreamableDataset& streamableDataset,
                                       const std::unordered_map<int, const StreamableDataFrame*>& frameIdToStreamable,
                                       std::unordered_map<int, DataFrame*>& idToFrame) {

  std::set<int> selectedTimes;
  std::unordered_map<int, int> timeToFrameId;
  for (int frameId : streamableDataset.frameIds) {
    int frameTime = frameIdToStreamable.at(frameId)->time;
    selectedTimes.insert(frameTime);
    timeToFrameId.insert(std::make_pair(frameTime, frameId));
  }

  std::unique_ptr<Dataset> dataset;
  switch (streamableDataset.source) {

  case Dataset::Log: {
    dataset = Dataset::loadLog(streamableDataset.sourcePath, selectedTimes);
    if (!dataset) {
      return nullptr;
    }

    break;
  }

  case Dataset::Folder: {
    dataset = Dataset::loadFolder(streamableDataset.sourcePath, selectedTimes);
    if (!dataset) {
      return nullptr;
    }

    break;
  }

  default: {
    return nullptr;
    break;
  }
  }

  // Fill in idToFrame
  for (const auto& i : dataset->frames) {
    int frameId = timeToFrameId.at(i.first);
    DataFrame* frame = i.second.get();
    idToFrame.insert(std::make_pair(frameId, frame));
  }

  dataset->name = streamableDataset.name;
  return dataset;
}

void Dataset::saveToFolder(const std::string& folderPath) const {
  // Create the folder
  struct stat s;
  const bool dirExists = (stat(folderPath.c_str(), &s) == 0 && S_ISDIR(s.st_mode));
  if (!dirExists) {
    int status = mkdir(folderPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (status != 0) {
      throw std::runtime_error(std::string("Dataset::saveFolder: could not mkdir ") + folderPath);
    }
  }

  // Save certain representations in each frame into a file
  for (const auto& i : frames) {
    DataFrame* frame = i.second.get();
    if (!frame) {
      continue;
    }

    const std::string pathPrefix = folderPath + "/" + std::to_string(frame->time) + ".";

    if (frame->has<Image>()) {
      // Export Images as .jpegs
      const std::string path = pathPrefix + "idImage.bmp";
      cv::Mat mat = frame->get<Image>()->convertToCVMat();
      cv::imwrite(path, mat);
    }

    if (frame->has<CameraInfo>()) {
      const std::string path = pathPrefix + "idCameraInfo.txt";
      OutMapFile stream(path);
      stream << *(frame->get<CameraInfo>());
    }

    if (frame->has<CameraMatrix>()) {
      const std::string path = pathPrefix + "idCameraMatrix.txt";
      OutMapFile stream(path);
      stream << *(frame->get<CameraMatrix>());
    }
  }
}

std::unique_ptr<Dataset> Dataset::loadLog(const std::string& fullPath, const std::set<int>& selectedTimes) {
  if (fullPath.empty()) {
    return nullptr;
  }

  std::unique_ptr<Dataset> datasetPtr(new Dataset());
  Dataset& dataset = *datasetPtr;

  dataset.name = baseName(fullPath);
  dataset.source = Dataset::Log;
  dataset.sourcePath = fullPath;

  LogHandler logHandler(dataset, selectedTimes);

  MessageQueue logQueue;
  LogPlayer logPlayer(logQueue);
  logPlayer.setSize(std::numeric_limits<unsigned>::max()); // max. 4 GB
  logPlayer.init();
  logPlayer.setLoop(false);

  if (!logPlayer.open(fullPath.c_str())) {
    return nullptr;
  }

  logPlayer.play();

  // Skip frames if too many
  const int maxFrames = 4000;
  int interval = (int)std::ceil(logPlayer.numberOfFrames / (float)maxFrames);

  while (logPlayer.replay()) {
    if (logPlayer.currentFrameNumber % interval == 0) {
      std::cout << "-- " << ((logPlayer.currentFrameNumber + logPlayer.numberOfFrames) % logPlayer.numberOfFrames) << " / "
                << logPlayer.numberOfFrames << " --" << std::endl;
      logQueue.handleAllMessages(logHandler);
    }
    logQueue.clear();
  }

  return datasetPtr;
}
std::unique_ptr<Dataset> Dataset::loadFolder(const std::string& folderPath, const std::set<int>& selectedTimes) {
  if (folderPath.empty()) {
    return nullptr;
  }

  std::unique_ptr<Dataset> datasetPtr(new Dataset());
  Dataset& dataset = *datasetPtr;

  dataset.name = baseName(folderPath);
  dataset.source = Dataset::Folder;
  dataset.sourcePath = folderPath;

  // Get vector of files inside dir
  std::vector<std::string> paths;
  {
    QDir qDir(folderPath.c_str());
    QStringList filters;
    filters << "*.bin"
            << "*.bmp"
            << "*.txt";

    for (const QString& path : qDir.entryList(filters, QDir::Files)) {
      paths.push_back(folderPath + "/" + path.toUtf8().constData());
    }
  }

  if (paths.empty()) {
    return nullptr;
  }

  for (const auto& path : paths) {
    std::string fileName = baseName(path);
    std::stringstream ss(fileName);
    std::string part;

    // Parse the file name
    int time;
    if (std::getline(ss, part, '.')) {
      try {
        time = std::stoi(part);
      } catch (std::invalid_argument& e) {
        std::cerr << "Skipping " << fileName << ": could not convert " << part << " to an integer" << std::endl;
        continue;
      }
    } else {
      std::cerr << "Skipping " << fileName << ": could not find time" << std::endl;
      continue;
    }

    std::string messageID;
    if (!std::getline(ss, messageID, '.')) {
      std::cerr << "Skipping " << fileName << ": could not find messageID" << std::endl;
      continue;
    }

    std::string extension;
    if (!std::getline(ss, extension, '.')) {
      std::cerr << "Skipping " << fileName << ": could not find extension" << std::endl;
      continue;
    }

    if (std::getline(ss, part, '.')) {
      // Can't have more than 3 "parts"
      std::cerr << "Skipping " << fileName << ": too many parts to file name" << std::endl;
      continue;
    }

    // Find or create the frame
    DataFrame* frame;
    {
      auto it = dataset.frames.find(time);
      if (it == dataset.frames.end()) {
        dataset.frames.insert(std::make_pair(time, std::unique_ptr<DataFrame>(new DataFrame(time))));
        frame = dataset.frames.at(time).get();
      } else {
        frame = it->second.get();
      }
    }

    // Read the file
    if (extension == "txt") {
      InMapFile stream(path);
      if (messageID == "idCameraMatrix") {
        CameraMatrix cm;
        stream >> cm;
        frame->store<CameraMatrix>(std::move(cm));
      } else if (messageID == "idCameraInfo") {
        CameraInfo ci;
        stream >> ci;
        frame->store<CameraInfo>(std::move(ci));
      }

    } else if (extension == "bmp") {
      if (messageID == "idImage") {
        Image img;
        img.timeStamp = time;

        cv::Mat mat = cv::imread(path, CV_LOAD_IMAGE_COLOR);
        img.importFromCVMat(mat);

        frame->store<Image>(std::move(img));
      }
    }
  }

  return datasetPtr;
}

bool Dataset::LogHandler::handleMessage(InMessage& message) {
  switch (message.getMessageID()) {

  case idProcessBegin: {
    char c;
    message.bin >> c;
    std::cout << "idProcessBegin " << c << std::endl;
    currentFrame.reset(new DataFrame());
    return true;
  }
  case idProcessFinished: {
    char c;
    message.bin >> c;
    std::cout << "idProcessFinished " << c << std::endl;
    if (currentFrame && currentFrame->time > 0) {
      int currentFrameTime = currentFrame->time;
      if (selectedTimes.empty() || selectedTimes.find(currentFrameTime) != selectedTimes.end()) {
        const bool result = dataset.frames.insert(std::make_pair(currentFrameTime, std::move(currentFrame))).second;
        if (!result) {
          std::cerr << "Dataset::LogHandler: ignoring frame with duplicate time = " << currentFrameTime << std::endl;
        }
      } else {
        std::cerr << "Dataset::LogHandler: ignoring frame with time = " << currentFrame->time
                  << " because it was not selected" << std::endl;
      }
    } else {
      std::cerr << "Dataset::LogHandler: ignoring log frame with no time" << std::endl;
    }
    currentFrame.reset(new DataFrame());
    return true;
  }

  case idImage: {
    Image img(false);
    message.bin >> img;
    std::cout << "idImage timeStamp: " << img.timeStamp << ", width: " << img.width << ", height: " << img.height
              << ", widthStep: " << img.widthStep << std::endl;
    currentFrame->time = img.timeStamp;
    currentFrame->store(std::move(img));
    return true;
  }
  case idJPEGImage: {
    JPEGImage jpimg;
    message.bin >> jpimg;
    std::cout << "idJPEGImage timeStamp: " << jpimg.timeStamp << ", width: " << jpimg.width << ", height: " << jpimg.height
              << ", widthStep: " << jpimg.widthStep << std::endl;
    Image img(false);
    jpimg.toImage(img);
    currentFrame->time = img.timeStamp;
    currentFrame->store(std::move(img));
    return true;
  }
  case idText: {
    std::string buffer(message.text.readAll());
    std::cout << "idText " << buffer << std::endl;
    return true;
  }
  case idBodyContour: {
    BodyContour bc;
    message.bin >> bc;
    std::cout << "idBodyContour" << std::endl;
    currentFrame->store(std::move(bc));
    return true;
  }
  case idCameraMatrix: {
    CameraMatrix cm;
    message.bin >> cm;
    std::cout << "idCameraMatrix" << std::endl;
    currentFrame->store(std::move(cm));
    return true;
  }
  case idCameraInfo: {
    CameraInfo info;
    message.bin >> info;
    std::cout << "idCameraInfo camera: " << (info.camera == CameraInfo::upper ? "upper" : "lower") << std::endl;
    currentFrame->store(std::move(info));
    return true;
  }

  default: {
    std::cout << message.getMessageID() << " (unhandled)" << std::endl;
    break;
  }
  }

  return false;
}

StreamableDataFrame::StreamableDataFrame(int id, const DataFrame& frame) : id(id), time(frame.time) {}

StreamableDataset::StreamableDataset(int id, const Dataset& dataset, const std::unordered_map<DataFrame*, int>& frameToId)
    : id(id), name(dataset.name), source(dataset.source), sourcePath(dataset.sourcePath) {
  for (const auto& i : dataset.frames) {
    DataFrame* frame = i.second.get();
    frameIds.push_back(frameToId.at(frame));
  }
}
