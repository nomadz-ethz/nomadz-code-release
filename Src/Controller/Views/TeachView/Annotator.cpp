/**
 * @file Annotator.cpp
 *
 * Implementation of Annotator classes
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include <iostream>
#include "Annotator.h"
#include "Controller/RobotConsole.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/BallSpots.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/ShapePercept.h"
#include "Core/MessageQueue/MessageIDs.h"

void RoboticAnnotator::spawnRobot(std::string name) {
  robot = console.spawnAnnotatorRobot(name);
  console.executeConsoleCommand("call Annotator");

  // Start CognitionLogDataProvider (TODO and MotionLogDataProvider?) to handle incoming "log" messages
  RobotConsole* rc = robot->getRobotProcess();
  console.executeConsoleCommand("mr Image CognitionLogDataProvider", rc);
  console.executeConsoleCommand("mr CameraInfo CognitionLogDataProvider", rc);
  console.executeConsoleCommand("mr CameraMatrix CognitionLogDataProvider", rc);
}

std::vector<std::unique_ptr<Annotation>> RoboticAnnotator::annotate(const std::vector<DataFrame*>& frames) {
  if (!robot) {
    return std::vector<std::unique_ptr<Annotation>>();
  }

  std::vector<std::unique_ptr<Annotation>> output;
  const int numFrames = frames.size();

  int f = rand() % numFrames;
  int idx = 0;

  for (auto i = frames.begin(); i != frames.end(); ++i, ++idx) {
    if (idx != f) {
      continue;
    }

    const DataFrame& frame = **i;
    if (!sendFrame(frame, true)) {
      continue;
    }

    break;
  }
  return output;
}

bool RoboticAnnotator::start(const std::vector<DataFrame*>& frames) {
  if (!toProcess.empty()) {
    // There is an existing annotation session; don't start a new one
    std::cerr << "RoboticAnnotator: cannot start: annotation already running" << std::endl;
    return false;
  }

  if (!receivedFrames.empty()) {
    std::cerr << "RoboticAnnotator: old results being overwritten by new annotation session" << std::endl;
  }

  for (DataFrame* frame : frames) {
    toProcess.insert(std::make_pair(frame->time, frame));
  }

  console.executeConsoleCommand("dr representation:BallSpots");
  console.executeConsoleCommand("dr representation:ShapePercept");

  robot->getRobotProcess()->roboticAnnotator = this;

  sendFrame(*(toProcess.begin()->second));
  return true;
}

void RoboticAnnotator::handleMessage(InMessage& message) {
  std::cout << "RoboticAnnotator::handleMessage ID: " << message.getMessageID() << std::endl;
  switch (message.getMessageID()) {
  case idProcessBegin: {
    incomingFrame.reset(new DataFrame());
    break;
  }
  case idProcessFinished: {
    // TODO Remove these prints after virtual robot proves itself quite adept at sending the right frames
    if (incomingFrame->time == -1) {
      std::cout << "RoboticAnnotator::handleMessage Ignoring incomingFrame because missing timestamp" << std::endl;
    }
    if (!incomingFrame->has<BallSpots, ShapePercept>()) {
      std::cout << "RoboticAnnotator::handleMessage Ignoring incomingFrame because missing BallSpots or ShapePercept"
                << std::endl;
    } else {
      auto matchingFrame = toProcess.find(incomingFrame->time);
      if (matchingFrame != toProcess.end()) {
        std::cout << "RoboticAnnotator::handleMessage Stored frame at time " << incomingFrame->time
                  << "; toProcess.size(): " << toProcess.size() << std::endl;
        receivedFrames.push_back(std::make_pair(matchingFrame->second, std::move(incomingFrame)));
        toProcess.erase(matchingFrame);
        emit progressed(receivedFrames.size(), receivedFrames.size() + toProcess.size());
      } else {
        std::cout << "RoboticAnnotator::handleMessage Ignoring incomingFrame because timestamp " << incomingFrame->time
                  << " does not match any requested" << std::endl;
      }
    }
    incomingFrame.reset(nullptr);

    // Send another frame
    while (!toProcess.empty()) {
      auto frameToSend = toProcess.begin();
      if (sendFrame(*(frameToSend->second))) {
        break;
      }
      toProcess.erase(frameToSend);
      std::cout << "RoboticAnnotator::handleMessage Failed to send frame " << frameToSend->first << ", skipping"
                << std::endl;
    }
    break;
  }
  case idImage: {
    Image img;
    message.bin >> img;
    incomingFrame->time = img.timeStamp;
    break;
  }
  case idBallSpots: {
    BallSpots bs;
    message.bin >> bs;
    incomingFrame->store<BallSpots>(std::move(bs));
    break;
  }
  case idShapePercept: {
    ShapePercept sp;
    message.bin >> sp;
    incomingFrame->store<ShapePercept>(std::move(sp));
    break;
  }
  default:
    break;
  }

  if (toProcess.empty()) {
    robot->getRobotProcess()->roboticAnnotator = nullptr;
    std::cout << "RoboticAnnotator::handleMessage Finished" << std::endl;
    emit finished();
  }
}

std::vector<std::unique_ptr<Annotation>> RoboticAnnotator::giveResults() {
  std::vector<std::unique_ptr<Annotation>> results;
  for (std::pair<DataFrame*, std::unique_ptr<DataFrame>>& frame : receivedFrames) {
    DataFrame* sourceFrame = frame.first;
    std::unique_ptr<DataFrame>& receivedFrame = frame.second;

    const ShapePercept* sp = receivedFrame->get<ShapePercept>();
    const BallSpots* bs = receivedFrame->get<BallSpots>();

    for (const BallSpot& b : bs->ballSpots) {
      std::unique_ptr<Annotation> annotation(
        new Annotation(sourceFrame, Annotation::Circle(b.radius, b.position.x, b.position.y)));
      results.push_back(std::move(annotation));
    }

    for (const ShapePercept::LineSegment& l : sp->segments) {
      const float x1 = l.p1.x;
      const float y1 = l.p1.y;
      const float x2 = l.p2.x;
      const float y2 = l.p2.y;

      Annotation::Polygon polygon({{x1, y1}, {x2, y2}});

      std::unique_ptr<Annotation> annotation(new Annotation(sourceFrame, polygon));
      results.push_back(std::move(annotation));
    }
  }

  receivedFrames.clear();
  return results;
}

// lock = false means we're already inside a RobotConsole SyncObject (so we can use its debugOut without locking)
bool RoboticAnnotator::sendFrame(const DataFrame& frame, bool lock) {
  std::cout << "RoboticAnnotator::sendFrame frame.time: " << frame.time << ", lock: " << lock << std::endl;
  if (!frame.has<Image>() && !frame.has<JPEGImage>()) {
    std::cout << "RoboticAnnotator::sendFrame frame " << frame.time << " missing an Image or JPEGImage" << std::endl;
    return false;
  }

  if (!frame.has<CameraInfo, CameraMatrix>()) {
    std::cout << "RoboticAnnotator::sendFrame frame " << frame.time << " missing CameraInfo or CameraMatrix" << std::endl;
    return false;
  }

  RobotConsole* rc = robot->getRobotProcess();

  const Image* image;
  Image tmpImage;
  if (frame.has<Image>()) {
    image = frame.get<Image>();
  } else {
    frame.get<JPEGImage>()->toImage(tmpImage);
    image = &tmpImage;
  }
  const CameraInfo* cameraInfo = frame.get<CameraInfo>();
  const CameraMatrix* cameraMatrix = frame.get<CameraMatrix>();

  // Apparently 'c'ognition sends upper images while 'd'ebug sends lower images
  const char process = (cameraInfo->camera == CameraInfo::upper) ? 'c' : 'd';

  MessageQueue tempQ;

  tempQ.out.bin << process;
  tempQ.out.finishMessage(idProcessBegin);
  if (lock) {
    rc->sendDebugMessage(tempQ.in);
  } else {
    tempQ.in >> rc->getDebugOut();
  }
  tempQ.clear();

  tempQ.out.bin << *image;
  tempQ.out.finishMessage(idImage);
  if (lock) {
    rc->sendDebugMessage(tempQ.in);
  } else {
    tempQ.in >> rc->getDebugOut();
  }
  tempQ.clear();

  tempQ.out.bin << *cameraInfo;
  tempQ.out.finishMessage(idCameraInfo);
  if (lock) {
    rc->sendDebugMessage(tempQ.in);
  } else {
    tempQ.in >> rc->getDebugOut();
  }
  tempQ.clear();

  tempQ.out.bin << *cameraMatrix;
  tempQ.out.finishMessage(idCameraMatrix);
  if (lock) {
    rc->sendDebugMessage(tempQ.in);
  } else {
    tempQ.in >> rc->getDebugOut();
  }
  tempQ.clear();

  tempQ.out.bin << process;
  tempQ.out.finishMessage(idProcessFinished);
  if (lock) {
    rc->sendDebugMessage(tempQ.in);
  } else {
    tempQ.in >> rc->getDebugOut();
  }
  tempQ.clear();

  std::cout << "Sent frame " << frame.time << std::endl;
  return true;
}
