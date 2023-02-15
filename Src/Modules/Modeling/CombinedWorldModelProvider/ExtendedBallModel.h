/**
 *@file ExtendedBallModel.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include "Representations/Modeling/BallModel.h"

class ExtendedBallModel : public BallModel {
public:
  int numberOfDetections;
  int numberOfActiveDetections;
  std::vector<unsigned> lastSeen = std::vector<unsigned>(TeamMateData::numOfPlayers);
  std::vector<float> seenFractions = std::vector<float>(TeamMateData::numOfPlayers);
  float firstSeen;

  bool seenByMe;
  float maxSideConfidenceOthers;

  ExtendedBallModel()
      : numberOfDetections(0), numberOfActiveDetections(0), firstSeen(0), seenByMe(false), maxSideConfidenceOthers(0.0) {
    timeWhenLastSeen = 0;
    estimate.velocity = {0.0, 0.0};
    estimate.position = {0.0, 0.0};
    for (int i = 0; i < TeamMateData::numOfPlayers; ++i) {
      lastSeen[i] = 0;
      seenFractions[i] = 0.0f;
    }
  }
  ExtendedBallModel(const BallModel& ballModel)
      : BallModel(ballModel), numberOfDetections(1), numberOfActiveDetections(1), firstSeen(ballModel.timeWhenLastSeen),
        seenByMe(false), maxSideConfidenceOthers(0.0) {
    for (int i = 0; i < TeamMateData::numOfPlayers; ++i) {
      lastSeen[i] = 0;
      seenFractions[i] = 0.0f;
    }
    seenFraction = 0.0;
    timeWhenLastSeen = ballModel.timeWhenLastSeen;
    estimate.velocity = {0, 0};
  }

  float dist(const BallModel& other) { return (estimate.position - other.estimate.position).abs(); }
  void merge(const BallModel& other, bool me, float sideConfidence) {
    // combine ball position (ignore velocity)
    Matrix2x2<> covPlusSensorCov = estimate.positionCovariance;
    covPlusSensorCov += other.estimate.positionCovariance;
    Matrix2x2<> k = estimate.positionCovariance * covPlusSensorCov.invert();
    Vector2<> innovation = other.estimate.position - estimate.position;
    Vector2<> correction = k * innovation;
    estimate.position += correction;
    estimate.positionCovariance -= k * estimate.positionCovariance;
    // combine timing informations
    timeWhenLastSeen = std::max(other.timeWhenLastSeen, timeWhenLastSeen);
    numberOfActiveDetections++;
    // combine own information
    seenByMe = seenByMe || me;
    if (!me) {
      maxSideConfidenceOthers = std::max(maxSideConfidenceOthers, sideConfidence);
    }
  }
  bool isBetter(const ExtendedBallModel& other, bool noSelf) {
    int realActiveDetections = numberOfActiveDetections - noSelf * seenByMe;
    int realActiveOtherDetections = other.numberOfActiveDetections - noSelf * other.seenByMe;

    int realDetections = numberOfDetections - noSelf * seenByMe;
    int realOtherDetections = other.numberOfDetections - noSelf * other.seenByMe;

    if (realActiveDetections == realActiveOtherDetections) {
      if (realDetections == realOtherDetections) {
        return seenFraction > other.seenFraction;
      } else {
        return realDetections > realOtherDetections;
      }
    } else {
      return realActiveDetections > realActiveOtherDetections;
    }
  }
  void updateSeenFraction() {
    seenFraction = 0;
    for (int i = 1; i < TeamMateData::numOfPlayers; ++i) {
      seenFraction += seenFractions[i];
    }
  }
};
