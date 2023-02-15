/**
 * @file LibWorldModel.h
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

class LibWorldModel : public LibraryBase {
public:
  /** Constructor for initializing all members*/
  LibWorldModel();
  LibWorldModel(internal::ref_init_tag t) : LibraryBase(t){};

  void preProcess() override;

  void postProcess() override;

  Vector2<> kickEstimate(float maxTime = 5000.f);
  float timeSinceBallLastSeen();
  Vector2<> ballPosition(float localSeen = 1500.f);

  float nearestPlayerInSight(float angle);
};
