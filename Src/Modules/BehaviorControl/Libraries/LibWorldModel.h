/**
 * @file LibWorldModel.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

class LibWorldModel : public LibraryBase {
public:
  /** Constructor for initializing all members*/
  LibWorldModel();
  LibWorldModel(internal::ref_init_tag t) : LibraryBase(t){};

  void preProcess() override;

  void postProcess() override;

  float timeSinceBallLastSeen();
  Vector2<> ballPosition();

  float nearestPlayerInSight(float angle);
};
