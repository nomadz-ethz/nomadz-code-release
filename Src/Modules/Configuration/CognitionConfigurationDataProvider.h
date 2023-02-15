/**
 * @file CognitionConfigurationDataProvider.h
 *
 * This file declares a module that provides data loaded from configuration files.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Core/Module/Module.h"
#include "Core/MessageQueue/InMessage.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/CameraSettings.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/OdometryCorrectionTable.h"
#include "Representations/Infrastructure/TeamInfo.h"

MODULE(CognitionConfigurationDataProvider)
USES(CameraCalibration)
USES(CameraSettings)
REQUIRES(OwnTeamInfo)
PROVIDES_WITH_DRAW(FieldDimensions)
PROVIDES_WITH_MODIFY(CameraSettings)
PROVIDES_WITH_MODIFY(CameraCalibration)
PROVIDES_WITH_MODIFY(RobotDimensions)
PROVIDES_WITH_MODIFY(DamageConfiguration)
PROVIDES_WITH_MODIFY(DamageConfigurationHead)
PROVIDES_WITH_MODIFY_AND_DRAW(HeadLimits)
PROVIDES_WITH_MODIFY(OdometryCorrectionTables)
END_MODULE

class CognitionConfigurationDataProvider : public CognitionConfigurationDataProviderBase {
private:
  static PROCESS_WIDE_STORAGE(CognitionConfigurationDataProvider)
    theInstance; /**< Points to the only instance of this class in this process or is 0 if there is none. */

  FieldDimensions* theFieldDimensions{};
  CameraSettings* theCameraSettings{};
  CameraCalibration* theCameraCalibration{};
  RobotDimensions* theRobotDimensions{};
  DamageConfiguration* theDamageConfiguration{};
  DamageConfigurationHead* theDamageConfigurationHead{};
  HeadLimits* theHeadLimits{};
  OdometryCorrectionTables* theOdometryCorrectionTables{};

  void update(FieldDimensions& fieldDimensions);
  void update(CameraSettings& cameraSettings);
  void update(CameraCalibration& cameraCalibration);
  void update(RobotDimensions& robotDimensions);
  void update(DamageConfigurationHead& damageConfigurationHead);
  void update(DamageConfiguration& damageConfiguration);
  void update(HeadLimits& headLimits);
  void update(OdometryCorrectionTables& odometryCorrectionTables);

  void readDamageConfigurationHead();
  void readFieldDimensions();
  void readCameraSettings();
  void readCameraCalibration();
  void readRobotDimensions();
  void readDamageConfiguration();
  void readHeadLimits();
  void readOdometryCorrectionTables();

public:
  /**
   * Default constructor.
   */
  CognitionConfigurationDataProvider();

  /**
   * Destructor.
   */
  ~CognitionConfigurationDataProvider();
};
