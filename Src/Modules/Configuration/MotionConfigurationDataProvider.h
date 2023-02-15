/**
 * @file MotionConfigurationDataProvider.h
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
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/SensorCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/OdometryCorrectionTable.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/MotionControl/Walk2014Modifier.h"

MODULE(MotionConfigurationDataProvider)
PROVIDES_WITH_MODIFY(JointCalibration)
PROVIDES_WITH_MODIFY(SensorCalibration)
PROVIDES_WITH_MODIFY(RobotDimensions)
PROVIDES_WITH_MODIFY(DamageConfiguration)
PROVIDES_WITH_MODIFY(DamageConfigurationHead)
PROVIDES_WITH_MODIFY(MassCalibration)
PROVIDES_WITH_MODIFY(OdometryCorrectionTables)
PROVIDES_WITH_MODIFY(HardnessSettings)
PROVIDES(Walk2014Modifier)
END_MODULE

class MotionConfigurationDataProvider : public MotionConfigurationDataProviderBase {
private:
  JointCalibration* theJointCalibration{};
  SensorCalibration* theSensorCalibration{};
  RobotDimensions* theRobotDimensions{};
  DamageConfiguration* theDamageConfiguration{};
  DamageConfigurationHead* theDamageConfigurationHead{};
  MassCalibration* theMassCalibration{};
  HardnessSettings* theHardnessSettings{};
  OdometryCorrectionTables* theOdometryCorrectionTables{};
  Walk2014Modifier* theWalk2014Modifier{};

  void update(JointCalibration& jointCalibration);
  void update(SensorCalibration& sensorCalibration);
  void update(RobotDimensions& robotDimensions);
  void update(MassCalibration& massCalibration);
  void update(OdometryCorrectionTables& odometryCorrectionTables);
  void update(HardnessSettings& hardnessSettings);
  void update(DamageConfiguration& damageConfiguration);
  void update(DamageConfigurationHead& damageConfigurationHead);
  void update(Walk2014Modifier& walk2014Modifier);

  void readDamageConfiguration();
  void readDamageConfigurationHead();
  // void readFieldDimensions();
  // void readMotionSettings();
  // void readStiffnessSettings();
  void readJointCalibration();
  void readSensorCalibration();
  void readOdometryCorrectionTables();
  void readRobotDimensions();
  void readMassCalibration();
  void readHardnessSettings();
  void readWalk2014Modifier();

public:
  /**
   * Default constructor.
   */
  MotionConfigurationDataProvider();

  /**
   * Destructor.
   */
  ~MotionConfigurationDataProvider();
};
