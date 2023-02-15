/**
 * @file MotionConfigurationDataProvider.cpp
 *
 * This file implements a module that provides data loaded from configuration files.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "MotionConfigurationDataProvider.h"

MotionConfigurationDataProvider::MotionConfigurationDataProvider() {
  readDamageConfiguration();
  readDamageConfigurationHead();
  readJointCalibration();
  readSensorCalibration();
  readOdometryCorrectionTables();
  readRobotDimensions();
  readMassCalibration();
  readHardnessSettings();
  readWalk2014Modifier();
}

MotionConfigurationDataProvider::~MotionConfigurationDataProvider() {

  if (theDamageConfiguration) {
    delete theDamageConfiguration;
  }
  if (theDamageConfigurationHead) {
    delete theDamageConfigurationHead;
  }
  if (theJointCalibration) {
    delete theJointCalibration;
  }
  if (theSensorCalibration) {
    delete theSensorCalibration;
  }
  if (theOdometryCorrectionTables) {
    delete theOdometryCorrectionTables;
  }
  if (theRobotDimensions) {
    delete theRobotDimensions;
  }
  if (theMassCalibration) {
    delete theMassCalibration;
  }
  if (theHardnessSettings) {
    delete theHardnessSettings;
  }
  // if(theDamageConfiguration)
  // delete theDamageConfiguration;
}

void MotionConfigurationDataProvider::update(DamageConfiguration& damageConfiguration) {
  if (theDamageConfiguration) {
    damageConfiguration = *theDamageConfiguration;
    delete theDamageConfiguration;
    theDamageConfiguration = 0;
  }
}

void MotionConfigurationDataProvider::update(DamageConfigurationHead& damageConfigurationHead) {
  if (theDamageConfigurationHead) {
    damageConfigurationHead = *theDamageConfigurationHead;
    delete theDamageConfigurationHead;
    theDamageConfigurationHead = 0;
  }
}

void MotionConfigurationDataProvider::update(JointCalibration& jointCalibration) {
  if (theJointCalibration) {
    jointCalibration = *theJointCalibration;
    delete theJointCalibration;
    theJointCalibration = 0;
  }
  DEBUG_RESPONSE_ONCE("representation:JointCalibration", OUTPUT(idJointCalibration, bin, jointCalibration););
}

void MotionConfigurationDataProvider::update(SensorCalibration& sensorCalibration) {
  if (theSensorCalibration) {
    sensorCalibration = *theSensorCalibration;
    delete theSensorCalibration;
    theSensorCalibration = 0;
  }
}

void MotionConfigurationDataProvider::update(OdometryCorrectionTables& odometryCorrectionTables) {
  if (theOdometryCorrectionTables) {
    odometryCorrectionTables = *theOdometryCorrectionTables;
    delete theOdometryCorrectionTables;
    theOdometryCorrectionTables = nullptr;
  }
}

void MotionConfigurationDataProvider::update(RobotDimensions& robotDimensions) {
  if (theRobotDimensions) {
    robotDimensions = *theRobotDimensions;
    delete theRobotDimensions;
    theRobotDimensions = 0;
  }
  DEBUG_RESPONSE_ONCE("representation:RobotDimensions", OUTPUT(idRobotDimensions, bin, robotDimensions););
}

void MotionConfigurationDataProvider::update(MassCalibration& massCalibration) {
  if (theMassCalibration) {
    massCalibration = *theMassCalibration;
    delete theMassCalibration;
    theMassCalibration = 0;
  }
}

void MotionConfigurationDataProvider::update(HardnessSettings& hardnessSettings) {
  if (theHardnessSettings) {
    hardnessSettings = *theHardnessSettings;
    delete theHardnessSettings;
    theHardnessSettings = 0;
  }
}

void MotionConfigurationDataProvider::update(Walk2014Modifier& Walk2014Modifier) {
  if (theWalk2014Modifier) {
    Walk2014Modifier = *theWalk2014Modifier;
    delete theWalk2014Modifier;
    theWalk2014Modifier = 0;
  }
}

void MotionConfigurationDataProvider::readDamageConfiguration() {
  ASSERT(!theDamageConfiguration);

  InMapFile stream("damageConfiguration.cfg");

  if (stream.exists()) {
    theDamageConfiguration = new DamageConfiguration;
    stream >> *theDamageConfiguration;
  }
}

void MotionConfigurationDataProvider::readDamageConfigurationHead() {
  ASSERT(!theDamageConfigurationHead);

  InMapFile stream("damageConfigurationHead.cfg");

  if (stream.exists()) {
    theDamageConfigurationHead = new DamageConfigurationHead;
    stream >> *theDamageConfigurationHead;
  }
}

void MotionConfigurationDataProvider::readJointCalibration() {
  ASSERT(!theJointCalibration);

  InMapFile stream("jointCalibration.cfg");
  if (stream.exists()) {
    theJointCalibration = new JointCalibration;
    stream >> *theJointCalibration;
  }
}

void MotionConfigurationDataProvider::readSensorCalibration() {
  ASSERT(!theSensorCalibration);

  InMapFile stream("sensorCalibration.cfg");
  if (stream.exists()) {
    theSensorCalibration = new SensorCalibration;
    stream >> *theSensorCalibration;
  }
}

/*void MotionConfigurationDataProvider::readMotionSettings()
{
  ASSERT(!theMotionSettings);

  InMapFile stream("motionSettings.cfg");
  if(stream.exists())
  {
    theMotionSettings = new MotionSettings;
    stream >> *theMotionSettings;
  }
}

void MotionConfigurationDataProvider::readFieldDimensions()
{
  ASSERT(!theFieldDimensions);

  theFieldDimensions = new FieldDimensions;
  theFieldDimensions->load();
}


void MotionConfigurationDataProvider::readStiffnessSettings()
{
  ASSERT(!theStiffnessSettings);

  InMapFile stream("stiffnessSettings.cfg");
  if(stream.exists())
  {
    theStiffnessSettings = new StiffnessSettings;
    stream >> *theStiffnessSettings;
  }
}
*/

void MotionConfigurationDataProvider::readOdometryCorrectionTables() {
  ASSERT(!theOdometryCorrectionTables);

  InMapFile stream("odometryCorrectionTables.cfg");
  if (stream.exists()) {
    theOdometryCorrectionTables = new OdometryCorrectionTables;
    stream >> *theOdometryCorrectionTables;
  }
}

void MotionConfigurationDataProvider::readRobotDimensions() {
  ASSERT(!theRobotDimensions);

  InMapFile stream("robotDimensions.cfg");
  if (stream.exists()) {
    theRobotDimensions = new RobotDimensions;
    stream >> *theRobotDimensions;
  }
}

void MotionConfigurationDataProvider::readMassCalibration() {
  ASSERT(!theMassCalibration);

  InMapFile stream("massCalibration.cfg");
  if (stream.exists()) {
    theMassCalibration = new MassCalibration;
    stream >> *theMassCalibration;
  }
}

void MotionConfigurationDataProvider::readHardnessSettings() {
  ASSERT(!theHardnessSettings);

  InMapFile stream("hardnessSettings.cfg");
  if (stream.exists()) {
    theHardnessSettings = new HardnessSettings;
    stream >> *theHardnessSettings;
  }
}

void MotionConfigurationDataProvider::readWalk2014Modifier() {
  ASSERT(!theWalk2014Modifier);

  InMapFile stream("walk2014Modifier.cfg");
  if (stream.exists()) {
    theWalk2014Modifier = new Walk2014Modifier;
    stream >> *theWalk2014Modifier;
  }
}

MAKE_MODULE(MotionConfigurationDataProvider, Motion Infrastructure)
