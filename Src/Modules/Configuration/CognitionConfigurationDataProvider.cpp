/**
 * @file CognitionConfigurationDataProvider.cpp
 *
 * This file implements a module that provides data loaded from configuration files.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include <cstdio>

#include "CognitionConfigurationDataProvider.h"
#include "Core/System/File.h"

PROCESS_WIDE_STORAGE(CognitionConfigurationDataProvider) CognitionConfigurationDataProvider::theInstance = 0;

CognitionConfigurationDataProvider::CognitionConfigurationDataProvider() {
  theInstance = this;

  readFieldDimensions();
  readCameraSettings();
  readCameraCalibration();
  readRobotDimensions();
  readDamageConfiguration();
  readDamageConfigurationHead();
  // readDamageConfiguration();
  readHeadLimits();
  readOdometryCorrectionTables();
}

CognitionConfigurationDataProvider::~CognitionConfigurationDataProvider() {
  if (theFieldDimensions) {
    delete theFieldDimensions;
  }
  if (theCameraSettings) {
    delete theCameraSettings;
  }
  if (theCameraCalibration) {
    delete theCameraCalibration;
  }
  // if(theDamageConfiguration)
  // delete theDamageConfiguration;
  if (theDamageConfiguration) {
    delete theDamageConfiguration;
  }
  if (theDamageConfigurationHead) {
    delete theDamageConfigurationHead;
  }
  if (theHeadLimits) {
    delete theHeadLimits;
  }
  if (theOdometryCorrectionTables) {
    delete theOdometryCorrectionTables;
  }
  theInstance = 0;
}

void CognitionConfigurationDataProvider::update(FieldDimensions& fieldDimensions) {
  if (theFieldDimensions) {
    fieldDimensions = *theFieldDimensions;
    delete theFieldDimensions;
    theFieldDimensions = 0;
  }
  EXECUTE_ONLY_IN_DEBUG(fieldDimensions.drawPolygons(theOwnTeamInfo.teamColor););
}

void CognitionConfigurationDataProvider::update(CameraSettings& cameraSettings) {
  if (theCameraSettings) {
    cameraSettings = *theCameraSettings;
    delete theCameraSettings;
    theCameraSettings = 0;
  }
}

void CognitionConfigurationDataProvider::update(CameraCalibration& cameraCalibration) {
  if (theCameraCalibration) {
    cameraCalibration = *theCameraCalibration;
    delete theCameraCalibration;
    theCameraCalibration = 0;
  }
}

void CognitionConfigurationDataProvider::update(RobotDimensions& robotDimensions) {
  if (theRobotDimensions) {
    robotDimensions = *theRobotDimensions;
    delete theRobotDimensions;
    theRobotDimensions = 0;
  }
}

void CognitionConfigurationDataProvider::update(DamageConfigurationHead& damageConfigurationHead) {
  if (theDamageConfigurationHead) {
    damageConfigurationHead = *theDamageConfigurationHead;
    delete theDamageConfigurationHead;
    theDamageConfigurationHead = 0;
  }
}

void CognitionConfigurationDataProvider::update(DamageConfiguration& damageConfiguration) {
  if (theDamageConfiguration) {
    damageConfiguration = *theDamageConfiguration;
    delete theDamageConfiguration;
    theDamageConfiguration = 0;
  }
}

void CognitionConfigurationDataProvider::update(HeadLimits& headLimits) {
  if (theHeadLimits) {
    headLimits = *theHeadLimits;
    delete theHeadLimits;
    theHeadLimits = 0;
  }
}

void CognitionConfigurationDataProvider::update(OdometryCorrectionTables& odometryCorrectionTables) {
  if (theOdometryCorrectionTables) {
    odometryCorrectionTables = *theOdometryCorrectionTables;
    delete theOdometryCorrectionTables;
    theOdometryCorrectionTables = 0;
  }
}

void CognitionConfigurationDataProvider::readFieldDimensions() {
  ASSERT(!theFieldDimensions);

  theFieldDimensions = new FieldDimensions;
  theFieldDimensions->load();
}

void CognitionConfigurationDataProvider::readCameraSettings() {
  ASSERT(!theCameraSettings);

  InMapFile stream("cameraSettings.cfg");
  if (stream.exists()) {
    theCameraSettings = new CameraSettings;
    stream >> *theCameraSettings;
  }
}

void CognitionConfigurationDataProvider::readCameraCalibration() {
  ASSERT(!theCameraCalibration);

  InMapFile stream("cameraCalibration.cfg");
  if (stream.exists()) {
    theCameraCalibration = new CameraCalibration;
    stream >> *theCameraCalibration;
  }
}

void CognitionConfigurationDataProvider::readRobotDimensions() {
  ASSERT(!theRobotDimensions);

  InMapFile stream("robotDimensions.cfg");
  if (stream.exists()) {
    theRobotDimensions = new RobotDimensions;
    stream >> *theRobotDimensions;
  }
}

void CognitionConfigurationDataProvider::readDamageConfigurationHead() {
  ASSERT(!theDamageConfigurationHead);

  InMapFile stream("damageConfigurationHead.cfg");
  if (stream.exists()) {
    theDamageConfigurationHead = new DamageConfigurationHead;
    stream >> *theDamageConfigurationHead;
  }
}

void CognitionConfigurationDataProvider::readDamageConfiguration() {
  ASSERT(!theDamageConfiguration);

  InMapFile stream("damageConfiguration.cfg");
  if (stream.exists()) {
    theDamageConfiguration = new DamageConfiguration;
    stream >> *theDamageConfiguration;
  }
}

void CognitionConfigurationDataProvider::readHeadLimits() {
  ASSERT(!theHeadLimits);

  InMapFile stream("headLimits.cfg");
  if (stream.exists()) {
    theHeadLimits = new HeadLimits;
    stream >> *theHeadLimits;
  }
}

void CognitionConfigurationDataProvider::readOdometryCorrectionTables() {
  ASSERT(!theOdometryCorrectionTables);

  InMapFile stream("odometryCorrectionTables.cfg");
  if (stream.exists()) {
    theOdometryCorrectionTables = new OdometryCorrectionTables;
    stream >> *theOdometryCorrectionTables;
  }
}
MAKE_MODULE(CognitionConfigurationDataProvider, Cognition Infrastructure)
