/**
 * @file NaoBody.h
 *
 * Declaration of a class for accessing the body of the Nao via naobridge.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#pragma once

#include "Core/Communication/RoboCupControlData.h"

/**
 * @class NaoBody
 * Encapsulates communication with naobridge.
 */
class NaoBody {
public:
  /** Default constructor */
  NaoBody();

  /** Destructor */
  ~NaoBody();

  /** Initializes access to naobridge
   * @return Whether the initialization was successful or not */
  bool init();

  /** Shutdown cleanly by setting flag. */
  void shutdown();

  /** Finalize access to naobridge */
  void cleanup();

  /** Waits for a new set of sensor data */
  bool wait();

  /** Activates the eye-blinking mode to indicate a crash.
   * @param termSignal The termination signal that was raised by the crash. */
  void setCrashed(int termSignal);

  /** Accesses the name of the robot's body.
   * @return The name. */
  const char* getName() const;

  /** Accesses the sensor value buffer.
   * @return An array of sensor values. Ordered corresponding to \c lbhSensorNames of \c nomadz.h. */
  float* getSensors();

  /** Accesses the lastst data from the GameController. */
  const RoboCup::RoboCupGameControlData& getGameControlData() const;

  /** Accesses temperature sensors */
  void getTemperature(float& cpu, float& mb);

  /** Determine status of wlan hardware. */
  bool getWlanStatus();

  /** Accesses the actuator value buffer for writing.
   * @param actuators A reference to a variable to store a pointer to the actuator value buffer in. Ordered corresponding to
   * \c lbhActuatorNames of \c nomadz.h. */
  void openActuators(float*& actuators);

  /** Commits the actuator value buffer. */
  void closeActuators();

  /** Sets the current robot info that will be used by naobridge. */
  void setRobotInfo(uint8_t teamNumber,
                    uint8_t teamColour,
                    uint8_t playerNumber,
                    uint8_t playerFallen,
                    float poseX,
                    float poseY,
                    float poseTheta,
                    float ballAge,
                    float ballX,
                    float ballY);

private:
  int writingActuators; /**< The index of the opened exclusive actuator writing buffer. */

  FILE* fdCpuTemp;
};
