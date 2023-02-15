/**
 * @file lola_bridge.cpp
 *
 * Declaration of shared data between framework and core
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt
 * (c) 2022 BHuman and NomadZ team
 */

#include "lola_bridge.h"

#include <cmath>
#include <csignal>
#include <cstring>
#include <cstdio>
#include <chrono>
#include <map>
#include <memory>
#include <algorithm>

#include <sys/types.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <sys/resource.h>
#include <unistd.h>
#include <limits.h>

#include <boost/asio.hpp>
#include <msgpack.hpp>

#include "lola_frame.h"
#include "game_control.h"
#include "naobridge/nomadz.h"

using namespace boost::asio;
using namespace std::chrono;

// NOTE: should be less than 12 to stay within the current cycle
#define DELAY_ACT_MS 2

#define DELAY_UNSTIFF 800
#define DELAY_STIFF 100

static const float sitDownAngles[25] = {0.f,    0.f,    0.89f, 0.06f, 0.26f, -0.62f, -1.57f, 0.f,    0.89f,
                                        -0.06f, -0.26f, 0.62f, 1.57f, 0.f,   0.f,    0.f,    -0.89f, 2.11f,
                                        -1.19f, 0.f,    0.f,   -0.9f, 2.12f, -1.19f, 0.f};

// bridge <-> lola
boost::asio::io_service io_serv;
std::shared_ptr<boost::asio::local::stream_protocol::socket> sock;

// bridge <-> nomadz
int memory_handle;    /**< The file handle of the shared memory. */
LBHData* shared_data; /**< The shared data between nomadz and lola bridge */
sem_t* sem;           /**< The semaphore used to notify nomadz about new data. */

// bridge internal
static const int allowedFrameDrops = 3;
static const int allowedFrameDropsCrashed = 100;
enum State { sitting, standingUp, standing, sittingDown, preShuttingDown, shuttingDown } state;
enum ChestButtonState { notPressed, pressed, released } chestButtonState;
long long whenLastNoAllHeadButtons;        /**< When last time not all head buttons where pushed. */
bool unstiffReset;                         /**< If unstiff mode switch is reset (all head button released). */
bool unstiff;                              /**< If unstiff mode is activated. */
bool previousChestButtonWasPressed;        /**< Whether the chest button was pressed during the previous cycle. */
long long whenLastChestButtonStateChanged; /**< When last state change of the chest button occured (DCM time). */
float defaultHardness;                     /**< The default hardness to apply if not set explicitly. */
float defaultActiveHardness; /**< The default hardness when a motion is being executed e.g. when switching from sitting to
                                standing. This should be usually > 0.7 */
long long curTime;
int lastReadingActuators; /**< The previous actuators read. For detecting frames without seemingly new data from nomadz. */
float lastPositionActuators[lbhNumOfPositionActuatorIds];
float lastHardnessActuators[lbhNumOfPositionActuatorIds];
RoboCup::RoboCupGameControlData lastGameCtrlData; /**< The last game control data received */
int actuatorDrops;                                /**< The number of frames without seemingly new data from nomadz. */
int frameDrops;                                   /**< The number frames without a reaction from nomadz. */
float startAngles[lbhNumOfPositionActuatorIds];   /**< Start angles for standing up or sitting down. */
float startHardness[lbhNumOfPositionActuatorIds]; /**< Start hardness for sitting down. */
float phase; /**< How far is the Nao in its current standing up or sitting down motion [0 ... 1]? */
/**
 * Auxiliary phase used for the final arm motion during stand up and sit down.
 * Basically a hack to split the motion into two parts. This phase is triggered
 * when the main phase reached 0.5.
 */
float phase_modified;
static bool nao_shutdown = false;
void ctrlc_handler(int) {
  nao_shutdown = true;
}

// initialize shared memory and socket
bool init() {
  // init
  shared_data = (LBHData*)MAP_FAILED;
  sem = SEM_FAILED;

  // create shared memory
  memory_handle = shm_open(LBH_MEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
  if (memory_handle == -1) {
    perror("naobridge: shm_open");
    return false;
  } else if (ftruncate(memory_handle, sizeof(LBHData)) == -1) {
    perror("naobridge: ftruncate");
    return false;
  }

  // map the shared memory
  shared_data = (LBHData*)mmap(NULL, sizeof(LBHData), PROT_READ | PROT_WRITE, MAP_SHARED, memory_handle, 0);
  if (shared_data == MAP_FAILED) {
    perror("naobridge: mmap");
    return false;
  }
  memset(shared_data, 0, sizeof(LBHData));
  shared_data->robotInfo[teamColour] = -1;
  shared_data->robotInfo[teamNumber] = 0;

  // open semaphore
  sem = sem_open(LBH_SEM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR, 0);
  if (sem == SEM_FAILED) {
    perror("naobridge: sem_open");
    return false;
  }

  // set the robot name
  gethostname(shared_data->robotName, sizeof(shared_data->robotName));

  // initialize data
  for (int i = 0; i < lbhNumOfPositionActuatorIds; ++i) {
    lastPositionActuators[i] = -1.f;
  }
  for (int i = 0; i < lbhNumOfHardnessActuatorIds; ++i) {
    lastHardnessActuators[i] = -1.f;
  }
  lastReadingActuators = -1;
  actuatorDrops = 0;
  frameDrops = allowedFrameDrops + 1;
  // start in shutdown state as required for unstiffed state
  state = shuttingDown;
  phase = 0.f;
  whenLastNoAllHeadButtons = 0;
  unstiff = true;
  unstiffReset = true;
  previousChestButtonWasPressed = false;
  whenLastChestButtonStateChanged = 0;
  chestButtonState = notPressed;
  defaultHardness = 0.0;
  defaultActiveHardness = defaultHardness;
  // initialize the socket
  while (access(LLB_SOCKET, F_OK) == -1) {
    fprintf(stderr, "naobridge: socket not yet available, waiting for a second...\n");
    sleep(1);
  }
  sock = std::make_shared<local::stream_protocol::socket>(io_serv);
  sock->connect(LLB_SOCKET);

  // initialize game control
  ctrl_init();

  return true;
}

// deinitialize shared memory and socket
void close() {
  // stop game control
  ctrl_close();

  // close and unlink semaphore and shared data
  if (sem != SEM_FAILED) {
    sem_close(sem);
    sem_unlink(LBH_SEM_NAME);
    sem = SEM_FAILED;
  }
  if (shared_data != MAP_FAILED) {
    munmap(shared_data, sizeof(LBHData));
    close(memory_handle);
    shm_unlink(LBH_MEM_NAME);
    shared_data = (LBHData*)MAP_FAILED;
  }

  // close socket
  sock->close();
}

// read the sensor data from socket to shared mem
void read_sensors(LolaFrameHandler& frame_handler) {
  // read from stream
  constexpr int max_len = 100000;
  char data[max_len] = {'\0'};
  int size = sock->receive(boost::asio::buffer(data, max_len));
  const LolaSensorFrame& sensor_frame = frame_handler.unpack(data, size);

  if (shared_data != MAP_FAILED) {
    // determine index to write in shared memory block
    int writingSensors = 0;
    if (writingSensors == shared_data->newestSensors)
      ++writingSensors;
    if (writingSensors == shared_data->readingSensors)
      if (++writingSensors == shared_data->newestSensors)
        ++writingSensors;
    assert(writingSensors != shared_data->newestSensors);
    assert(writingSensors != shared_data->readingSensors);

    // copy sensors to shared memory block
    float* sensors = shared_data->sensors[writingSensors];
    for (int i = 0; i < lbhNumOfSensorIds; ++i)
      sensors[i] = sensor_frame.data[i];

    // handle core state machine
    handle_core_state(sensors);

    // read robocup game control data
    shared_data->gameControlData[writingSensors] = lastGameCtrlData;
    ctrl_handle_input(sensors, shared_data->gameControlData[writingSensors], state == standing, unstiff);
    lastGameCtrlData = shared_data->gameControlData[writingSensors];

    // update newest sensor data
    shared_data->newestSensors = writingSensors;
  }

  // raise the semaphore
  if (sem != SEM_FAILED) {
    int sval;
    if (sem_getvalue(sem, &sval) == 0) {
      if (sval < 1) {
        sem_post(sem);
        if (shared_data->state != okState) {
          // Consider frames to be dropped when not in OK state.
          frameDrops = allowedFrameDrops;
        } else {
          frameDrops = 0;
        }
      } else {
        if (frameDrops == 0)
          fprintf(stderr, "naobridge: dropped sensor data.\n");
        ++frameDrops;
      }
    }
  }
}

// handle core behavior from sensors
void handle_core_state(float* sensors) {
  if (sensors[headFrontSensor] == 0.f || sensors[headMiddleSensor] == 0.f || sensors[headRearSensor] == 0.f) {
    whenLastNoAllHeadButtons = curTime;
  }
  if (sensors[headFrontSensor] == 0.f && sensors[headMiddleSensor] == 0.f && sensors[headRearSensor] == 0.f) {
    unstiffReset = true;
  }

  if (whenLastNoAllHeadButtons != 0 && curTime - whenLastNoAllHeadButtons > DELAY_UNSTIFF) {
    if (unstiffReset && !unstiff) {
      state = preShuttingDown;
      unstiff = true;
    }
    unstiffReset = false;
  }

  if (unstiff) {
    bool chestButtonPressed = sensors[chestButtonSensor] != 0.f;
    switch (chestButtonState) {
    case notPressed: {
      if (chestButtonPressed != previousChestButtonWasPressed && curTime - whenLastChestButtonStateChanged >= DELAY_STIFF) {
        if (chestButtonPressed) {
          chestButtonState = pressed;
        }
        previousChestButtonWasPressed = chestButtonPressed;
        whenLastChestButtonStateChanged = curTime;
      }
      break;
    }
    case pressed: {
      if (!chestButtonPressed) {
        chestButtonState = released;
      }
      break;
    }
    case released: {
      state = sitting;
      unstiff = false;
      chestButtonState = notPressed;
      break;
    }
    }
  }
}

// set the eyes based on the current core state
void set_override_leds(float* actuators) {
  float blink = float(curTime / 500 & 1);

  if (nao_shutdown) {
    for (int i = faceLedRedLeft0DegActuator; i <= rFootLedBlueActuator; ++i) {
      actuators[i] = 0.f;
    }
  } else if (frameDrops > allowedFrameDrops) {
    for (int i = faceLedRedLeft0DegActuator; i <= faceLedBlueRight315DegActuator; ++i) {
      actuators[i] = 0.f;
    }
  }

  if (unstiff && !nao_shutdown) {
    actuators[chestBoardLedRedActuator] = 0.f;
    actuators[chestBoardLedGreenActuator] = 0.f;
    actuators[chestBoardLedBlueActuator] = blink;
  }

  if (nao_shutdown) {
    // red lights during real shutdown
    actuators[faceLedRedLeft180DegActuator] = 1.f;
    actuators[faceLedRedRight180DegActuator] = 1.f;
  } else {
    if (frameDrops > allowedFrameDropsCrashed) {
      if (shared_data->state != shutdownState) {
        // set the "naobridge is active and nomadz crashed" leds
        for (int i = faceLedRedLeft0DegActuator; i <= faceLedRedLeft315DegActuator; ++i)
          actuators[i] = blink;
        for (int i = faceLedRedRight0DegActuator; i <= faceLedRedRight315DegActuator; ++i)
          actuators[i] = 1.f - blink;
      } else {
        // set the "naobridge is active and nomadz is not running" LEDs
        actuators[faceLedBlueLeft180DegActuator] = blink;
        actuators[faceLedBlueRight180DegActuator] = blink;
      }
    }
  }
}

// internal handling of actuator state (FIXME: copied from bhuman, can be reworked)
float* handle_state(LolaFrameHandler& frame_handler, float* actuators) {
  static float controlledActuators[lbhNumOfActuatorIds];

  switch (state) {
  sitting:
    state = sitting;

  case sitting:
    memset(controlledActuators, 0, sizeof(controlledActuators));
    for (int i = 0; i < lbhNumOfPositionActuatorIds; ++i) {
      controlledActuators[i] = sitDownAngles[i];
      controlledActuators[i + headYawHardnessActuator] = defaultHardness;
    }
    if (frameDrops > allowedFrameDrops ||
        (actuators[lHipPitchHardnessActuator] == 0.f && actuators[rHipPitchHardnessActuator] == 0.f)) {
      return controlledActuators;
    }

    for (int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
      startAngles[i] = frame_handler.sensor_frame.data[i * 3];

  standingUp:
    state = standingUp;
    phase = 0.f;
    defaultActiveHardness = 0.7f;
  case standingUp:
    if (phase < 1.f && frameDrops <= allowedFrameDrops) {
      memset(controlledActuators, 0, sizeof(controlledActuators));
      phase = std::min(phase + 0.01f, 1.f);
      for (int i = 0; i < lbhNumOfPositionActuatorIds; ++i) {
        if (i != lElbowRollPositionActuator && i != rElbowRollPositionActuator && i != lShoulderPitchPositionActuator &&
            i != rShoulderPitchPositionActuator) {
          phase_modified = phase;
        } else if (i == lShoulderPitchPositionActuator || i == rShoulderPitchPositionActuator) {
          phase_modified = std::max(phase, 0.5f) * 2.0f - 1.0f;
        } else {
          phase_modified = std::min(phase, 0.5f) * 2.0f;
        }
        controlledActuators[i] = actuators[i] * phase_modified + startAngles[i] * (1.f - phase_modified);

        controlledActuators[i + headYawHardnessActuator] = defaultActiveHardness;
      }
      return controlledActuators;
    }
    state = standing;

  case standing:
    // if standup succesfull, keep applying default hardness
    defaultHardness = 0.3;
    if (frameDrops <= allowedFrameDrops) {
      return actuators; // use original actuators
    }

  case preShuttingDown:
    for (int i = 0; i < lbhNumOfPositionActuatorIds; ++i) {
      startAngles[i] = lastPositionActuators[i];
      startHardness[i] = std::min<float>(lastHardnessActuators[i], defaultHardness);
    }
    state = state == preShuttingDown ? shuttingDown : sittingDown;
    phase = 0.f;

  case sittingDown:
  case shuttingDown:
    if ((phase < 1.f && frameDrops > allowedFrameDrops) || state == shuttingDown) {
      memset(controlledActuators, 0, sizeof(controlledActuators));
      if (state == shuttingDown && phase + 0.005f > 1.f) {
        // at the end of unstiff or normal shutdown drop all hardness
        defaultHardness = 0.0;
      }
      phase = std::min(phase + 0.01f, 1.f);
      if (phase >= 1.0f) {
        defaultActiveHardness = 0.0f;
      }
      for (int i = 0; i < lbhNumOfPositionActuatorIds; ++i) {
        if (i != lElbowRollPositionActuator && i != rElbowRollPositionActuator && i != lShoulderPitchPositionActuator &&
            i != rShoulderPitchPositionActuator) {
          phase_modified = phase;
        } else if (i == lShoulderPitchPositionActuator || i == rShoulderPitchPositionActuator) {
          phase_modified = std::min(phase, 0.5f) * 2.0f;
        } else {
          phase_modified = std::max(phase, 0.5f) * 2.0f - 1.0f;
        }
        controlledActuators[i] = sitDownAngles[i] * phase_modified + startAngles[i] * (1.f - phase_modified);
        controlledActuators[i + headYawHardnessActuator] = defaultActiveHardness;
        if (i == lShoulderRollPositionActuator) {
          if (startAngles[i] < 0.18) { // when less than 6 degree
            controlledActuators[i] += 0.18 * (1 - phase) * phase * 5;
          }
        }
        if (i == rShoulderRollPositionActuator) {
          if (startAngles[i] > -0.18) { // when less than 6 degree
            controlledActuators[i] -= 0.18 * (1 - phase) * phase * 5;
          }
        }
      }
      return controlledActuators;
    } else if (frameDrops <= allowedFrameDrops) {
      for (int i = 0; i < lbhNumOfPositionActuatorIds; ++i)
        startAngles[i] = lastPositionActuators[i];
      goto standingUp;
    } else {
      goto sitting;
    }
  }
  std::abort();
}

// write actuator data from shared mem to socket
void write_actuators(LolaFrameHandler& frame_handler) {
  auto& data = frame_handler.actuator_frame.data;
  auto& sonar = frame_handler.actuator_frame.sonar;

  auto& sensor_data = frame_handler.sensor_frame.data;
  // if(sensor_data[lHipYawPitchCurrentSensor] < 1e-3) {
  //   printf("%f %f %f %f\n", sensor_data[lHipYawPitchCurrentSensor], sensor_data[lHipYawPitchPositionSensor],
  //   data[lHipYawPitchPositionActuator],
  //           data[lHipYawPitchHardnessActuator]);
  // }

  // read actuators
  shared_data->readingActuators = shared_data->newestActuators;
  if (shared_data->readingActuators == lastReadingActuators) {
    if (actuatorDrops == 0)
      fprintf(stderr, "naobridge: missed actuator request.\n");
    ++actuatorDrops;
  } else
    actuatorDrops = 0;
  lastReadingActuators = shared_data->readingActuators;
  float* readingActuators = shared_data->actuators[lastReadingActuators];
  float* actuators = handle_state(frame_handler, readingActuators);

  // initialize leds
  if (state != standing && shared_data->state == okState) {
    for (int i = faceLedRedLeft0DegActuator; i < lbhNumOfActuatorIds; ++i) {
      actuators[i] = readingActuators[i];
    }
  }
  // TODO: set battery LEDs

  // set game control status LEDs
  ctrl_handle_output(actuators, lastGameCtrlData);
  // override core state LEDs
  set_override_leds(actuators);

  // FIXME: probably we can remove the checks below and write everything
  // set position actuators
  for (int i = 0; i < lbhNumOfPositionActuatorIds; ++i) {
    lastPositionActuators[i] = actuators[i];
    data[i] = actuators[i];
  }

  // set hardness actuators
  for (int j = 0; j < lbhNumOfHardnessActuatorIds; ++j) {
    lastHardnessActuators[j] = actuators[headYawHardnessActuator + j];
    data[headYawHardnessActuator + j] = actuators[headYawHardnessActuator + j];
  }

  // set sonar actuator
  sonar[0] = true;
  sonar[1] = true;

  // set leds
  for (int i = 0; i < lbhNumOfLedActuatorIds; ++i) {
    data[faceLedRedLeft0DegActuator + i] = actuators[faceLedRedLeft0DegActuator + i];
  }

  // send the actuator values
  char* buffer;
  size_t size;
  std::tie(buffer, size) = frame_handler.pack();
  sock->send(boost::asio::buffer(buffer, size));
}

int main(int, char*[]) {
  // initialize
  if (!init()) {
    fprintf(stderr, "naobridge: initialization failed!\n");
    close();
    return 1;
  }

  // register some handlers so we can clean-up when we're killed.
  signal(SIGINT, ctrlc_handler);
  signal(SIGTERM, ctrlc_handler);

  LolaFrameHandler frame_handler;
  bool shutdown = false;

  while (!shutdown || (state == preShuttingDown || (state == shuttingDown && phase + 0.005f < 1.0f))) {
    // update time
    curTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

    // read sensor data
    read_sensors(frame_handler);

    // wait for new actuator data
    usleep(DELAY_ACT_MS * 1000);

    // write actuator data
    write_actuators(frame_handler);

    // check signal
    if (nao_shutdown) {
      if (!shutdown) {
        fprintf(stderr, "naobridge: shutting down\n");
        state = preShuttingDown;
        shutdown = true;
      }
    }
  }

  // close connection
  fprintf(stderr, "naobridge: reset stiffness of all joints\n");

  // reset all joint stiffness
  constexpr int max_len = 100000;
  char data[max_len] = {'\0'};
  char* buffer;
  size_t size;
  sock->receive(boost::asio::buffer(data, max_len));
  for (int i = headYawHardnessActuator; i <= rAnkleRollHardnessActuator; ++i) {
    frame_handler.actuator_frame.data[i] = -1.f;
  }
  std::tie(buffer, size) = frame_handler.pack();
  sock->send(boost::asio::buffer(buffer, size));

  // close
  close();
  fprintf(stderr, "naobridge: bye!\n");

  return 0;
}
