/**
 * @file lola_bridge.h
 *
 * Declaration of shared data between framework and core
 *
 * This file is subject to the terms of the BHuman License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#ifndef LOLA_BRIDGE_H
#define LOLA_BRIDGE_H

#define LLB_SOCKET "/tmp/robocup"

#include <semaphore.h>

#include <boost/asio.hpp>
#include <msgpack.hpp>

#include "lola_frame.h"
#include "naobridge/nomadz.h"

// time
extern long long curTime;
extern LBHData* shared_data;

// control functions
int main(int, char*[]);
bool init();
void close();

// sensor functions
void read_sensors(LolaFrameHandler& frame_handler);
void handle_core_state(float* sensors);

// actuator functions
void set_override_leds(float* actuators);
float* handle_state(LolaFrameHandler& frame_handler, float* actuators);
void write_actuators(LolaFrameHandler& frame_handler);

#endif
