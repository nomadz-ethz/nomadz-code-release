/**
 * @file Joystick.h
 *
 * Inclusion of platform dependend joystick interface implementation.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#pragma once

#ifdef WIN32
#include "Win32/Joystick.h"
#define Joystick_H
#endif

#ifdef LINUX
#include "Linux/Joystick.h"
#define Joystick_H
#endif

#ifdef MACOSX
#include "MacOS/Joystick.h"
#define Joystick_H
#endif

#ifndef Joystick_H
#error "Unknown platform"
#endif
