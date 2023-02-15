/**
 * @file DrawingOptions.h
 *
 * DrawingOptions offer a stramable class that enable or disable all the different plot on the worldstate
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"

STREAMABLE(DrawingOptions,
           {
             ,
             (bool)(true)AllRobotNumbers,
             (bool)(true)AllPoints,
             (bool)(true)PossiblePoints,
             (bool)(true)Shadows,
             (bool)(true)SelectedPoints,
             (bool)(true)AllSelectedPoints,
             (bool)(true)PassPoint,
             (bool)(false)PassZones,
             (bool)(true)VisualizeLegend,
             (bool)(false)SetAllFalse,
           });
