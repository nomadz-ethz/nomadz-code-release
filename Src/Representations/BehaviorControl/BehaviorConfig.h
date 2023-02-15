/**
 * @file BehaviorConfig.h
 *
 * The file declares a class that contains a per-robot behavior configuration
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once
#include "Core/Streams/AutoStreamable.h"
#include "BehaviorStatus.h"

/**
 * @class BehaviorStatus
 * A class that containts data about the initial behavior state.
 */
#include "Core/Streams/FieldWrapper.h"

STREAMABLE_DECLARE_LEGACY(BehaviorConfig)
STREAMABLE_LEGACY(BehaviorConfig,
{
public:
    static const char* getName(BehaviorStatus::Role e) { return BehaviorStatus::getName(e);
}
, (BehaviorStatus::Role)(BehaviorStatus::striker)role,
});
