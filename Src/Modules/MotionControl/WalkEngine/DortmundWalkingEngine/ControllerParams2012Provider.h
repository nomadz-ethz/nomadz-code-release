/**
 * @file ControllerParams2012Provider.h
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#pragma once

#include "Representations/MotionControl/ControllerParams.h"
#include "Core/Module/Module.h"

MODULE(ControllerParams2012Provider)
PROVIDES_WITH_MODIFY(ControllerParams)
DEFINES_PARAMETER(std::string, paramFileName, "ZMPIPController2012.dat")
END_MODULE

class ControllerParams2012Provider : public ControllerParams2012ProviderBase {
public:
  ControllerParams2012Provider();

protected:
  void update(ControllerParams& controllerParams);
  int load();
  ControllerParams contParams;
};
