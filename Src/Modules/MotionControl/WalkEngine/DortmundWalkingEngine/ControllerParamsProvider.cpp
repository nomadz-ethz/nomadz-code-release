/**
 * @file ControllerParamsProvider.cpp
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 */

#include "ControllerParamsProvider.h"
#include "Core/Settings.h"
#include "Core/System/File.h"
#include <cstdlib>

ControllerParamsProvider::ControllerParamsProvider() {
  load();
}

void ControllerParamsProvider::update(ControllerParams& controllerParams) {
  controllerParams = contParams;
}

int ControllerParamsProvider::load() {
  FILE* stream;

  std::list<std::string> names = File::getFullNames(paramFileName);
  bool foundFile = false;
  for (auto& path : names) {
    stream = fopen(path.c_str(), "r");
    if (stream) {
      foundFile = true;
      break;
    }
  }
  ASSERT(foundFile);

  char tempstr[200], *stopstring;

  fscanf(stream, "%s", tempstr);
  contParams.z_h = strtof(tempstr, &stopstring);

  fscanf(stream, "%s", tempstr);
  contParams.dt = strtof(tempstr, &stopstring);

  fscanf(stream, "%s", tempstr);
  ASSERT(contParams.N == (int)strtof(tempstr, &stopstring));

  fscanf(stream, "%s", tempstr);
  contParams.L[0][0] = strtof(tempstr, &stopstring);
  fscanf(stream, "%s", tempstr);
  contParams.L[0][1] = strtof(tempstr, &stopstring);
  fscanf(stream, "%s", tempstr);
  contParams.L[0][2] = strtof(tempstr, &stopstring);

  fscanf(stream, "%s", tempstr);
  contParams.L[1][0] = strtof(tempstr, &stopstring);
  fscanf(stream, "%s", tempstr);
  contParams.L[1][1] = strtof(tempstr, &stopstring);
  fscanf(stream, "%s", tempstr);
  contParams.L[1][2] = strtof(tempstr, &stopstring);

  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      fscanf(stream, "%s", tempstr);
      contParams.A0[col][row] = strtof(tempstr, &stopstring);
    }
  }

  fscanf(stream, "%s", tempstr);
  contParams.Gi = strtof(tempstr, &stopstring);

  for (int i = 0; i < 3; i++) {
    fscanf(stream, "%s", tempstr);
    contParams.Gx[i][0] = strtof(tempstr, &stopstring);
  }

  for (int i = 0; i < 3; i++) {
    fscanf(stream, "%s", tempstr);
    contParams.b0[i] = strtof(tempstr, &stopstring);
  }

  for (int i = 0; i < 3; i++) {
    fscanf(stream, "%s", tempstr);
    contParams.c0[i][0] = strtof(tempstr, &stopstring);
  }

  contParams.Gd = Matrix<50, 1, float>();

  for (int i = 0; i < contParams.N; i++) {
    fscanf(stream, "%s", tempstr);
    contParams.Gd[0][i] = strtof(tempstr, &stopstring);
  }

  for (int i = 1; i < contParams.N; i++) {
    fscanf(stream, "%s", tempstr);
    contParams.Ge[i][0] = strtof(tempstr, &stopstring);
    fscanf(stream, "%s", tempstr);
    contParams.Ge[i][1] = strtof(tempstr, &stopstring);
    fscanf(stream, "%s", tempstr);
    contParams.Ge[i][2] = strtof(tempstr, &stopstring);
  }

  fclose(stream);

  return 1;
}
MAKE_MODULE(ControllerParamsProvider, dortmundWalkingEngine)
