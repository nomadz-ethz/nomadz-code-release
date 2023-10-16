/**
 * @file UniversalResourceCompiler.cpp
 *
 * Is used to compile the motion net for the special actions
 * and for generating a variety of other files.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in License.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are Uwe Düffert and Martin Lötzsch
 */

#include <cstdio>

#include "URC/MofCompiler.h"

int main(int argc, char* argv[]) {
  char buffer[1000];
  if (compileMofs(buffer, sizeof(buffer)))
    printf("Created 'Config/specialActions.dat' successfully\n");
  else
    printf("%s", buffer);
}
