/**
 * @file Enum.h
 *
 * Defines a macro that declares an enum and provides
 * a function to access the names of its elements.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Thomas Röfer
 */

#pragma once

/**
 * @param enums A string that contains a comma-separated list of enum
 *              elements. It is allowed that an element is initialized
 *              with the value of its predecessor. Any other
 *              initializations are forbidden.
 *              "a, b, numOfLettersBeforeC, c = numOfLettersBeforeC, d"
 *              would be a legal parameter.
 * @param names An array of string pointers that will be filled with
 *              the names of the enums.
 * @param numOfEnums The number of enums in the string. Reassignments do
 *                   not count, i.e. in the example above, this
 *                   parameter had to be 4.
 */
void enumInit(char* enums, const char** names, int numOfEnums);

/**
 * Defining an enum and a function getName(<Enum>) that can return
 * the name of each enum element. The enum will automatically
 * contain an element numOf<Enum>s that reflects the number of
 * elements defined.
 */
#define ENUM(Enum, ...)                                                                                                     \
  enum Enum : unsigned char { __VA_ARGS__, numOf##Enum##s };                                                                \
  inline static const char* getName(Enum e) {                                                                               \
    static char enums[] = #__VA_ARGS__;                                                                                     \
    static const char* names[numOf##Enum##s];                                                                               \
    static bool init = true;                                                                                                \
    if (init) {                                                                                                             \
      enumInit(enums, names, numOf##Enum##s);                                                                               \
      init = false;                                                                                                         \
    }                                                                                                                       \
    return e >= numOf##Enum##s ? 0 : names[e];                                                                              \
  }

// HACK to integrate aliased enums in ROS streamables
#define ENUM_ALIAS(Base, Enum, ...)                                                                                         \
  inline static const char* getName(Base::Enum e) {                                                                         \
    static char enums[] = #__VA_ARGS__;                                                                                     \
    static const char* names[Base::numOf##Enum##s];                                                                         \
    static bool init = true;                                                                                                \
    if (init) {                                                                                                             \
      enumInit(enums, names, Base::numOf##Enum##s);                                                                         \
      init = false;                                                                                                         \
    }                                                                                                                       \
    return e >= Base::numOf##Enum##s ? 0 : names[e];                                                                        \
  }
