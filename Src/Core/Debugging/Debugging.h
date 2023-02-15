/**
 * @file Debugging.h
 *
 * Macros and functions for debugging
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Martin Lötzsch
 */

#pragma once

#include "Core/MessageQueue/OutMessage.h"
#include "Core/Debugging/DebugRequest.h"

#include <iostream>

#include "Core/Global.h"

#ifdef RELEASE

#define EXECUTE_ONLY_IN_DEBUG(...) ((void)0)
#define OUTPUT(type, format, expression) ((void)0)
#define OUTPUT_TEXT(expression) ((void)0)
#define OUTPUT_WARNING(message) ((void)0)
#define DEBUG_RESPONSE(id, ...) ((void)0)
#define DEBUG_RESPONSE_ONCE(id, ...) ((void)0)
#define DEBUG_RESPONSE_NOT(id, ...)                                                                                         \
  { __VA_ARGS__ }
#define NOT_POLLABLE_DEBUG_RESPONSE(id, ...) ((void)0)

#else // RELEASE

/**
 * A macro which executes an expression only if in debug mode.
 */
#define EXECUTE_ONLY_IN_DEBUG(...)                                                                                          \
  { __VA_ARGS__ }

/**
 * A macro for sending debug messages.
 *
 * @param type The type of the message from the MessageID enum in MessageIDs.h
 * @param format The message format of the message (bin or text)
 * @param expression A streamable expression
 *
 * Examples:
 * <pre>
 * OUTPUT(idImage, bin, *pMyImage);
 * OUTPUT(idText, text, "MyObject:myFunction() invoked");
 * OUTPUT(idText, text, "i: " << i << ", j:" << j);
 * </pre>
 */
#define OUTPUT(type, format, expression)                                                                                    \
  do {                                                                                                                      \
    Global::getDebugOut().format << expression;                                                                             \
    Global::getDebugOut().finishMessage(type);                                                                              \
  } while (false)

#ifdef TARGET_TOOL
/**
 * A macro for sending warning messages.
 *
 * @param message A message streamable as text.
 */
#define OUTPUT_WARNING(message)                                                                                             \
  do {                                                                                                                      \
    std::cerr << "Error: " << message << std::endl;                                                                         \
  } while (false)

/**
 * Shortcut for outputting text messages.
 * @param expression A streaming expression to output.
 */
#define OUTPUT_TEXT(message)                                                                                                \
  do {                                                                                                                      \
    std::cerr << message << std::endl;                                                                                      \
  } while (false)
#else

/**
 * Shortcut for outputting text messages.
 * @param expression A streaming expression to output.
 */
#define OUTPUT_TEXT(message)                                                                                                \
  do {                                                                                                                      \
    std::cerr << message << std::endl;                                                                                      \
    OUTPUT(idText, text, message);                                                                                          \
  } while (false)

/**
 * A macro for sending warning messages.
 *
 * @param message A message streamable as text.
 */
// FIXME: investigate segfault in printing warnings
#define OUTPUT_WARNING(message)                                                                                             \
  do {                                                                                                                      \
    std::cerr << "Warning: " << message << std::endl;                                                                       \
    OUTPUT(idText, text, "Warning: " << message);                                                                           \
  } while (false)
#endif // TARGET_TOOL

/**
 * A debugging switch, allowing the enabling or disabling of expressions.
 * @param id The id of the debugging switch
 * @param ... The expression to be executed if id is enabled
 */
#define DEBUG_RESPONSE(id, ...)                                                                                             \
  do {                                                                                                                      \
    if (Global::getDebugRequestTable().poll && Global::getDebugRequestTable().notYetPolled(id)) {                           \
      OUTPUT(idDebugResponse, text, id << Global::getDebugRequestTable().isActive(id));                                     \
    }                                                                                                                       \
    if (Global::getDebugRequestTable().isActive(id)) {                                                                      \
      __VA_ARGS__                                                                                                           \
    }                                                                                                                       \
  } while (false)

/**
 * register a debug id. it can be used for conditional debug code, which is not
 * executed on the first iteration (eg in behavior).
 * later it can be used like this: DEBUG_RESPONSE(id, ...);
 * @param id The id of the debugging switch
 */
#define DEBUG_RESPONSE_REGISTER(id)                                                                                         \
  do {                                                                                                                      \
    if (Global::getDebugRequestTable().poll && Global::getDebugRequestTable().notYetPolled(id)) {                           \
      OUTPUT(idDebugResponse, text, id << Global::getDebugRequestTable().isActive(id));                                     \
    }                                                                                                                       \
  } while (false)

/**
 * A debugging switch, allowing the non-recurring execution of expressions.
 * @param id The id of the debugging switch
 * @param ... The expression to be executed if id is enabled
 */
#define DEBUG_RESPONSE_ONCE(id, ...)                                                                                        \
  do {                                                                                                                      \
    if (Global::getDebugRequestTable().poll && Global::getDebugRequestTable().notYetPolled(id)) {                           \
      OUTPUT(idDebugResponse, text, id << Global::getDebugRequestTable().isActive(id));                                     \
    }                                                                                                                       \
    if (Global::getDebugRequestTable().isActive(id)) {                                                                      \
      Global::getDebugRequestTable().disable(id);                                                                           \
      __VA_ARGS__                                                                                                           \
    }                                                                                                                       \
  } while (false)

/**
 * A debugging switch, allowing the enabling or disabling of expressions.
 * @param id The id of the debugging switch
 * @param ... The expression to be executed if id is disabled or software is compiled in release mode
 */
#define DEBUG_RESPONSE_NOT(id, ...)                                                                                         \
  do {                                                                                                                      \
    if (Global::getDebugRequestTable().poll && Global::getDebugRequestTable().notYetPolled(id)) {                           \
      OUTPUT(idDebugResponse, text, id << Global::getDebugRequestTable().isActive(id));                                     \
    }                                                                                                                       \
    if (!Global::getDebugRequestTable().isActive(id)) {                                                                     \
      __VA_ARGS__                                                                                                           \
    }                                                                                                                       \
  } while (false)

/** A debugging switch not to be polled */
#define NOT_POLLABLE_DEBUG_RESPONSE(id, ...)                                                                                \
  do {                                                                                                                      \
    if (Global::getDebugRequestTable().isActive(id)) {                                                                      \
      __VA_ARGS__                                                                                                           \
    }                                                                                                                       \
  } while (false)

#endif // RELEASE

#ifdef TARGET_TOOL
#define OUTPUT_ERROR(message)                                                                                               \
  do {                                                                                                                      \
    std::cerr << "Error: " << message << std::endl;                                                                         \
  } while (false)
#else
/**
 * A macro for sending error messages.
 * Note that this macro is enabled regardless of which
 * target is used to build.
 *
 * @param message A message streamable as text.
 */
#define OUTPUT_ERROR(message)                                                                                               \
  do {                                                                                                                      \
    OUTPUT(idText, text, "Error: " << message);                                                                             \
    OutTextSize _size;                                                                                                      \
    _size << "Error: " << message;                                                                                          \
    char* _buf = new char[_size.getSize() + 1];                                                                             \
    OutTextMemory _steam(_buf);                                                                                             \
    _steam << "Error: " << message;                                                                                         \
    _buf[_size.getSize()] = 0;                                                                                              \
    DebugRequestTable::print(_buf);                                                                                         \
    delete[] _buf;                                                                                                          \
  } while (false)
#endif //#endif // TARGET_TOOL
