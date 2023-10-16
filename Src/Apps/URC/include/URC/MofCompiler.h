/**
 * @file MofCompiler.h
 *
 * This file declares a single function to compile the motion net for special actions.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in License.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are Uwe Düffert and Martin Lötzsch
 */

#pragma once

#include <cstddef>

/**
 * The function compiles all mofs.
 * @param buffer A buffer that receives any error message output. It may contain
 *               several lines separated by \n.
 * @param size The length of the buffer.
 * @return Success of compilation.
 */
bool compileMofs(char* buffer, size_t size);
