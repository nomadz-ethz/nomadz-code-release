/**
 * @file Directory.h
 *
 * Declares a platform dependend class for accessing directories.
 * Yeah, this is the POSIX implementation.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#pragma once

#include <string>

/**
 * A Class for accessing directories.
 */
class Directory {
public:
  /**
   * Default constructor.
   */
  Directory();

  /**
   * Destructor.
   */
  ~Directory();

  /**
   * Opens a directory for searching files.
   * \param pattern A search pattern like "/etc/a*.ini"
   * \return Whether the directory was opened successfully.
   */
  bool open(const std::string& pattern);

  /**
   * Searches the next matching entry in the opened directory.
   * \param name The name of the next matching entry.
   * \param isDir Whether the next entry is a directory.
   * \return true when a matching entry was found.
   */
  bool read(std::string& name, bool& isDir);

private:
  void* dp;                /**< Directory descriptor. */
  std::string dirname;     /**< The name of the directory. */
  std::string filepattern; /**< The pattern for file name matching (e.g. "*.dll"). */
};
