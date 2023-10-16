/**
 * @file Directory.cpp
 *
 * Implements a class for accessing directories.
 * Yeah, this is the POSIX implementation.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include <cerrno>
#include <fnmatch.h>

#include "Directory.h"
#include "BHAssert.h"

Directory::Directory() : dp(0) {}

Directory::~Directory() {
  if (dp) {
    closedir((DIR*)dp);
  }
}

bool Directory::open(const std::string& pattern) {
  if (dp) {
    closedir((DIR*)dp);
    dp = 0;
  }

  size_t end = pattern.find_last_of("/\\");
  if (end == std::string::npos) {
    return false;
  }
  filepattern = pattern.substr(end + 1);
  dirname = pattern.substr(0, end);

  dp = opendir(dirname.c_str());
  return dp != 0;
}

bool Directory::read(std::string& name, bool& isDir) {
  if (!dp) {
    return false;
  }

  for (;;) {
    struct dirent* dent = readdir((DIR*)dp);
    if (!dent) {
      closedir((DIR*)dp);
      dp = 0;
      return false;
    }
    if (fnmatch(filepattern.c_str(), dent->d_name, 0) == 0) {
      name = dirname + "/" + dent->d_name;
      isDir = false;
      struct stat buff;
      if (stat(name.c_str(), &buff) == 0) {
        if (S_ISDIR(buff.st_mode)) {
          isDir = true;
        }
      }
      return true;
    }
  }
  return false; // unreachable
}
