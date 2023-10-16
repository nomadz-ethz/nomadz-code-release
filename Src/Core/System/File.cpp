/**
 * @file File.cpp
 *
 * Declaration of class File for Win32 and Linux.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 */

#include <cstdlib> // for getenv
#include <sys/stat.h>
#include <unistd.h>

#include <cstring>
#include <cstdio>
#include <cstdarg>

#include "BHAssert.h"
#include "File.h"

#include "Core/Global.h"
#include "Core/Settings.h"

File::File(const std::string& name, const char* mode, bool tryAlternatives) : stream(0) {
  std::list<std::string> names = getFullNames(name);
  if (tryAlternatives) {
    for (std::list<std::string>::const_iterator i = names.begin(); !stream && i != names.end(); ++i) {
      stream = fopen(i->c_str(), mode);
    }
  } else {
    stream = fopen(names.back().c_str(), mode);
  }
}

std::list<std::string> File::getFullNames(const std::string& rawName) {
#if (defined(TARGET_SIM) || defined(TARGET_TOOL))
  std::string name(rawName);
  for (int i = name.length(); i >= 0; i--) {
    if (name[i] == '\\') {
      name[i] = '/';
    }
  }
#else
  const std::string& name(rawName);
#endif

  std::list<std::string> names;
  if (name[0] != '.' && !isAbsolute(name.c_str())) // given path is relative to getBHDir()
  {
    if (Global::settingsExist()) {
      names.push_back(std::string(getBHDir()) + "/Config/Robots/" + Global::getSettings().robot + "/" + name);
      names.push_back(std::string(getBHDir()) + "/Config/Robots/Default/" + name);
      names.push_back(std::string(getBHDir()) + "/Config/Locations/" + Global::getSettings().location + "/" + name);
#ifdef ROBOT_V6
      names.push_back(std::string(getBHDir()) + "/Config/Locations/" + Global::getSettings().location + "/V6/" + name);
#endif
      names.push_back(std::string(getBHDir()) + "/Config/Perception/" + name);
      if (Global::getSettings().location != "Default") {
        names.push_back(std::string(getBHDir()) + "/Config/Locations/Default/" + name);
      }
    }
    names.push_back(std::string(getBHDir()) + "/Config/" + name);
  } else {
    names.push_back(name);
  }
  return names;
}

std::string File::getFullName(const std::string& name) {
  std::list<std::string> files = getFullNames(name);
  for (std::list<std::string>::iterator iter = files.begin(); iter != files.end(); ++iter) {
    File file(*iter, "rb", false);
    if (file.exists()) {
      return *iter;
    }
  }
  return "";
}

File::~File() {
  if (stream != 0) {
    fclose((FILE*)stream);
  }
}

void File::read(void* p, unsigned size) {
  VERIFY(!eof());
  VERIFY(fread(p, 1, size, (FILE*)stream) > 0);
}

char* File::readLine(char* p, unsigned size) {
  VERIFY(!eof());
  return fgets(p, size, (FILE*)stream);
}

void File::write(const void* p, unsigned size) {
  // if opening failed, stream will be 0 and fwrite would crash
  ASSERT(stream != 0);
  VERIFY(fwrite(p, 1, size, (FILE*)stream) == size);
}

void File::printf(const char* format, ...) {
  va_list args;
  va_start(args, format);
  vfprintf((FILE*)stream, format, args);
  va_end(args);
}

bool File::eof() {
  // never use feof(stream), because it informs you only after reading
  // to far and is never reset, e.g. if the stream grows afterwards
  // our implementation can handle both correctly:
  if (!stream) {
    return false;
  } else {
    int c = fgetc((FILE*)stream);
    if (c == EOF) {
      return true;
    } else {
      VERIFY(ungetc(c, (FILE*)stream) != EOF);
      return false;
    }
  }
}

unsigned File::getSize() {
  if (!stream) {
    return 0;
  }
  long currentPos = ftell((FILE*)stream);
  ASSERT(currentPos >= 0);
  VERIFY(fseek((FILE*)stream, 0, SEEK_END) == 0);
  long size = ftell((FILE*)stream);
  VERIFY(fseek((FILE*)stream, currentPos, SEEK_SET) == 0);
  return (unsigned)size;
}

const char* File::getBHDir() {
#ifdef BH_DIR
  return BH_DIR;
#else
  static char dir[FILENAME_MAX] = {0};
  if (!dir[0]) {
#if defined(TARGET_SIM) || defined(TARGET_TOOL)
    VERIFY(getcwd(dir, sizeof(dir) - 7) != 0);
    char* end = dir + strlen(dir) - 1;
    struct stat buff;
    while (true) {
      if (end < dir || *end == '/') {
        strcpy(end + 1, "Config");
        if (stat(dir, &buff) == 0)
          if (S_ISDIR(buff.st_mode)) {
            end[end > dir ? 0 : 1] = '\0';
            return dir;
          }
      }
      if (end < dir)
        break;
      end--;
    }
#endif
    strcpy(dir, ".");
  }
  return dir;
#endif
}

bool File::isAbsolute(const char* path) {
  return (path[0] && path[1] == ':') || path[0] == '/' || path[0] == '\\';
}

bool File::isExplicitRelative(const char* path) {
  return path[0] == '.' && (path[1] == '/' || path[1] == '\\');
}
