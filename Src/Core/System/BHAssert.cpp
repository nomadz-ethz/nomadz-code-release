/**
 * @file BHAssert.cpp
 *
 * Some helper functions for low level debugging
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#ifndef NDEBUG

#include "BHAssert.h"

#include <cstdlib>
#include <cstdio>
#include <cstdarg>

void Assert::print(const char* file, int line, const char* format, ...) {
  char data[320];
  int length = snprintf(data, sizeof(data) - 2, "%s:%d: ", file, line);
  if (length < 0) {
    length = sizeof(data) - 2;
  }
  va_list ap;
  va_start(ap, format);
  int i = vsnprintf(data + length, sizeof(data) - length - 2, format, ap);
  if (i < 0) {
    length = sizeof(data) - 2;
  } else {
    length += i;
  }
  va_end(ap);
  data[length++] = '\n';
  data[length] = '\0';
  fputs(data, stderr);
  fflush(stderr);
}

void Assert::abort() {
  ::abort();
}

#endif // NDEBUG

#ifdef TARGET_ROBOT

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <sstream>
#include <ctime>
#include <iostream>
#include <mutex>
#include "BHAssert.h"

class AssertFramework {
public:
  struct Line {
    char file[128];
    int line;
    char message[128];
  };

  struct Track {
    Line line[16];
    int currentLine;
    bool active;
  };

  struct Thread {
    char name[32];
    Track track[2];
  };

  struct Data {
    Thread thread[2];
    int currentThread;
  };

  static std::mutex mutex;
  static __thread Thread* threadData;

  int fd;
  Data* data;

  AssertFramework() : fd(-1), data((Data*)MAP_FAILED) {}

  ~AssertFramework() {
    if (data != MAP_FAILED)
      munmap(data, sizeof(Data));
    if (fd != -1)
      close(fd);
  }

  bool init(bool reset) {
    if (data != MAP_FAILED)
      return true;

    fd = shm_open("/bhuman_assert", O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
    if (fd == -1)
      return false;

    if (ftruncate(fd, sizeof(Data)) == -1 ||
        (data = (Data*)mmap(NULL, sizeof(Data), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)) == MAP_FAILED) {
      close(fd);
      fd = -1;
      return false;
    }

    if (reset)
      memset(data, 0, sizeof(Data));

    return true;
  }

} assertFramework;

std::mutex AssertFramework::mutex;
__thread AssertFramework::Thread* AssertFramework::threadData = 0;

bool Assert::logInit(const char* name) {
  int thread = -1;
  AssertFramework::mutex.lock();
  if (!assertFramework.init(true)) {
    std::cerr << "Failed to init assert framework" << std::endl;
    std::abort();
  }
  if (assertFramework.data != MAP_FAILED) {
    thread = assertFramework.data->currentThread++;
  } else {
    std::cerr << "Failed to map thread data for assert" << std::endl;
    std::abort();
  }
  AssertFramework::mutex.unlock();
  ASSERT(thread >= 0 && thread < int(sizeof(assertFramework.data->thread) / sizeof(*assertFramework.data->thread)));
  AssertFramework::threadData = &assertFramework.data->thread[thread];
  memccpy(AssertFramework::threadData->name, name, 0, sizeof(AssertFramework::threadData->name) - 1);
  AssertFramework::threadData->name[sizeof(AssertFramework::threadData->name) - 1] = 0;
  return true;
}

void Assert::logAdd(int trackId, const char* file, int lineNum, const char* message) {
  if (!AssertFramework::threadData) {
    std::cerr << "Failed to add to log: no thread data!" << std::endl;
    return;
  }
  if (!(trackId >= 0 &&
        trackId < int(sizeof(AssertFramework::threadData->track) / sizeof(*AssertFramework::threadData->track)))) {
    std::cerr << "Failed to add to log: invalid thread data!" << std::endl;
    return;
  }
  AssertFramework::Track* track = &AssertFramework::threadData->track[trackId];
  AssertFramework::Line* line =
    &track->line[track->currentLine = (track->currentLine + 1) % (sizeof(track->line) / sizeof(*track->line))];
  memccpy(line->file, file, 0, sizeof(line->file) - 1);
  line->file[sizeof(line->file) - 1] = 0;
  line->line = lineNum;
  memccpy(line->message, message, 0, sizeof(line->message) - 1);
  line->message[sizeof(line->message) - 1] = 0;
  track->active = true;
}

// Get current date/time, format is YYYY-MM-DD_HH:mm:ss
const std::string currentTimeStamp() {
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%F_%T", &tstruct);
  return buf;
}

void Assert::logDump(bool toStderr, int termSignal) {
  assertFramework.init(false);
  std::stringstream ssfilename;
  ssfilename << "/tmp/bhuman.assert_" << currentTimeStamp() << ".log";

  FILE* fp = toStderr ? stderr : fopen(ssfilename.str().c_str(), "w");
  if (fp == 0)
    return;

  for (int i = 0; i < int(sizeof(assertFramework.data->thread) / sizeof(*assertFramework.data->thread)); ++i) {
    AssertFramework::Thread* thread = &assertFramework.data->thread[i];
    if (!*thread->name)
      continue;

    for (int i = 0; i < int(sizeof(thread->track) / sizeof(*thread->track)); ++i) {
      AssertFramework::Track* track = &thread->track[i];
      if (!track->active)
        continue;
      fprintf(fp, "---- %s %s ----\n[...]\n", thread->name, i == 0 ? "BH_TRACE" : "ASSERT, VERIFY, TRACE");
      for (int i = 0; i < int(sizeof(track->line) / sizeof(*track->line)); ++i) {
        int j = track->currentLine - (int(sizeof(track->line) / sizeof(*track->line)) - 1 - i);
        if (j < 0)
          j += int(sizeof(track->line) / sizeof(*track->line));
        AssertFramework::Line* line = &track->line[j];
        if (!*line->file)
          continue;
        fprintf(fp, "%s:%d: %s\n", line->file, line->line, line->message);
      }
    }
  }

  const char* termSignalNames[] = {"",
                                   "",
                                   "sigINT",
                                   "sigQUIT",
                                   "sigILL",
                                   "",
                                   "sigABRT",
                                   "",
                                   "sigFPE",
                                   "sigKILL",
                                   "",
                                   "sigSEGV",
                                   "",
                                   "sigPIPE",
                                   "sigALRM",
                                   "sigTERM"};

  fprintf(fp, "----\n");
  const char* termSignalName = termSignal < 0 || termSignal >= int(sizeof(termSignalNames) / sizeof(*termSignalNames))
                                 ? ""
                                 : termSignalNames[termSignal];
  if (*termSignalName)
    fprintf(fp, "%s\n", termSignalName);

  if (fp != stderr)
    fclose(fp);
}

#endif // TARGET_ROBOT
