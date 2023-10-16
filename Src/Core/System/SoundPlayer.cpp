/**
 * @file SoundPlayer.cpp
 *
 * Implementation of class SoundPlayer.
 * attention this is the Linux implementation
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#include <csignal>
#include <sys/types.h>
#include <sys/wait.h>
#include <cstdlib>
#include <cstdio>

#include "SoundPlayer.h"
#include "File.h"

SoundPlayer SoundPlayer::soundPlayer;

SoundPlayer::SoundPlayer() : Thread<SoundPlayer>(), started(false), closing(false) {}

SoundPlayer::~SoundPlayer() {
  if (started) {
    closing = true;
    sem.post();
  }
}

void SoundPlayer::start() {
  Thread<SoundPlayer>::start(this, &SoundPlayer::main);
}

void SoundPlayer::main() {
  while (isRunning() && !closing) {
    flush();
    VERIFY(sem.wait());
  }
}

void SoundPlayer::playDirect(const std::string& basename) {
  std::string fileName(filePrefix);
  fileName += basename;

  int r = vfork();
  if (r == -1) {
    perror("SoundPlayer: fork() failed");
  } else if (r != 0) // parent
  {
    int status;
    waitpid(r, &status, 0);
  } else // child
  {
    if (execlp("aplay", "aplay", "-q", fileName.c_str(), (char*)0) == -1) {
      perror("SoundPlayer: exec failed");
    }
  }
}

void SoundPlayer::flush() {
  for (;;) {
    std::string first;
    {
      SYNC;
      if (0 == queue.size()) {
        break;
      }
      first = queue.front();
      queue.pop_front();
    }

    playDirect(first);
  }
}

int SoundPlayer::play(const std::string& name) {
  int queuelen;

  {
    SYNC_WITH(soundPlayer);
    if (soundPlayer.queue.empty() || soundPlayer.queue.back() != name) {
      soundPlayer.queue.push_back(name.c_str()); // avoid copy-on-write
    }
    queuelen = soundPlayer.queue.size();
    if (!soundPlayer.started) {
      soundPlayer.started = true;
      soundPlayer.filePrefix = File::getBHDir();
      soundPlayer.filePrefix += "/Config/Sounds/";
      soundPlayer.start();
    } else {
      soundPlayer.sem.post();
    }
  }
  return queuelen;
}
