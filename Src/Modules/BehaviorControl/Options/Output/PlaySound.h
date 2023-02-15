/**
 * @file PlaySound.h
 *
 * Plays a sound if this option was not called in the previous cycle.
 * @param name The name of the sound file.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(PlaySound, const std::string& name) {
  initial_state(playSound) {
    transition {
      if (state_time) {
        goto waitForNewSound;
      }
    }
    action {
      SystemCall::playSound(name.c_str());
      lastSoundPlayed = name;
    }
  }

  target_state(waitForNewSound) {
    transition {
      if (name != lastSoundPlayed) {
        goto playSound;
      }
    }
  }
}

std::string lastSoundPlayed;
