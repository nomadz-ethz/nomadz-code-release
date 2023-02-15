/**
 * @file VisualRefereeReact.h
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include <iostream>
#include <vector>

// for testing don't play sound
bool play_sound = true;

option(VisualRefereeReact, const int& signal) {

  initial_state(initial) {
    transition {
      if (signal == RefereePercept::kickInRedTeam) {
        goto kickInRedTeam;
      } else if (signal == RefereePercept::kickInBlueTeam) {
        goto kickInBlueTeam;
      } else if (signal == RefereePercept::goalKickRedTeam) {
        goto goalKickRedTeam;
      } else if (signal == RefereePercept::goalKickBlueTeam) {
        goto goalKickBlueTeam;
      } else if (signal == RefereePercept::cornerKickRedTeam) {
        goto cornerKickRedTeam;
      } else if (signal == RefereePercept::cornerKickBlueTeam) {
        goto cornerKickBlueTeam;
      } else if (signal == RefereePercept::goalRedTeam) {
        goto goalRedTeam;
      } else if (signal == RefereePercept::goalBlueTeam) {
        goto goalBlueTeam;
      } else if (signal == RefereePercept::pushingFreeKickRedTeam) {
        goto pushingFreeKickRedTeam;
      } else if (signal == RefereePercept::pushingFreeKickBlueTeam) {
        goto pushingFreeKickBlueTeam;
      } else if (signal == RefereePercept::fullTime) {
        goto fullTime;
      }
    }
  }

  state(kickInRedTeam) {
    transition {
      if (action_done) {
        goto delay;
      }
    }
    action {
      // Red team: Robot extend RIGHT arm
      SpecialAction(SpecialActionRequest::Ref_KickInRight);

      if (play_sound) {
        PlaySound("kickIn.wav");
        PlaySound("redTeam.wav");
      }
    }
  }

  state(kickInBlueTeam) {
    transition {
      if (action_done) {
        goto delay;
      }
    }
    action {
      // Blue team: Robot extend LEFT arm
      SpecialAction(SpecialActionRequest::Ref_KickInLeft);

      if (play_sound) {
        PlaySound("kickIn.wav");
        PlaySound("blueTeam.wav");
      }
    }
  }

  state(goalKickRedTeam) {
    transition {
      if (action_done) {
        goto delay;
      }
    }
    action {
      // Red team: Right arm
      SpecialAction(SpecialActionRequest::Ref_GoalKickRight);

      if (play_sound) {
        PlaySound("goalKick.wav");
        PlaySound("redTeam.wav");
      }
    }
  }

  state(goalKickBlueTeam) {
    transition {
      if (action_done) {
        goto delay;
      }
    }
    action {
      // Blue team: Left arm
      SpecialAction(SpecialActionRequest::Ref_GoalKickLeft);

      if (play_sound) {
        PlaySound("goalKick.wav");
        PlaySound("blueTeam.wav");
      }
    }
  }

  state(cornerKickRedTeam) {
    transition {
      if (action_done) {
        goto delay;
      }
    }
    action {
      // Red team: Right arm
      SpecialAction(SpecialActionRequest::Ref_CornerKickRight);

      if (play_sound) {
        PlaySound("cornerKick.wav");
        PlaySound("redTeam.wav");
      }
    }
  }

  state(cornerKickBlueTeam) {
    transition {
      if (action_done) {
        goto delay;
      }
    }
    action {
      // Blue team: Left arm
      SpecialAction(SpecialActionRequest::Ref_CornerKickLeft);

      if (play_sound) {
        PlaySound("cornerKick.wav");
        PlaySound("blueTeam.wav");
      }
    }
  }

  state(goalRedTeam) {
    transition {
      if (action_done) {
        goto delay;
      }
    }
    action {
      // Red team: Right arm
      SpecialAction(SpecialActionRequest::Ref_GoalRight);

      if (play_sound) {
        PlaySound("goal.wav");
        PlaySound("redTeam.wav");
      }
    }
  }

  state(goalBlueTeam) {
    transition {
      if (action_done) {
        goto delay;
      }
    }
    action {
      // Blue team: Left arm
      SpecialAction(SpecialActionRequest::Ref_GoalLeft);

      if (play_sound) {
        PlaySound("goal.wav");
        PlaySound("blueTeam.wav");
      }
    }
  }

  state(pushingFreeKickRedTeam) {
    transition {
      if (action_done) {
        goto delay;
      }
    }
    action {
      // Red team: Right arm
      SpecialAction(SpecialActionRequest::Ref_PushingFreeKickRight);

      if (play_sound) {
        PlaySound("pushingFreekick.wav");
        PlaySound("redTeam.wav");
      }
    }
  }

  state(pushingFreeKickBlueTeam) {
    transition {
      if (action_done) {
        goto delay;
      }
    }
    action {
      // Blue team: Left arm
      SpecialAction(SpecialActionRequest::Ref_PushingFreeKickLeft);

      if (play_sound) {
        PlaySound("pushingFreekick.wav");
        PlaySound("blueTeam.wav");
      }
    }
  }

  state(fullTime) {
    transition {
      if (action_done) {
        goto delay;
      }
    }
    action {
      // No team
      SpecialAction(SpecialActionRequest::Ref_FullTime);

      if (play_sound) {
        PlaySound("fullTime.wav");
      }
    }
  }

  state(delay) {
    transition {
      if (state_time > 2000) {
        // repeatedly announce signal until front head button pressed
        goto initial;
      }
    }
  }
}
