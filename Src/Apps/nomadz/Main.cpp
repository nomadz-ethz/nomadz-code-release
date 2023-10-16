/**
 * @file Main.cpp
 *
 * Implementation of the main() function for starting and stopping the module framework.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in License.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original author is Colin Graf
 */

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/types.h>
#include <sys/file.h> // flock
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#ifdef ENABLE_ROS
#include <rclcpp/rclcpp.hpp>
#endif

#include "Robot.h"
#include "Nao/NaoBody.h"
#include "Core/Global.h"
#include "Core/Settings.h"

#include "naobridge/nomadz.h"

static pid_t bhumanPid = 0;
static Robot* robot = 0;
static bool run = true;

static void nomadzStart(int argc, char* argv[]) {
  fprintf(stderr, "NomadZ: Start\n");

#ifdef ENABLE_ROS
  rclcpp::init(argc, argv);
  fprintf(stderr, "NomadZ: ROS2 initialized\n");
#endif

  robot = new Robot();
  robot->start();
}

static void nomadzStop() {
#ifdef ENABLE_ROS
  rclcpp::shutdown();
  fprintf(stderr, "NomadZ: ROS2 shutdown\n");
#endif

  fprintf(stderr, "NomadZ: Stop\n");
  robot->announceStop();
  robot->stop();
  delete robot;
  robot = 0;
}

static void sighandlerShutdown(int sig) {
  if (run)
    printf("Caught signal %i\nShutting down...\n", sig);
  run = false;
}

static void sighandlerRedirect(int sig) {
  // if(bhumanPid != 0)
  // kill(bhumanPid, sig);
  run = false;
}

int main(int argc, char* argv[]) {
  bool recover = false;
  {
    // parse command-line arguments
    bool background = false;
    bool watchdog = false;
    const char* bhDir = "/home/nao";

    for (int i = 1; i < argc; ++i)
      if (!strcmp(argv[i], "-b"))
        background = true;
      else if (!strcmp(argv[i], "-w"))
        watchdog = true;
      else if (!strcmp(argv[i], "-c") && i + 1 < argc)
        bhDir = argv[++i];
      else {
        fprintf(stderr,
                "Usage: %s [-b] [-c <dir>] [-w]\n\
    -b            run in background (as daemon)\n\
    -c <dir>      used gt directory (default is /home/nao)\n\
    -w            use a watchdog for crash recovery and creating trace dumps\n",
                argv[0]);
        exit(EXIT_FAILURE);
      }

    // avoid duplicated instances
    int fd = open("/tmp/nomadz", O_CREAT, 0600);
    if (fd == -1 || flock(fd, LOCK_EX | LOCK_NB) == -1) {
      fprintf(stderr, "There is already an instance of this process!\n");
      exit(EXIT_FAILURE);
    }

    // start as deamon
    if (background) {
      fprintf(stderr, "Starting as daemon...\n");
      pid_t childPid = fork();
      if (childPid == -1)
        exit(EXIT_FAILURE);
      if (childPid != 0)
        exit(EXIT_SUCCESS);
    }

    // change working directory
    if (*bhDir && chdir(bhDir) != 0) {
      fprintf(stderr, "chdir to config directory failed!\n");
      exit(EXIT_FAILURE);
    }

    // the watchdog
    if (watchdog) {
      for (;;) {
        // create pipe for logging
        int stdoutPipe[2];
        int stderrPipe[2];
        bool pipeReady = true;

        if (pipe(stdoutPipe) == -1 || pipe(stderrPipe) == -1) {
          fprintf(stderr, "NomadZ: Error while creating pipes for logging. All logs will be printed on console only! \n");
          pipeReady = false;
        }

        bhumanPid = fork();
        if (bhumanPid == -1)
          exit(EXIT_FAILURE);
        if (bhumanPid != 0) {
          int status;
          signal(SIGTERM, sighandlerRedirect);
          signal(SIGINT, sighandlerRedirect);
          signal(SIGPIPE, SIG_IGN);
          if (waitpid(bhumanPid, &status, 0) != bhumanPid) {
            exit(EXIT_FAILURE);
          }
          signal(SIGTERM, SIG_DFL);
          signal(SIGINT, SIG_DFL);
          signal(SIGPIPE, SIG_DFL);

          if (pipeReady) {
            // close unused write end
            close(stdoutPipe[1]);
            close(stderrPipe[1]);

            dup2(STDOUT_FILENO, stdoutPipe[0]); // redirect out-pipe to stdout
            dup2(STDERR_FILENO, stderrPipe[0]); // redirect err-pipe to stderr
          }

          // detect requested or normal exit
          bool normalExit = !run || (WIFEXITED(status) && WEXITSTATUS(status) == EXIT_SUCCESS);

          // dump trace and assert trace
          if (!normalExit) {
            NaoBody naoBody;
            if (naoBody.init()) {
              naoBody.setCrashed(WIFSIGNALED(status) ? int(WTERMSIG(status)) : int(abnormalTerminationState));
              naoBody.cleanup();
            }
            Assert::logDump(true, WIFSIGNALED(status) ? int(WTERMSIG(status)) : 0);
          }
          Assert::logDump(false, WIFSIGNALED(status) ? int(WTERMSIG(status)) : 0);

          // quit here?
          if (normalExit)
            exit(WIFEXITED(status) ? WEXITSTATUS(status) : EXIT_FAILURE);

          // don't restart if the child process got killed
          if (WIFSIGNALED(status) && WTERMSIG(status) == SIGKILL)
            exit(EXIT_FAILURE);

// restart in release mode only
#ifndef NDEBUG
          exit(EXIT_FAILURE);
#endif

          // deactivate the pre-initial state
          recover = true;

          usleep(2000 * 1000);
        } else {
          if (pipeReady) {
            // close unused read end
            close(stdoutPipe[0]);
            close(stderrPipe[0]);

            dup2(STDOUT_FILENO, stdoutPipe[1]); // redirect stdout to out-pipe
            dup2(STDERR_FILENO, stderrPipe[1]); // redirect stderr to err-pipe
          }
          break;
        }
      }
    }
  }

  // wait for naobridge
  NaoBody naoBody;
  if (!naoBody.init()) {
    fprintf(stderr, "NomadZ: Waiting for nao bridge...\n");
    do {
      usleep(1000000);
    } while (!naoBody.init());
  }
  printf("Hi, I am %s.\n", naoBody.getName());

#ifdef ROBOT_V6
  // reset camera
  printf("Resetting cameras...\n");
  system("/usr/libexec/reset-cameras.sh toggle");
  SystemCall::sleep(2000);
#endif

  // load first settings instance
  Settings settings;
  settings.recover = recover;

  // register signal handler for strg+c and termination signal
  signal(SIGTERM, sighandlerShutdown);
  signal(SIGINT, sighandlerShutdown);

  nomadzStart(argc, argv);

  while (run
#ifdef ENABLE_ROS
         && rclcpp::ok()
#endif
  )
    pause();

  nomadzStop();
  naoBody.shutdown();

  return EXIT_SUCCESS;
}
