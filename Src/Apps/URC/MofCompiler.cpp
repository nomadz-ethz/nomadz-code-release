/**
 * @file MofCompiler.cpp
 *
 * This file implements a all functions required to compile the motion net for special actions.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in License.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are Uwe Düffert and Martin Lötzsch
 */
#include "URC/MofCompiler.h"

#include "Core/System/File.h"
#include "Representations/MotionControl/SpecialActionRequest.h"
#include "Representations/Infrastructure/JointData.h"
#include <fstream>
#include <cstdarg>
#include <cstring>

#include <dirent.h>
#include <string>

#define _strdup strdup
#define _vsnprintf vsnprintf

static const int MAXLAB = 5000;
static int numOfLabels = 0;
static char* label_motion[MAXLAB];
static char* label_name[MAXLAB];
static short label_number[MAXLAB];

static const int MAXLIN = 32000;
static int numOfLines = 0;
static char motion[512];
static int actMotionID = -1;
static char* line_data[MAXLIN];
static short line_number[MAXLIN];
static short line_motionID[MAXLIN];

static const int MAXFIL = 500;
static int numOfFiles = 0;
static char* file_name[MAXFIL];
static short file_startindex[MAXFIL];

static int jumpTable[SpecialActionRequest::numOfSpecialActionIDs];

static char* printBuffer;
static int printBufferSize;

/**
 * The function replaces printf so that the output is written into a buffer.
 * @param format Normal printf format string.
 * @return Normal printf return value.
 */
static int myprintf(const char* format, ...) {
  va_list args;
  va_start(args, format);
  size_t size = strlen(printBuffer);
  return _vsnprintf(printBuffer + size, printBufferSize - size, format, args);
}

#define printf myprintf

/** generate MotionNetData.cpp
 * @return True if successful
 */
static bool generateMotionNet() {
  char s[256];
  sprintf(s, "%s/Config/specialActions.dat", File::getBHDir());
  std::ofstream stream;
  stream.open(s, std::ios::out);
  stream << "//\n";
  stream << "// This file contains the special actions and was\n";
  stream << "// generated by the Universal Resource Compiler\n";
  stream << "//\n";
  stream << "// authors: Uwe Düffert, Martin Lötzsch, Max Risler\n";
  stream << "//\n";

  stream << "\n// jump table:\n";
  for (int i = 0; i < SpecialActionRequest::numOfSpecialActionIDs; ++i)
    stream << jumpTable[i] << " ";

  stream << "\n\n// number of nodes:\n" << numOfLines << "\n\n";

  stream << "// nodes:\n";
  int labelnum = 0;
  int filenum = 0;
  for (int i = 0; i < numOfLines; ++i) {
    if (file_startindex[filenum] == i)
      ++filenum;
    if (label_number[labelnum] == i)
      ++labelnum;
    if (!strncmp(line_data[i], "transition", 10)) {
      char request[512];
      char viamotion[512];
      char vialabel[512];
      sscanf(line_data[i] + 11, "%s %s %s", request, viamotion, vialabel);
      // check if label exists (request was already checked)

      bool found = false;
      int j;
      for (j = 0; j < numOfLabels; ++j)
        if (!strcmp(label_motion[j], viamotion) && !strcmp(label_name[j], vialabel)) {
          found = true;
          break;
        }

      if (!found)
        for (int k = numOfFiles - 1; k >= 0; --k)
          if (file_startindex[k] <= i) {
            printf("%s(%i) : error: jump label unknown\n", file_name[k], line_number[i]);
            return false;
          }

      if (strcmp(request, "allMotions"))
        stream << "1 " << label_number[j] << " "
               << static_cast<int>(SpecialActionRequest::getSpecialActionFromName(request));
      else
        stream << "2 " << label_number[j];
    } else if (!strncmp(line_data[i], "hardness", 8))
      stream << "4 " << (line_data[i] + 9);
    else
      stream << "3 " << line_data[i];

    if (line_motionID[i] >= 0)
      stream << " " << line_motionID[i];
    else
      stream << " -1";

    if (i < numOfLines - 1)
      stream << "\n";
  }

  stream.close();
  return true;
}

/** parse all mof files except extern.mof to generate motion net
 * @return True if successful
 */
static bool parseMofs() {
  numOfFiles = 0;
#ifndef WIN32
  dirent* ff = 0;
  DIR* fd;
#define FFNAME ff->d_name
#else
  struct _finddata_t ff;
  long fd;
#define FFNAME ff.name
#endif

  char ffname[256];
#ifndef WIN32
  sprintf(ffname, "%s/Src/Modules/MotionControl/mof", File::getBHDir());
#else
  sprintf(ffname, "%s/Src/Modules/MotionControl/mof/*.mof", File::getBHDir());
#endif
  bool thereAreMore;
#ifndef WIN32
  if ((fd = opendir(ffname))) {
    do {
      ff = readdir(fd);
      thereAreMore = ff != NULL;
    } while (thereAreMore && (strcmp(FFNAME, ".mof") <= 0 || strlen(FFNAME) <= 4));
  } else {
    thereAreMore = false;
  }

#else
  fd = _findfirst(ffname, &ff);
  thereAreMore = fd != NULL;
#endif
  while (thereAreMore) {
    if (strcmp(FFNAME, "extern.mof")) {
      char name[512];
      sprintf(name, "%s/Src/Modules/MotionControl/mof/%s", File::getBHDir(), FFNAME);
      FILE* f = fopen(name, "r");
      if (f == NULL) {
        printf("error opening %s. Aborting.\n", name);
        return false;
      } else {
        file_name[numOfFiles] = _strdup(name);
        file_startindex[numOfFiles++] = (short)numOfLines;
        bool thisMofHasLabels = false;
        strcpy(motion, FFNAME);
        if (!strcmp(motion + strlen(motion) - 4, ".mof"))
          motion[strlen(motion) - 4] = 0;
        actMotionID = -1;
        for (int j = 0; j < SpecialActionRequest::numOfSpecialActionIDs; ++j)
          if (!strcmp(SpecialActionRequest::getName((SpecialActionRequest::SpecialActionID)j), motion)) {
            actMotionID = j;
            break;
          }

        char s[128000];
        int siz = fread(s, 1, 128000, f);
        fclose(f);
        if (siz > 0) {
          s[siz] = 0;
          char* t = &s[0];
          int line = 1;
          while (*t) {
            char* u = strchr(t, '\n');
            if (u >= t) {
              *u = 0;

              char sval[JointData::numOfJoints + 3][256]; // joints + interpolate + duration + bad argument
              int c = sscanf(
                t,
                "%s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s %s", // (numOfJoints
                                                                                                          // + 3) * %s
                sval[0],
                sval[1],
                sval[2],
                sval[3],
                sval[4],
                sval[5],
                sval[6],
                sval[7],
                sval[8],
                sval[9],
                sval[10],
                sval[11],
                sval[12],
                sval[13],
                sval[14],
                sval[15],
                sval[16],
                sval[17],
                sval[18],
                sval[19],
                sval[20],
                sval[21],
                sval[22],
                sval[23],
                sval[24],
                sval[25],
                sval[26],
                sval[27],
                sval[28]); // sval[0]..sval[numOfJoints + 2]
              if (c == -1 || sval[0][0] == '"' || !strncmp(sval[0], "//", 2)) {
                // skip comments and empty lines by doing nothing
              } else if (!strcmp(sval[0], "motion_id")) {
                // if there is a motion_id we use it, otherwise filename
                if (!strcmp(sval[1], "=") && c == 3) {
                  strcpy(motion, sval[2]);
                  for (int j = 0; j < SpecialActionRequest::numOfSpecialActionIDs; ++j)
                    if (!strcmp(SpecialActionRequest::getName(SpecialActionRequest::SpecialActionID(j)), motion)) {
                      actMotionID = j;
                      break;
                    }
                } else {
                  printf("%s(%i) : error: motion_id format\n", name, line);
                  return false;
                }
              } else if (!strcmp(sval[0], "transition")) {
                bool found = false;
                if (!thisMofHasLabels) {
                  printf("%s(%i) : error: this line is unreachable, because there was no label before\n", name, line);
                  return false;
                } else if (!strcmp(sval[1], "allMotions"))
                  found = true;
                else
                  for (int j = 0; j < SpecialActionRequest::numOfSpecialActionIDs; ++j)
                    if (!strcmp(SpecialActionRequest::getName(SpecialActionRequest::SpecialActionID(j)), sval[1])) {
                      found = true;
                      break;
                    }

                if (!found) {
                  printf("%s(%i) : error: request for transition unknown\n", name, line);
                  return false;
                }

                if (c == 4) {
                  line_motionID[numOfLines] = (short)actMotionID;
                  line_data[numOfLines] = _strdup(t);
                  line_number[numOfLines++] = (short)line;
                } else {
                  printf("%s(%i) : error: transition format\n", name, line);
                  return false;
                }
              } else if (!strcmp(sval[0], "label")) {
                if (c == 2) {
                  label_motion[numOfLabels] = _strdup(motion);
                  label_name[numOfLabels] = _strdup(sval[1]);
                  label_number[numOfLabels++] = (short)numOfLines;
                  thisMofHasLabels = true;
                } else {
                  printf("%s(%i) : error: label format\n", name, line);
                  return false;
                }
              } else if (!strcmp(sval[0], "hardness")) {
                if (!thisMofHasLabels) {
                  printf("%s(%i) : error: this line is unreachable, because there was no label before\n", name, line);
                  return false;
                }
                if (c == JointData::numOfJoints + 2) {
                  char temp[1024];
                  temp[0] = 0;
                  strcat(temp, "hardness");
                  int val = 0;

                  for (int j = 1; j < JointData::numOfJoints + 1; j++) {
                    if (!strcmp(sval[j], "*"))
                      sprintf(sval[j], "%i", HardnessData::useDefault);

                    if ((!strcmp(sval[j], "-1") || sscanf(sval[j], "%i", &val) == 1) && val >= 0 && val <= 100) {
                      strcat(temp, " ");
                      strcat(temp, sval[j]);
                    } else {
                      printf("%s(%i) : error: hardness data format\n", name, line);
                      return false;
                    }
                  }
                  strcat(temp, " ");
                  strcat(temp, sval[JointData::numOfJoints + 1]);

                  line_motionID[numOfLines] = (short)actMotionID;
                  line_data[numOfLines] = _strdup(temp);
                  line_number[numOfLines++] = (short)line;
                } else {
                  printf("%s(%i) : error: hardness format\n", name, line);
                  return false;
                }
              } else if (c == JointData::numOfJoints + 2) {
                char temp[1024];
                temp[0] = 0;
                int val = 0;
                if (!thisMofHasLabels) {
                  printf("%s(%i) : error: this line is unreachable, because there was no label before\n", name, line);
                  return false;
                }

                for (int j = 0; j < JointData::numOfJoints; ++j) {
                  if (!strcmp(sval[j], "*"))
                    sprintf(sval[j], "%i", JointData::ignore);
                  if (!strcmp(sval[j], "-"))
                    sprintf(sval[j], "%i", JointData::off);

                  if ((!strcmp(sval[j], "1000") || !strcmp(sval[j], "2000") || sscanf(sval[j], "%i", &val) == 1) &&
                      val >= -210 && val <= 210) {
                    if (j != 0)
                      strcat(temp, " ");
                    strcat(temp, sval[j]);
                  } else {
                    printf("%s(%i) : error: joint data format\n", name, line);
                    return false;
                  }
                }
                if (sscanf(sval[JointData::numOfJoints], "%i", &val) == 1 && (val >= 0 && val <= 3)) {
                  strcat(temp, " ");
                  strcat(temp, sval[JointData::numOfJoints]);
                } else {
                  printf("%s(%i) : error: interpolation data format\n", name, line);
                  return false;
                }
                if (sscanf(sval[JointData::numOfJoints + 1], "%i", &val) == 1 && val > 0) {
                  strcat(temp, " ");
                  strcat(temp, sval[JointData::numOfJoints + 1]);
                } else {
                  printf("%s(%i) : error: time data format\n", name, line);
                  return false;
                }
                line_motionID[numOfLines] = (short)actMotionID;
                line_data[numOfLines] = _strdup(temp);
                line_number[numOfLines++] = (short)line;
              } else {
                printf("%s(%i) : error: illegal number of arguments\n", name, line);
                return false;
              }
              ++line;
              t = u + 1;
            } else
              t += strlen(t);
          }
        } else {
          printf("error reading from %s. Aborting.\n", name);
          return false;
        }
      }
    }
#ifndef WIN32
    if (fd) {
      do {
        ff = readdir(fd);
        thereAreMore = ff != NULL;
      } while (thereAreMore && (strcmp(FFNAME, ".mof") <= 0 || strlen(FFNAME) <= 4));
    } else {
      thereAreMore = false;
    }
#else
    thereAreMore = !_findnext(fd, &ff);
#endif
  }
  return true;
}

/** parse extern.mof to generate jump table from extern into motion net
 * @return True if successful
 */
static bool parseExternMof() {
  for (int i = 0; i < SpecialActionRequest::numOfSpecialActionIDs; ++i)
    jumpTable[i] = -1;

  char name[512];
  sprintf(name, "%s/Src/Modules/MotionControl/mof/extern.mof", File::getBHDir());
  FILE* f = fopen(name, "r");
  if (f == NULL) {
    printf("error opening %s. Aborting.\n", name);
    return false;
  } else {
    char s[128000] = {0};
    int siz = fread(s, 1, 128000, f);
    fclose(f);
    if (siz > 0) {
      s[siz] = 0;
      char* t = &s[0];
      int line = 1;
      while (*t) {
        char* u = strchr(t, '\n');
        if (u >= t) {
          *u = 0;

          char sval[5][256];
          int c = sscanf(t, "%s %s %s %s %s", sval[0], sval[1], sval[2], sval[3], sval[4]);

          if (c == -1 || sval[0][0] == '"' || !strncmp(sval[0], "//", 2)) {
            // skip comments and empty lines by doing nothing
          } else if (!strcmp(sval[0], "motion_id") || !strcmp(sval[0], "label")) {
            // skip labels and id
          } else if (!strcmp(sval[0], "transition") && c == 4) {
            bool found = false;
            if (!strcmp(sval[1], "extern"))
              continue;
            else {
              int i;
              for (i = 0; i < SpecialActionRequest::numOfSpecialActionIDs; ++i)
                if (!strcmp(SpecialActionRequest::getName(SpecialActionRequest::SpecialActionID(i)), sval[1])) {
                  found = true;
                  break;
                }

              if (!found) {
                printf("%s(%i) : error: special action '%s' unknown (not in SpecialActionRequest.h)\n", name, line, sval[1]);
                return false;
              }

              for (int j = 0; j < numOfLabels; ++j)
                if (!strcmp(label_motion[j], sval[2]) && !strcmp(label_name[j], sval[3])) {
                  jumpTable[i] = label_number[j];
                  found = true;
                  break;
                }

              if (!found) {
                printf("%s(%i) : error: jump label '%s:%s' unknown (not in *.mof)\n", name, line, sval[2], sval[3]);
                return false;
              }
            }
          } else {
            printf("%s(%i) : error: unexpected data\n", name, line);
            return false;
          }
          ++line;
          t = u + 1;
        } else
          t += strlen(t);
      }
    } else {
      printf("error reading from %s. Aborting.\n", name);
      return false;
    }
  }

  for (int i = 0; i < SpecialActionRequest::numOfSpecialActionIDs; ++i)
    if (jumpTable[i] == -1) {
      printf("%s/Src/Modules/MotionControl/mof/extern.mof(1): error: no motion net entry defined for special action %s\n",
             File::getBHDir(),
             SpecialActionRequest::getName(SpecialActionRequest::SpecialActionID(i)));
      return false;
    }
  return true;
}

bool compileMofs(char* buffer, size_t size) {
  printBuffer = buffer;
  printBufferSize = size;
  printBuffer[0] = 0;

  numOfLabels = 0;
  numOfLines = 0;
  label_motion[numOfLabels] = _strdup("extern");
  label_name[numOfLabels] = _strdup("start");
  label_number[numOfLabels++] = (short)numOfLines;

  line_data[numOfLines] = _strdup("transition allMotions extern start");
  line_number[numOfLines++] = 1;

  if (!parseMofs() || !parseExternMof())
    return false;

  // check whether the last line of every mof is an unconditional jump
  for (int i = 1; i <= numOfFiles; ++i) {
    int ind = i < numOfFiles ? file_startindex[i] - 1 : numOfLines - 1;
    if (strncmp(line_data[ind], "transition allMotions", 21)) {
      printf("%s(%i) : error: this mof does not end with a transition for allMotions\n", file_name[i - 1], line_number[ind]);
      return false;
    }
  }

  // check whether there are mofs without SpecialActionRequest::SpecialAction
  for (int i = 1; i < numOfLabels; ++i) {
    if (!strcmp(label_motion[i], label_motion[i - 1]))
      continue;
    bool found = false;
    for (int j = 0; j < SpecialActionRequest::numOfSpecialActionIDs; ++j)
      if (!strcmp(label_motion[i], SpecialActionRequest::getName(SpecialActionRequest::SpecialActionID(j)))) {
        found = true;
        break;
      }
    if (!found)
      for (int j = numOfFiles - 1; j >= 0; --j)
        if (file_startindex[j] <= label_number[i]) {
          printf("%s(%i) : warning: there is no special action id for this mof\n", file_name[j], 1);
          break;
        }
  }

  /** @todo warning for motions without labels */

  if (!generateMotionNet())
    return false;

  for (int i = 0; i < numOfLines; ++i)
    delete line_data[i];

  for (int i = 0; i < numOfFiles; ++i)
    delete file_name[i];

  for (int i = 0; i < numOfLabels; ++i) {
    delete label_name[i];
    delete label_motion[i];
  }

  return true;
}
