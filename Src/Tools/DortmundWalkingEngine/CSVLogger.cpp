/**
 * @file CSVLogger.cpp
 *
 * This file is subject to the terms of the Nao Devils 2017 License.
 * A copy of this license is included in LICENSE.NaoDevils.txt.
 * (c) 2022 Nao Devils and NomadZ team
 *
 * @note The original author is <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#include "CSVLogger.h"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include "Core/System/File.h"

using namespace std;

CSVLogger::CSVLogger(void) {}

CSVLogger::~CSVLogger(void) {}

CSVLogger::LogfileList CSVLogger::logs;
bool CSVLogger::disabled = false;

void CSVLogger::log(string titel, string name, string data) {
  if (disabled) {
    return;
  }
  stringstream s;
  s << data;
  add(titel, name, s.str());
}

void CSVLogger::log(string titel, string name, unsigned int data) {
  if (disabled) {
    return;
  }
  stringstream s;
  s << data;
  add(titel, name, s.str());
}

void CSVLogger::log(string titel, string name, unsigned long data) {
  if (disabled) {
    return;
  }
  stringstream s;
  s << data;
  add(titel, name, s.str());
}

void CSVLogger::log(string titel, string name, int data) {
  if (disabled) {
    return;
  }
  stringstream s;
  s << data;
  add(titel, name, s.str());
}

void CSVLogger::log(string titel, string name, long data) {
  if (disabled) {
    return;
  }
  stringstream s;
  s << data;
  add(titel, name, s.str());
}

void CSVLogger::log(string titel, string name, double data) {
  if (disabled) {
    return;
  }
  stringstream s;
  s << data;
  add(titel, name, s.str());
}

void CSVLogger::mark(string titel, string name) {
  if (disabled) {
    return;
  }
  Logfile* f = getFile(titel);
  if (f == NULL) {
    f = addFile(titel);
  }

  if (f->headerWritten) {
    return;
  }

  Column* c = getColumn(f, name);
  if (c == NULL) {
    c = addColumn(f, name);
  }
}

bool CSVLogger::add(string titel, string name, string data) {
  Logfile* f = getFile(titel);
  if (f == NULL) {
    f = addFile(titel);
  }

  Column* c = getColumn(f, name);

  if (c == NULL) {
    if (f->headerWritten) {
      return false;
    } else {
      c = addColumn(f, name);
    }
  }

  if (c->filled) {
    flush(f);
  }

  c->data = data;
  c->filled = true;

  return true;
}

void CSVLogger::flush() {
  if (disabled) {
    return;
  }
  for (LogfileList::iterator log = logs.begin(); log != logs.end(); ++log) {
    flush(*log);
  }
}

void CSVLogger::flush(Logfile* f) {
  if (disabled) {
    return;
  }
  if (!f->headerWritten) {
    for (ColumnList::iterator el = f->columns.begin(); el != f->columns.end(); ++el) {
      f->s << (*el)->name;
      if ((*el) != f->columns.back()) {
        f->s << ";";
      } else {
        f->s << "\n";
      }
    }
    f->headerWritten = true;
  }

  for (ColumnList::iterator el = f->columns.begin(); el != f->columns.end(); ++el) {
    f->s << (*el)->data;
    if ((*el) != f->columns.back()) {
      f->s << ";";
    } else {
      f->s << "\n";
    }
    (*el)->filled = false;
    (*el)->data = "0";
  }
  f->s.flush();
}

CSVLogger::Logfile* CSVLogger::getFile(string name) {
  for (LogfileList::iterator log = logs.begin(); log != logs.end(); ++log) {
    if ((*log)->name == name) {
      return *log;
    }
  }
  return NULL;
}

CSVLogger::Column* CSVLogger::getColumn(Logfile* f, string name) {
  if (f->columns.size() == 0) {
    return NULL;
  }

  ColumnList::iterator el = f->lastFound;
  for (int i = 0; i < (int)f->columns.size(); i++, el++) {
    if (el == f->columns.end()) {
      el = f->columns.begin();
    }

    if ((*el)->name == name) {
      f->lastFound = el;
      return *el;
    }
  }
  return NULL;
}

CSVLogger::Logfile* CSVLogger::addFile(string name) {

  Logfile* neu = new Logfile;
  string path = File::getBHDir();
  path += "/Config/Logs/CSVLogger/";
  path += name;
  path += ".csv";
  neu->s.open(path.c_str(), ios::out);
  if (neu->s.fail()) {
    disabled = true;
  } else {
    disabled = false;
    std::cout << "Logfile: " << name << std::endl;
    neu->headerWritten = false;
    neu->name = name;
    logs.push_back(neu);
  }
  return neu;
}

CSVLogger::Column* CSVLogger::addColumn(Logfile* f, string name) {
  Column* neu = new Column;
  neu->filled = false;
  neu->data = "";
  neu->name = name;
  f->columns.push_back(neu);
  if (f->columns.size() == 1) {
    f->lastFound = f->columns.begin();
  }
  return neu;
}
