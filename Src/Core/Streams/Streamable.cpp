/**
 * @file Streamable.cpp
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#include <typeinfo>
#include <cstring>

#include "Streamable.h"
#include "StreamHandler.h"
#include "Core/Math/Common.h"
#include "Core/System/BHAssert.h"

#include "Core/Global.h"

#include <cxxabi.h>

void Streamable::streamOut(Out& out) const {
  const_cast<Streamable*>(this)->serialize(NULL, &out);
}

void Streamable::streamIn(In& in) {
  serialize(&in, NULL);
}

In& operator>>(In& in, Streamable& streamable) {
  streamable.streamIn(in);
  return in;
}

Out& operator<<(Out& out, const Streamable& streamable) {
  streamable.streamOut(out);
  return out;
}

namespace Streaming {
  void finishRegistration() { Global::getStreamHandler().finishRegistration(); }

  void startRegistration(const std::type_info& ti, bool registerWithExternalOperator) {
    Global::getStreamHandler().startRegistration(ti.name(), registerWithExternalOperator);
  }

  void registerBase() { Global::getStreamHandler().registerBase(); }

  void registerWithSpecification(const char* name, const std::type_info& ti) {
    Global::getStreamHandler().registerWithSpecification(name, ti);
  }

  void registerEnum(const std::type_info& ti, const char* (*fp)(unsigned char)) {
    Global::getStreamHandler().registerEnum(ti, fp);
  }

  std::string demangle(const char* name) {
#ifdef WIN32
    return name;
#else
    char realName[1000]; // This should be big enough, so realloc is never called.
    int status;
    size_t length = sizeof(realName);
    abi::__cxa_demangle(name, realName, &length, &status);
    ASSERT(!status);
    return realName;
#endif
  }

  Out& dummyStream() { return Global::getStreamHandler().dummyStream; }

  const char* skipDot(const char* name) { return strchr(name, '.') + 1; }
} // namespace Streaming
