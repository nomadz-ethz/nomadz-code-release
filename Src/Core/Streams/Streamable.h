/**
 * @file Streamable.h
 *
 * Base class for all types streamed through StreamHandler macros.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original authors are Michael Spranger and Tobias Oberlies
 */

#pragma once

#include <type_traits>

#ifdef _MSC_VER
class type_info;
#else
#include <typeinfo>
#endif

#include <vector>
#include "InOut.h"

#ifdef RELEASE

/** Must be used at the end of any streaming operator or serialize(In*, Out*) function */
#define STREAM_REGISTER_FINISH

// Macros dedicated for the use within the serialize(In*, Out*) of data types derived from Streamable function

/** Must be used at the beginning of the serialize(In*, Out*) function. */
#define STREAM_REGISTER_BEGIN

/**
 * Registers and streams a base class
 * @param s A pointer to the base class.
 */
#define STREAM_BASE(s) _STREAM_BASE(s, )

/**
 * Must be used at the beginning of the streaming operator
 * @param s Object to be streamed within this streaming operator (should be the second argument to the streaming operator)
 */
#define STREAM_REGISTER_BEGIN_EXT(s)

// Imported from BH2012
/**
 * Streams a Vector2<float> as Vector2<short>.
 * @param s The member to be streamed.
 */
#define STREAM_COMPRESSED_POSITION(s)                                                                                       \
  {                                                                                                                         \
    Vector2<short> _c(static_cast<short>(s.x), static_cast<short>(s.y));                                                    \
    {                                                                                                                       \
      Vector2<short>& s(_c);                                                                                                \
      STREAM(s)                                                                                                             \
    }                                                                                                                       \
    if (in)                                                                                                                 \
      s = Vector2<float>(static_cast<float>(_c.x), static_cast<float>(_c.y));                                               \
  }

/**
 * Registers and streams a base class.
 * @param s A reference to a base class representation.
 */
#define STREAM_BASE_EXT(stream, s) _STREAM_BASE_EXT(stream, s, )

#else

#define STREAM_REGISTER_FINISH Streaming::finishRegistration();

#define STREAM_REGISTER_BEGIN Streaming::startRegistration(typeid(*this), false);
#define STREAM_BASE(s) _STREAM_BASE(s, Streaming::registerBase();)

#define STREAM_REGISTER_BEGIN_EXT(s) Streaming::startRegistration(typeid(s), true);
#define STREAM_BASE_EXT(stream, s) _STREAM_BASE_EXT(stream, s, Streaming::registerBase();)

#endif

#define _STREAM_BASE(s, reg) reg this->s::serialize(in, out);

#define _STREAM_BASE_EXT(stream, s, reg) reg streamObject(stream, s);

/**
 * Registration and streaming of a member.
 * The first parameter is the attribute to be registered and streamed.
 * If the type of that attribute is an enumeration and it is not defined
 * in the current class, the name of the class in which it is defined
 * has to be specified as second parameter.
 */
#define STREAM(...)                                                                                                         \
  _STREAM_EXPAND(_STREAM_EXPAND(_STREAM_THIRD(__VA_ARGS__, _STREAM_WITH_CLASS, _STREAM_WITHOUT_CLASS))(__VA_ARGS__))

#define _STREAM_THIRD(first, second, third, ...) third
#define _STREAM_EXPAND(s) s // needed for Visual Studio

#define _STREAM_WITHOUT_CLASS(s)                                                                                            \
  Streaming::streamIt(                                                                                                      \
    in,                                                                                                                     \
    out,                                                                                                                    \
    #s,                                                                                                                     \
    s,                                                                                                                      \
    Streaming::Casting<::internal::is_enum<decltype(Streaming::unwrap(s))>::value>::getNameFunction(*this, s));

#define _STREAM_WITH_CLASS(s, class) Streaming::streamIt(in, out, #s, s, Streaming::castFunction(s, class ::getName));

/**
 * Registration and streaming of a member in an external streaming operator
 * (<< or >>).
 * The second parameter is the attribute to be registered and streamed.
 * If the type of that attribute is an enumeration, the name of the class
 * in which it is defined has to be specified as third parameter.
 * @param stream Reference to the stream, that should be streamed to.
 */
#define STREAM_EXT(stream, ...)                                                                                             \
  _STREAM_EXPAND(_STREAM_EXPAND(_STREAM_THIRD(__VA_ARGS__, _STREAM_EXT_ENUM, _STREAM_EXT_NORMAL))(stream, __VA_ARGS__))

#define _STREAM_EXT_NORMAL(stream, s)                                                                                       \
  Streaming::streamIt(                                                                                                      \
    stream,                                                                                                                 \
    #s,                                                                                                                     \
    s,                                                                                                                      \
    Streaming::Casting<::internal::is_enum<decltype(Streaming::unwrap(s))>::value>::getNameFunction(stream, s));

#define _STREAM_EXT_ENUM(stream, s, class) Streaming::streamIt(stream, #s, s, Streaming::castFunction(s, class ::getName));

/**
 * Base class for all classes using the STREAM or STREAM_EXT macros (see Tools/Debugging/
 * StreamHandler.h) for automatic streaming of class specifications to RobotControl.
 * Only those instances of those classes can be parsed by RobotControl when they are
 * transmitted through MODIFY and SEND macros (see Tools/Debugging/DebugDataTable.h).
 */
class ImplicitlyStreamable {};

/**
 * Base class for all classes using the STREAM macros for streaming instances.
 */
class Streamable : public ImplicitlyStreamable {
protected:
  virtual void serialize(In*, Out*) = 0;

public:
  virtual ~Streamable() {}
  void streamOut(Out&) const;
  void streamIn(In&);
};

In& operator>>(In& in, Streamable& streamable);

Out& operator<<(Out& out, const Streamable& streamable);

// Helpers

namespace Streaming {
  Out& dummyStream();

  template <class T> static void registerDefaultElement(const std::vector<T>&) {
    static T dummy;
    dummyStream() << dummy;
  }

  template <class T>
  In& streamComplexStaticArray(In& in, T inArray[], int size, const char* (*enumToString)(unsigned char)) {
    int numberOfEntries = size / sizeof(T);
    for (int i = 0; i < numberOfEntries; ++i) {
      in.select(0, i, enumToString);
      in >> inArray[i];
      in.deselect();
    }
    return in;
  }

  template <class T>
  Out& streamComplexStaticArray(Out& out, T outArray[], int size, const char* (*enumToString)(unsigned char)) {
    int numberOfEntries = size / sizeof(T);
    for (int i = 0; i < numberOfEntries; ++i) {
      out.select(0, i, enumToString);
      out << outArray[i];
      out.deselect();
    }
    return out;
  }

  template <class T> In& streamBasicStaticArray(In& in, T inArray[], int size, const char* (*enumToString)(unsigned char)) {
    if (in.isBinary()) {
      in.read(inArray, size);
      return in;
    } else
      return streamComplexStaticArray(in, inArray, size, enumToString);
  }

  template <class T>
  Out& streamBasicStaticArray(Out& out, T outArray[], int size, const char* (*enumToString)(unsigned char)) {
    if (out.isBinary()) {
      out.write(outArray, size);
      return out;
    } else
      return streamComplexStaticArray(out, outArray, size, enumToString);
  }

  inline In& streamStaticArray(In& in, unsigned char inArray[], int size, const char* (*enumToString)(unsigned char)) {
    return streamBasicStaticArray(in, inArray, size, enumToString);
  }
  inline Out& streamStaticArray(Out& out, unsigned char outArray[], int size, const char* (*enumToString)(unsigned char)) {
    return streamBasicStaticArray(out, outArray, size, enumToString);
  }
  inline In& streamStaticArray(In& in, char inArray[], int size, const char* (*enumToString)(unsigned char)) {
    return streamBasicStaticArray(in, inArray, size, enumToString);
  }
  inline Out& streamStaticArray(Out& out, char outArray[], int size, const char* (*enumToString)(unsigned char)) {
    return streamBasicStaticArray(out, outArray, size, enumToString);
  }
  inline In& streamStaticArray(In& in, unsigned short inArray[], int size, const char* (*enumToString)(unsigned char)) {
    return streamBasicStaticArray(in, inArray, size, enumToString);
  }
  inline Out& streamStaticArray(Out& out, unsigned short outArray[], int size, const char* (*enumToString)(unsigned char)) {
    return streamBasicStaticArray(out, outArray, size, enumToString);
  }
  inline In& streamStaticArray(In& in, short inArray[], int size, const char* (*enumToString)(unsigned char)) {
    return streamBasicStaticArray(in, inArray, size, enumToString);
  }
  inline Out& streamStaticArray(Out& out, short outArray[], int size, const char* (*enumToString)(unsigned char)) {
    return streamBasicStaticArray(out, outArray, size, enumToString);
  }
  inline In& streamStaticArray(In& in, unsigned int inArray[], int size, const char* (*enumToString)(unsigned char)) {
    return streamBasicStaticArray(in, inArray, size, enumToString);
  }
  inline Out& streamStaticArray(Out& out, unsigned int outArray[], int size, const char* (*enumToString)(unsigned char)) {
    return streamBasicStaticArray(out, outArray, size, enumToString);
  }
  inline In& streamStaticArray(In& in, int inArray[], int size, const char* (*enumToString)(unsigned char)) {
    return streamBasicStaticArray(in, inArray, size, enumToString);
  }
  inline Out& streamStaticArray(Out& out, int outArray[], int size, const char* (*enumToString)(unsigned char)) {
    return streamBasicStaticArray(out, outArray, size, enumToString);
  }
  inline In& streamStaticArray(In& in, float inArray[], int size, const char* (*enumToString)(unsigned char)) {
    return streamBasicStaticArray(in, inArray, size, enumToString);
  }
  inline Out& streamStaticArray(Out& out, float outArray[], int size, const char* (*enumToString)(unsigned char)) {
    return streamBasicStaticArray(out, outArray, size, enumToString);
  }
  inline In& streamStaticArray(In& in, double inArray[], int size, const char* (*enumToString)(unsigned char)) {
    return streamBasicStaticArray(in, inArray, size, enumToString);
  }
  inline Out& streamStaticArray(Out& out, double outArray[], int size, const char* (*enumToString)(unsigned char)) {
    return streamBasicStaticArray(out, outArray, size, enumToString);
  }
  template <class T> In& streamStaticArray(In& in, T inArray[], int size, const char* (*enumToString)(unsigned char)) {
    return streamComplexStaticArray(in, inArray, size, enumToString);
  }
  template <class T> Out& streamStaticArray(Out& out, T outArray[], int size, const char* (*enumToString)(unsigned char)) {
    return streamComplexStaticArray(out, outArray, size, enumToString);
  }

  template <class T, class U> void cast(T& t, const U& u) { t = static_cast<T>(u); }

  void finishRegistration();

  void startRegistration(const std::type_info& ti, bool registerWithExternalOperator);

  void registerBase();

  void registerWithSpecification(const char* name, const std::type_info& ti);

  void registerEnum(const std::type_info& ti, const char* (*fp)(unsigned char));

  std::string demangle(const char* name);

  const char* skipDot(const char* name);

  template <typename S> struct Streamer {
    static void stream(In* in, Out* out, const char* name, S& s, const char* (*enumToString)(unsigned char)) {
#ifndef RELEASE
      registerWithSpecification(name, typeid(s));
      if (enumToString)
        Streaming::registerEnum(typeid(s), (const char* (*)(unsigned char))enumToString);
#endif
      if (in) {
        in->select(name, -2, enumToString);
        *in >> s;
        in->deselect();
      } else {
        out->select(name, -2, enumToString);
        *out << s;
        out->deselect();
      }
    }
  };

  template <typename E, size_t N> struct Streamer<E[N]> {
    typedef E S[N];
    static void stream(In* in, Out* out, const char* name, S& s, const char* (*enumToString)(unsigned char)) {
#ifndef RELEASE
      registerWithSpecification(name, typeid(s));
      if (enumToString)
        Streaming::registerEnum(typeid(s[0]), (const char* (*)(unsigned char))enumToString);
#endif
      if (in) {
        in->select(name, -1);
        streamStaticArray(*in, s, sizeof(s), enumToString);
        in->deselect();
      } else {
        out->select(name, -1);
        streamStaticArray(*out, s, sizeof(s), enumToString);
        out->deselect();
      }
    }
  };

  template <typename E> struct Streamer<std::vector<E>> {
    typedef std::vector<E> S;
    static void stream(In* in, Out* out, const char* name, S& s, const char* (*enumToString)(unsigned char)) {
#ifndef RELEASE
      registerDefaultElement(s);
      registerWithSpecification(name, typeid(&s[0]));
      if (enumToString)
        Streaming::registerEnum(typeid(s[0]), (const char* (*)(unsigned char))enumToString);
#endif
      if (in) {
        in->select(name, -1);
        unsigned _size;
        *in >> _size;
        s.resize(_size);
        if (!s.empty())
          streamStaticArray(*in, &s[0], s.size() * sizeof(s[0]), enumToString);
        in->deselect();
      } else {
        out->select(name, -1);
        *out << (unsigned)s.size();
        if (!s.empty())
          streamStaticArray(*out, &s[0], s.size() * sizeof(s[0]), enumToString);
        out->deselect();
      }
    }
  };

  /**
   * The following three functions are helpers for streaming data. (in == 0) != (out == 0).
   * @param S The type of the variable to be streamed.
   * @param in The stream to read from.
   * @param out The stream to write to.
   * @param name The name of the variable to be streamed.
   * @param s The variable to be streamed.
   * @param enumToString A function that provides a string representation for each enum value or 0 if
   *                     its parameter is outside the enum's range. If the variable to be streamed is not of enum
   *                     type, this parameter is 0.
   * This is the version for using inside of serialize methods.
   */
  template <typename S>
  void streamIt(In* in, Out* out, const char* name, S& s, const char* (*enumToString)(unsigned char) = 0) {
#ifdef ENABLE_ROS
    if constexpr (internal::has_wrapped_type<S>::value) {
      Streamer<typename S::wrapped_type>::stream(in, out, name, s, enumToString);
    } else {
      Streamer<S>::stream(in, out, name, s, enumToString);
    }
#else
    Streamer<S>::stream(in, out, name, s, enumToString);
#endif
  }

  /** This is the version for using inside operator>>. */
  template <typename S> void streamIt(In& in, const char* name, S& s, const char* (*enumToString)(unsigned char) = 0) {
    Streamer<S>::stream(&in, 0, skipDot(name), s, enumToString);
  }

  /** This is the version for using inside operator<<. */
  template <typename S>
  void streamIt(Out& out, const char* name, const S& s, const char* (*enumToString)(unsigned char) = 0) {
    Streamer<S>::stream(0, &out, skipDot(name), const_cast<S&>(s), enumToString);
  }

  /**
   * The function for returning a string representation for each enum value is internally handled as
   * a function with an int parameter. However, the only method of the following template struct ensures that the
   * correct overloaded version is picked, i.e. the one that accepts the enum T.
   */
  template <typename T> struct Function {
    inline static const char* (*cast(const char* (*enumToString)(T)))(unsigned char) {
      return (const char* (*)(unsigned char))enumToString;
    }
  };

  /**
   * The function for returning a string representation for each enum value is internally handled as
   * a function with an int parameter. However, the the following template functions ensure that the
   * correct overloaded version is picked, i.e. the one that accepts the enum T. These functions are
   * used in the case it is known that a type is an enum.
   * This is the implementation for plain enums.
   */
  template <typename T> inline const char* (*castFunction(const T&, const char* (*enumToString)(T)))(unsigned char) {
    return (const char* (*)(unsigned char))enumToString;
  }

  /** Implementation for fixed-size arrays of enums. */
  template <typename T, size_t N>
  inline const char* (*castFunction(const T (&)[N], const char* (*enumToString)(T)))(unsigned char) {
    return (const char* (*)(unsigned char))enumToString;
  }

  /** Implementation for vectors of enums. */
  template <typename T>
  inline const char* (*castFunction(const std::vector<T>&, const char* (*enumToString)(T)))(unsigned char) {
    return (const char* (*)(unsigned char))enumToString;
  }

  /**
   * The template struct Casting distinguishes between types that are enums and types that are not.
   * Each method returns the address of a function that can translate enum constants to string representations
   * of those constants, i.e. their names. The ruturned function will return 0 if it is parameterized with a
   * value outside the enums range.
   * Here, the versions for enums is implemented.
   */
  template <bool isEnum = true> struct Casting {
    /** Implementation for plain enums. */
    template <typename T, typename E> inline static const char* (*getNameFunction(const T&, const E&))(unsigned char) {
      return Function<E>::cast(T::getName);
    }

    /** Implementation for fixed-size arrays of enums. */
    template <typename T, typename E, size_t N>
    inline static const char* (*getNameFunction(const T&, const E (&)[N]))(unsigned char) {
      return Function<E>::cast(T::getName);
    }

    /** Implementation for vectors of enums. */
    template <typename T, typename E>
    inline static const char* (*getNameFunction(const T&, const std::vector<E>&))(unsigned char) {
      return Function<E>::cast(T::getName);
    }

    /** Plain ints are misclassified as enums. Do not return a function in this case. */
    template <typename T> inline static const char* (*getNameFunction(const T&, const int&))(unsigned char) { return 0; }

    /** int arrays are misclassified as enum arrays. Do not return a function in this case. */
    template <typename T, size_t N> inline static const char* (*getNameFunction(const T&, const int (&)[N]))(unsigned char) {
      return 0;
    }

    /** int vectors are misclassified as enum vectors. Do not return a function in this case. */
    template <typename T> inline static const char* (*getNameFunction(const T&, const std::vector<int>&))(unsigned char) {
      return 0;
    }
  };

  /**
   * Specialization of template struct Casting for the case that a type is not an enum type.
   * In that case, there is no function that can return names for enum elements.
   */
  template <> struct Casting<false> {
    template <typename T, typename E> inline static const char* (*getNameFunction(const T&, const E&))(unsigned char) {
      return 0;
    }
  };

  /**
   * The following three function signatures assure that in the process of determining whether a variable
   * is of an enum type, automatic type conversion operators of objects are not applied. Otherwise,
   * classes that implement an operator int () might be classified as enums.
   * The functions will only be used for resolving types. They are never called. Therefore, they are not
   * implemented.
   */
  template <typename T> const T unwrap(const T&);
  template <typename T, size_t N> const T unwrap(const T (&)[N]);
  template <typename T> const T unwrap(const std::vector<T>&);

  /**
   * Together with decltype, the following template allows to use any type
   * for declarations, even array types such as int[4]. It also works with
   * template parameters without the use of typename.
   * decltype(Streaming::TypeWrapper<myType>::type) myVar;
   */
  template <typename T> struct TypeWrapper { static T type; };
} // namespace Streaming
