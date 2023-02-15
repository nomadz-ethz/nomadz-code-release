/**
 * @file StreamableHelper.h
 *
 * This file declares macros used by auto streaming that work
 * with clang.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 */

#pragma once

/**
 * Determine the number of entries in a tuple.
 * The third part is declared somewhere else, because it is
 * platform independent.
 */
#define _STREAM_TUPLE_SIZE(...)                                                                                             \
  _STREAM_TUPLE_SIZE_I((__VA_ARGS__,                                                                                        \
                        100,                                                                                                \
                        99,                                                                                                 \
                        98,                                                                                                 \
                        97,                                                                                                 \
                        96,                                                                                                 \
                        95,                                                                                                 \
                        94,                                                                                                 \
                        93,                                                                                                 \
                        92,                                                                                                 \
                        91,                                                                                                 \
                        90,                                                                                                 \
                        89,                                                                                                 \
                        88,                                                                                                 \
                        87,                                                                                                 \
                        86,                                                                                                 \
                        85,                                                                                                 \
                        84,                                                                                                 \
                        83,                                                                                                 \
                        82,                                                                                                 \
                        81,                                                                                                 \
                        80,                                                                                                 \
                        79,                                                                                                 \
                        78,                                                                                                 \
                        77,                                                                                                 \
                        76,                                                                                                 \
                        75,                                                                                                 \
                        74,                                                                                                 \
                        73,                                                                                                 \
                        72,                                                                                                 \
                        71,                                                                                                 \
                        70,                                                                                                 \
                        69,                                                                                                 \
                        68,                                                                                                 \
                        67,                                                                                                 \
                        66,                                                                                                 \
                        65,                                                                                                 \
                        64,                                                                                                 \
                        63,                                                                                                 \
                        62,                                                                                                 \
                        61,                                                                                                 \
                        60,                                                                                                 \
                        59,                                                                                                 \
                        58,                                                                                                 \
                        57,                                                                                                 \
                        56,                                                                                                 \
                        55,                                                                                                 \
                        54,                                                                                                 \
                        53,                                                                                                 \
                        52,                                                                                                 \
                        51,                                                                                                 \
                        50,                                                                                                 \
                        49,                                                                                                 \
                        48,                                                                                                 \
                        47,                                                                                                 \
                        46,                                                                                                 \
                        45,                                                                                                 \
                        44,                                                                                                 \
                        43,                                                                                                 \
                        42,                                                                                                 \
                        41,                                                                                                 \
                        40,                                                                                                 \
                        39,                                                                                                 \
                        38,                                                                                                 \
                        37,                                                                                                 \
                        36,                                                                                                 \
                        35,                                                                                                 \
                        34,                                                                                                 \
                        33,                                                                                                 \
                        32,                                                                                                 \
                        31,                                                                                                 \
                        30,                                                                                                 \
                        29,                                                                                                 \
                        28,                                                                                                 \
                        27,                                                                                                 \
                        26,                                                                                                 \
                        25,                                                                                                 \
                        24,                                                                                                 \
                        23,                                                                                                 \
                        22,                                                                                                 \
                        21,                                                                                                 \
                        20,                                                                                                 \
                        19,                                                                                                 \
                        18,                                                                                                 \
                        17,                                                                                                 \
                        16,                                                                                                 \
                        15,                                                                                                 \
                        14,                                                                                                 \
                        13,                                                                                                 \
                        12,                                                                                                 \
                        11,                                                                                                 \
                        10,                                                                                                 \
                        9,                                                                                                  \
                        8,                                                                                                  \
                        7,                                                                                                  \
                        6,                                                                                                  \
                        5,                                                                                                  \
                        4,                                                                                                  \
                        3,                                                                                                  \
                        2,                                                                                                  \
                        1))
#define _STREAM_TUPLE_SIZE_I(params) _STREAM_TUPLE_SIZE_II params

/**
 * Determine whether a sequence is of the form "(a) b" or "(a)(b) c".
 * In the first case, 1 is returned, otherwise 2.
 */
#define _STREAM_SEQ_SIZE(seq) _STREAM_CAT(_STREAM_SEQ_SIZE, _STREAM_SEQ_SIZE_0 seq))
#define _STREAM_SEQ_SIZE_0(...) _STREAM_SEQ_SIZE_1
#define _STREAM_SEQ_SIZE_1(...) _STREAM_SEQ_SIZE_2
#define _STREAM_SEQ_SIZE_STREAM_SEQ_SIZE_1 1 _STREAM_DROP(
#define _STREAM_SEQ_SIZE_STREAM_SEQ_SIZE_2 2 _STREAM_DROP(

/** Remove surrounding parentheses from header. */
#define _STREAM_UNWRAP(...) __VA_ARGS__

/**
 * Generate a streamable class that is directly derived from Streamable
 * @param name The name of the class.
 * @param header A header that starts with a curly bracket and can contain
 *               arbitrary declarations. Please note that commas are only
 *               allowed if enclosed by parantheses.
 * @param ... The actual declarations. However, the last entry can contain
 *            code for the body of the default constructor. In any case, it
 *            must end with a closing curly bracket.
 */
#define STREAMABLE(name, header, ...) _STREAM_STREAMABLE(name, Streamable, , (header), __VA_ARGS__)

/**
 * Generate a streamable class that is derived from a class that is already
 * streamable. Please note that the serialize method in the base class must
 * not be private. For instance, this is the case if that class also was
 * created by a STREAMABLE macro.
 * @param name The name of the class.
 * @param name The name of the base class.
 * @param header A header that starts with a curly bracket and can contain
 *               arbitrary declarations. Please note that commas are only
 *               allowed if enclosed by parantheses.
 * @param ... The actual declarations. However, the last entry can contain
 *            code for the body of the default constructor. In any case, it
 *            must end with a closing curly bracket.
 */
#define STREAMABLE_WITH_BASE(name, base, header, ...)                                                                       \
  _STREAM_STREAMABLE(name, base, STREAM_BASE(base), (header), __VA_ARGS__)

/**
 *  Forward declarations required for ROS support.
 */

namespace internal {
  struct ROSBaseTag {};
  template <typename T, bool = true> struct ROSMessage : public T, public ROSBaseTag {
  public:
    using ROSType = T;
    using ROSDeclType = T;
  };
  template <typename T> class ROSMessage<T, false> {
  public:
    static constexpr bool ros_compatible = false;
    using ROSDeclType = T;
  };
  template <typename T, typename = void> struct ros_msg_wrapper { using internal_type = ROSMessage<T, false>; };
  template <typename T> struct ros_msg_wrapper<T, std::void_t<decltype(sizeof(T))>> { using internal_type = ROSMessage<T>; };

  class ref_init_tag {};
  class ref_alias_tag {};
} // namespace internal

#define STREAMABLE_DECLARE(name)                                                                                            \
  namespace nomadz_msgs {                                                                                                   \
    namespace msg {                                                                                                         \
      template <typename> struct name##_;                                                                                   \
      using name = name##_<std::allocator<void>>;                                                                           \
    }                                                                                                                       \
  }                                                                                                                         \
  class name;                                                                                                               \
  class name##BaseWrapper : public Streamable, public internal::ros_msg_wrapper<nomadz_msgs::msg::name>::internal_type {    \
  protected:                                                                                                                \
    void serialize(In* in, Out* out) override {                                                                             \
      STREAM_REGISTER_BEGIN;                                                                                                \
      STREAM_REGISTER_FINISH;                                                                                               \
    };                                                                                                                      \
  };

#ifdef ENABLE_ROS
#define STREAMABLE_ROS(name, header, ...)                                                                                   \
  _STREAM_STREAMABLE_ROS(name, name##BaseWrapper, STREAM_BASE(name##BaseWrapper), (header), __VA_ARGS__)
#else
#define STREAMABLE_ROS(name, header, ...)                                                                                   \
  _STREAM_STREAMABLE(name, name##BaseWrapper, STREAM_BASE(name##BaseWrapper), (header), __VA_ARGS__)
#endif

#define STREAMABLE_DECLARE_LEGACY(name) STREAMABLE_DECLARE(name)

#define STREAMABLE_LEGACY(name, header, ...)                                                                                \
  _STREAM_STREAMABLE(name, name##BaseWrapper, STREAM_BASE(name##BaseWrapper), (header), __VA_ARGS__)

#define STREAMABLE_ALIAS(name, base, body) class name : public base body
#define ALIAS_ROS_HELPERS(name, base)                                                                                       \
  template <typename RefType> name(RefType& ref, internal::ref_alias_tag tag) : base(ref, tag) {}                           \
  template <typename RefType> name(RefType& ref, internal::ref_init_tag tag) : base(ref, tag) {}
