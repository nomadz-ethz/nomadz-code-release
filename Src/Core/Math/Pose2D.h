/**
 * @file Pose2D.h
 *
 * Contains class Pose2D
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * and Max Risler
 */

#pragma once

#include "Vector2.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/pose2_d.hpp"
namespace internal {
  class pose2d_ros_wrapper : public ROSMessage<nomadz_msgs::msg::Pose2D> {
  public:
    using base_type = nomadz_msgs::msg::Pose2D;
    static constexpr bool ros_compatible = true;
  };
} // namespace internal
#else
namespace internal {
  class pose2d_ros_wrapper : public ROSMessage<void, false> {
  public:
    using base_type = pose2d_ros_wrapper;
    static constexpr bool ros_compatible = false;
    float rotation;
  };
} // namespace internal
#endif

#ifdef ENABLE_ROS
#define STREAMABLE_WITH_POSE2D(name, header, ...)                                                                           \
  _STREAM_STREAMABLE_ROS(name, name##BaseWrapper, STREAM_BASE(Pose2D), (header), __VA_ARGS__)
#else
#define STREAMABLE_WITH_POSE2D(name, header, ...)                                                                           \
  _STREAM_STREAMABLE(name, name##BaseWrapper, STREAM_BASE(Pose2D), (header), __VA_ARGS__)
#endif

#ifdef ENABLE_ROS
#define STREAMABLE_DECLARE_WITH_POSE2D(name)                                                                                \
  using name##BaseWrapperType = internal::ros_msg_wrapper<nomadz_msgs::msg::name>::internal_type;                           \
  class name##BaseWrapper : public Pose2D, public name##BaseWrapperType {                                                   \
  public:                                                                                                                   \
    name##BaseWrapper() : Pose2D(ROSType::pose, internal::ref_init_tag{}) {}                                                \
    name##BaseWrapper(const name##BaseWrapper& o) : name##BaseWrapper() { *this = o; }                                      \
                                                                                                                            \
    using ROSType = name##BaseWrapperType::ROSType;                                                                         \
    using ROSDeclType = name##BaseWrapperType::ROSDeclType;                                                                 \
  };
#else
#define STREAMABLE_DECLARE_WITH_POSE2D(name)                                                                                \
  namespace nomadz_msgs {                                                                                                   \
    namespace msg {                                                                                                         \
      template <typename> struct name##_;                                                                                   \
      using name = name##_<std::allocator<void>>;                                                                           \
    }                                                                                                                       \
  }                                                                                                                         \
  using name##BaseWrapperType = internal::ros_msg_wrapper<nomadz_msgs::msg::name>::internal_type;                           \
  class name##BaseWrapper : public Pose2D, public name##BaseWrapperType {                                                   \
  public:                                                                                                                   \
    using ROSDeclType = name##BaseWrapperType::ROSDeclType;                                                                 \
  };
#endif

template <class T> class Range;

/** representation for 2D Transformation and Position (Location + Orientation)*/
class Pose2D : public Streamable, public internal::pose2d_ros_wrapper {
public:
  virtual void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(rotation);
    STREAM(translation);
    STREAM_REGISTER_FINISH;
  }

  /** Rotation as an angle*/
  float& rotation;

  /** translation as an vector2*/
  Vector2<> translation;

  /** Default constructors */
#ifdef ENABLE_ROS
  Pose2D()
      : rotation(internal::pose2d_ros_wrapper::base_type::rotation),
        translation(internal::pose2d_ros_wrapper::base_type::translation, internal::ref_init_tag{}) {
    rotation = 0;
    translation = Vector2<>(0, 0);
  }
#else
  Pose2D() : rotation(internal::pose2d_ros_wrapper::base_type::rotation) {
    rotation = 0;
    translation = Vector2<>(0, 0);
  }
#endif
  template <typename U>
  inline Pose2D(U& ref, internal::ref_init_tag t) : rotation(ref.rotation), translation(ref.translation, t) {
    rotation = 0;
    translation = Vector2<>(0, 0);
  }
  template <typename U>
  inline Pose2D(U& ref, internal::ref_alias_tag t) : rotation(ref.rotation), translation(ref.translation, t) {}

  /** constructor from rotation and translation
   * \param rotation rotation (float)
   * \param translation translation (Vector2)
   */
  Pose2D(const float rotation, const Vector2<>& translation) : Pose2D() {
    this->rotation = rotation;
    this->translation = translation;
  }

  /** constructor from rotation and translation
   * \param rot rotation (float)
   * \param x translation.x (float)
   * \param y translation.y (float)
   */
  Pose2D(const float rot, const float x, const float y) : Pose2D() {
    rotation = rot;
    translation = Vector2<>(x, y);
  }

  /** constructor from rotation
   * \param rotation rotation (float)
   */
  Pose2D(const float rotation) : Pose2D() {
    this->rotation = rotation;
    translation = Vector2<>(0, 0);
  }

  /** constructor from translation
   * \param translation translation (Vector2)
   */
  Pose2D(const Vector2<>& translation) : Pose2D() {
    rotation = 0;
    this->translation = translation;
  }

  /** constructor from translation
   * \param translation translation (Vector2)
   */
  Pose2D(const Vector2<int>& translation) : Pose2D() {
    rotation = 0;
    this->translation = Vector2<>(static_cast<float>(translation.x), static_cast<float>(translation.y));
  }

  /** constructor from two translation values
   * \param x translation x component
   * \param y translation y component
   */
  Pose2D(const float x, const float y) : Pose2D() {
    rotation = 0;
    translation = Vector2<>(x, y);
  }

  /** Assignment operator
   *\param other The other Pose2D that is assigned to this one
   *\return A reference to this object after the assignment.
   */
  Pose2D& operator=(const Pose2D& other) {
    rotation = other.rotation;
    translation = other.translation;
    return *this;
  }

  /** Copy constructor
   *\param other The other vector that is copied to this one
   */
  Pose2D(const Pose2D& other) : Pose2D() { *this = other; }

  /** Multiplication of a Vector2 with this Pose2D.
   *
   * Same as (point.rotate(Pose2D.rotation) + Pose2D.translation)
   *
   *\param point The Vector2 that will be multiplicated with this Pose2D
   *\return The resulting Vector2
   */
  Vector2<> operator*(const Vector2<>& point) const {
    float s = std::sin(rotation);
    float c = std::cos(rotation);
    return (Vector2<>(point.x * c - point.y * s, point.x * s + point.y * c) + translation);
  }

  /** Comparison of another pose with this one.
   *\param other The other pose that will be compared to this one
   *\return Whether the two poses are equal.
   */
  bool operator==(const Pose2D& other) const { return ((translation == other.translation) && (rotation == other.rotation)); }

  /** Comparison of another pose with this one.
   *\param other The other pose that will be compared to this one
   *\return Whether the two poses are unequal.
   */
  bool operator!=(const Pose2D& other) const { return !(*this == other); }

  /**Concatenation of this pose with another pose.
   *\param other The other pose that will be concatenated to this one.
   *\return A reference to this pose after concatenation.
   */
  Pose2D& operator+=(const Pose2D& other) {
    translation = *this * other.translation;
    rotation += other.rotation;
    rotation = normalize(rotation);
    return *this;
  }

  /**A concatenation of this pose and another pose.
   *\param other The other pose that will be concatenated to this one.
   *\return The resulting pose.
   */
  Pose2D operator+(const Pose2D& other) const { return Pose2D(*this) += other; }

  /**Difference of this pose relative to another pose. So if A+B=C is the addition/concatenation, this calculates C-A=B.
   *\param other The other pose that will be used as origin for the new pose.
   *\return A reference to this pose after calculating the difference.
   */
  Pose2D& operator-=(const Pose2D& other) {
    translation -= other.translation;
    Pose2D p(-other.rotation);
    return *this = p + *this;
  }

  /**Difference of this pose relative to another pose.
   *\param other The other pose that will be used as origin for the new pose.
   *\return The resulting pose.
   */
  Pose2D operator-(const Pose2D& other) const { return Pose2D(*this) -= other; }

  /**Concatenation of this pose with another pose
   *\param other The other pose that will be concatenated to this one.
   *\return A reference to this pose after concatenation
   */
  Pose2D& conc(const Pose2D& other) { return *this += other; }

  /**Translate this pose by a translation vector
   *\param trans Vector to translate with
   *\return A reference to this pose after translation
   */
  Pose2D& translate(const Vector2<>& trans) {
    translation = *this * trans;
    return *this;
  }

  /**Translate this pose by a translation vector
   *\param x x component of vector to translate with
   *\param y y component of vector to translate with
   *\return A reference to this pose after translation
   */
  Pose2D& translate(const float x, const float y) {
    translation = *this * Vector2<>(x, y);
    return *this;
  }

  /**Rotate this pose by a rotation
   *\param angle Angle to rotate.
   *\return A reference to this pose after rotation
   */
  Pose2D& rotate(const float angle) {
    rotation += angle;
    return *this;
  }

  /** Calculates the inverse transformation from the current pose
   * @return The inverse transformation pose.
   */
  Pose2D invert() const {
    const float& invRotation = -rotation;
    return Pose2D(invRotation, (Vector2<>() - translation).rotate(invRotation));
  }

  /** Calculates a scaled pose with a scaler
   * @return The scaled pose.
   */
  Pose2D scale(const float scaler) const { return Pose2D(rotation * scaler, translation * scaler); }

  /** Calculates a the pose ratio with respect to an reference pose.
   * @return The pose ratio with respect to an reference pose.
   */
  Pose2D elementwiseDiv(const Pose2D referencePose) const {
    return Pose2D(rotation / referencePose.rotation,
                  translation.x / referencePose.translation.x,
                  translation.y / referencePose.translation.y);
  }

  /** Calculates a the scaled pose with respect to an reference pose ratio.
   * @return The scaled pose.
   */
  Pose2D elementwiseMul(const Pose2D referencePoseRatio) const {
    return Pose2D(rotation * referencePoseRatio.rotation,
                  translation.x * referencePoseRatio.translation.x,
                  translation.y * referencePoseRatio.translation.y);
  }

  /** Calculates the elementwise difference to a reference pose.
   * @return The diff pose.
   */
  Pose2D elementwiseAdd(const Pose2D referencePose) const {
    return Pose2D(rotation + referencePose.rotation, translation + referencePose.translation);
  }

  /** Calculates the elementwise sum.
   * @return The sum pose.
   */
  Pose2D elementwiseSub(const Pose2D referencePoseRatio) const {
    return Pose2D(rotation - referencePoseRatio.rotation, translation - referencePoseRatio.translation);
  }

  /**
   * The function creates a random pose.
   * @param x The range for x-values of the pose.
   * @param y The range for y-values of the pose.
   * @param angle The range for the rotation of the pose.
   */
  static Pose2D random(const Range<float>& x, const Range<float>& y, const Range<float>& angle);
};
