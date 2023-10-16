/**
 * @file Pose3D.h
 *
 * Contains class Pose3D
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * and Max Risler
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "RotationMatrix.h"

#ifdef ENABLE_ROS
#include "nomadz_msgs/msg/pose3_d.hpp"
namespace internal {
  class pose3d_ros_wrapper : public ROSMessage<nomadz_msgs::msg::Pose3D> {
  public:
    using base_type = nomadz_msgs::msg::Pose3D;
    static constexpr bool ros_compatible = true;
  };
} // namespace internal
#else
namespace internal {
  class pose3d_ros_wrapper : public ROSMessage<void, false> {
  public:
    static constexpr bool ros_compatible = false;
  };
} // namespace internal
#endif

#ifdef ENABLE_ROS
#define STREAMABLE_WITH_POSE3D(name, header, ...)                                                                           \
  _STREAM_STREAMABLE_ROS(name, name##BaseWrapper, STREAM_BASE(Pose3D), (header), __VA_ARGS__)
#else
#define STREAMABLE_WITH_POSE3D(name, header, ...)                                                                           \
  _STREAM_STREAMABLE(name, name##BaseWrapper, STREAM_BASE(Pose3D), (header), __VA_ARGS__)
#endif

#ifdef ENABLE_ROS
#define STREAMABLE_DECLARE_WITH_POSE3D(name)                                                                                \
  using name##BaseWrapperType = internal::ros_msg_wrapper<nomadz_msgs::msg::name>::internal_type;                           \
  class name##BaseWrapper : public Pose3D, public name##BaseWrapperType {                                                   \
  public:                                                                                                                   \
    name##BaseWrapper() : Pose3D(ROSType::pose, internal::ref_init_tag{}) {}                                                \
    name##BaseWrapper(const name##BaseWrapper& o) : name##BaseWrapper() { *this = o; }                                      \
                                                                                                                            \
    using ROSType = name##BaseWrapperType::ROSType;                                                                         \
    using ROSDeclType = name##BaseWrapperType::ROSDeclType;                                                                 \
  };
#else
#define STREAMABLE_DECLARE_WITH_POSE3D(name)                                                                                \
  namespace nomadz_msgs {                                                                                                   \
    namespace msg {                                                                                                         \
      template <typename> struct name##_;                                                                                   \
      using name = name##_<std::allocator<void>>;                                                                           \
    }                                                                                                                       \
  }                                                                                                                         \
  using name##BaseWrapperType = internal::ros_msg_wrapper<nomadz_msgs::msg::name>::internal_type;                           \
  class name##BaseWrapper : public Pose3D, public name##BaseWrapperType {                                                   \
  public:                                                                                                                   \
    using ROSDeclType = name##BaseWrapperType::ROSDeclType;                                                                 \
  };
#endif

/** representation for 3D Transformation (Location + Orientation)*/
class Pose3D : public Streamable, public internal::pose3d_ros_wrapper {
protected:
  virtual void serialize(In* in, Out* out) {
    STREAM_REGISTER_BEGIN;
    STREAM(rotation);
    STREAM(translation);
    STREAM_REGISTER_FINISH;
  }

public:
  /** rotation as a RotationMatrix*/
  RotationMatrix rotation;

  /** translation as a Vector3*/
  Vector3<> translation;

/** constructor*/
#ifdef ENABLE_ROS
  Pose3D()
      : rotation(internal::pose3d_ros_wrapper::base_type::rotation, internal::ref_init_tag{}),
        translation(internal::pose3d_ros_wrapper::base_type::translation, internal::ref_init_tag{}) {
    rotation = RotationMatrix();
    translation = Vector3<>(0, 0, 0);
  }
#else
  Pose3D() {
    rotation = RotationMatrix();
    translation = Vector3<>(0, 0, 0);
  }
#endif
  template <typename U>
  inline Pose3D(U& ref, internal::ref_init_tag t) : rotation(ref.rotation, t), translation(ref.translation, t) {
    rotation = RotationMatrix();
    translation = Vector3<>(0, 0, 0);
  }
  template <typename U>
  inline Pose3D(U& ref, internal::ref_alias_tag t) : rotation(ref.rotation, t), translation(ref.translation, t) {}

  /** constructor from rotation and translation
   * \param rot Rotation
   * \param trans Translation
   */
  inline Pose3D(const RotationMatrix& rot, const Vector3<>& trans) : Pose3D() {
    rotation = rot;
    translation = trans;
  }

  /** constructor from rotation
   * \param rot Rotation
   */
  inline Pose3D(const RotationMatrix& rot) : Pose3D() { rotation = rot; }

  /** constructor from translation
   * \param trans Translation
   */
  inline Pose3D(const Vector3<>& trans) : Pose3D() { translation = trans; }

  /** constructor from three translation values
   * \param x translation x component
   * \param y translation y component
   * \param z translation z component
   */
  inline Pose3D(const float x, const float y, const float z) : Pose3D() { translation = Vector3<>(x, y, z); }

  /** Copy constructor
   *\param other The other vector that is copied to this one
   */
  inline Pose3D(const Pose3D& other) : Pose3D() {
    rotation = other.rotation;
    translation = other.translation;
  }

  /** Assignment operator
   *\param other The other Pose3D that is assigned to this one
   *\return A reference to this object after the assignment.
   */
  inline Pose3D& operator=(const Pose3D& other) {
    rotation = other.rotation;
    translation = other.translation;
    return *this;
  }

  /** Multiplication with Point
   *\param point (Vector3&lt;float&gt;)
   */
  inline Vector3<> operator*(const Vector3<>& point) const { return rotation * point + translation; }

  /** Comparison of another vector with this one.
   *\param other The other vector that will be compared to this one
   *\return Whether the two vectors are equal.
   */
  inline bool operator==(const Pose3D& other) const {
    return translation == other.translation && rotation == other.rotation;
  }

  /** Comparison of another vector with this one.
   *\param other The other vector that will be compared to this one
   *\return Whether the two vectors are unequal.
   */
  inline bool operator!=(const Pose3D& other) const {
    return translation != other.translation || rotation != other.rotation;
  }

  /**Concatenation of this pose with another pose
   *\param other The other pose that will be concatenated to this one.
   *\return A reference to this pose after concatenation
   */
  inline Pose3D& conc(const Pose3D& other) {
    translation = *this * other.translation;
    rotation *= other.rotation;
    return *this;
  }

  /** Calculates the inverse transformation from the current pose
   * @return The inverse transformation pose.
   */
  inline Pose3D invert() const {
    Pose3D result;
    result.rotation = rotation.invert();
    result.translation = result.rotation * (-translation);
    return result;
  }

  /**Translate this pose by a translation vector
   *\param trans Vector to translate with
   *\return A reference to this pose after translation
   */
  inline Pose3D& translate(const Vector3<>& trans) {
    translation = *this * trans;
    return *this;
  }

  /**Translate this pose by a translation vector
   *\param x x component of vector to translate with
   *\param y y component of vector to translate with
   *\param z z component of vector to translate with
   *\return A reference to this pose after translation
   */
  inline Pose3D& translate(const float x, const float y, const float z) {
    translation = *this * Vector3<>(x, y, z);
    return *this;
  }

  /**Rotate this pose by a rotation
   *\param rot Rotationmatrix to rotate with
   *\return A reference to this pose after rotation
   */
  inline Pose3D& rotate(const RotationMatrix& rot) {
    rotation *= rot;
    return *this;
  }

  /**Rotate this pose around its x-axis
   *\param angle angle to rotate with
   *\return A reference to this pose after rotation
   */
  inline Pose3D& rotateX(const float angle) {
    rotation.rotateX(angle);
    return *this;
  }

  /**Rotate this pose around its y-axis
   *\param angle angle to rotate with
   *\return A reference to this pose after rotation
   */
  inline Pose3D& rotateY(const float angle) {
    rotation.rotateY(angle);
    return *this;
  }

  /**Rotate this pose around its z-axis
   *\param angle angle to rotate with
   *\return A reference to this pose after rotation
   */
  inline Pose3D& rotateZ(const float angle) {
    rotation.rotateZ(angle);
    return *this;
  }

  inline Pose3D operator+(const Vector3<>& trans) const { return Pose3D(rotation, *this * trans); }

  inline Pose3D operator*(const RotationMatrix& rot) const { return Pose3D(rotation * rot, translation); }

  inline Pose3D& operator*=(const Pose3D& other) {
    translation = *this * other.translation;
    rotation *= other.rotation;
    return *this;
  }

  inline Pose3D& operator*=(const RotationMatrix& rot) {
    rotation *= rot;
    return *this;
  }

  inline Pose3D& operator+=(const Vector3<>& trans) {
    translation = *this * trans;
    return *this;
  }

  inline Pose3D operator*(const Pose3D& other) const { return Pose3D(rotation * other.rotation, *this * other.translation); }
};

inline Pose3D operator*(const RotationMatrix& rotation, const Pose3D& pose) {
  return Pose3D(rotation * pose.rotation, rotation * pose.translation);
}
