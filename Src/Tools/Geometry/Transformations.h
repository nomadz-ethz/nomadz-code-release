/**
 * @file Transformations.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note This file is based on "Geometry.h" from the B-Human code release
 * 2013. The file has been heavily modified by the NomadZ team, authors have
 * not been tracked.
 */

#pragma once

#include "Core/Math/Vector2.h"
#include "Core/Math/Vector3.h"
#include "Shapes.h"

class CameraInfo;
class CameraMatrix;
class Pose2D;
class Pose3D;

namespace Geometry {

  /**
   * Function does the transformation from 2d relative robot coordinates
   * to absolute field coordinates.
   * @param rp current Robot Pose.
   * @param x relative x-coordinate of ball (relative to robot)
   * @param y relative y-coordinate of ball (relative to robot)
   * @return Returns the ball positon in absolute coordinates
   */
  Vector2<> relative2FieldCoord(const Pose2D& rp, float x, float y);

  /**
   * Function does the transformation from 2d relative robot coordinates
   * to absolute field coordinates.
   * @param rp current Robot Pose.
   * @param relPosOnField relative position on the field (relative to robot)
   * @return Returns the ball positon in absolute coordinates
   */
  Vector2<> relative2FieldCoord(const Pose2D& rp, const Vector2<>& relPosOnField);

  /**
   * Function does the transformation from 2d field coordinates
   * to coordinates relative to the robot.
   * @param robotPose current Robot Pose.
   * @param fieldCoord
   * @return Returns the positon in relative
   */
  Vector2<> fieldCoord2Relative(const Pose2D& robotPose, const Vector2<>& fieldCoord);

  /**
   * The function determines how far an object is away depending on its real size and the size in the image.
   * @param cameraInfo Information about the camera (opening angles, resolution, etc.).
   * @param sizeInReality The real size of the object.
   * @param sizeInPixels The size in the image.
   * @return The distance between camera and object.
   */
  float getDistanceBySize(const CameraInfo& cameraInfo, float sizeInReality, float sizeInPixels);

  /**
   * The function determines how far an object is away depending on its real size and the size in the image
   * along with its center position, using camera intrinsic parameters.
   * @param cameraInfo Class containing the intrinsic paramaters
   * @param sizeInReality The real size of the object.
   * @param sizeInPixels The size in the image.
   * @param centerX X coordinate (in image reference) of object's baricenter.
   * @param centerY Y coordinate (in image reference) of object's baricenter.
   * @return The distance between camera and object.
   */
  float
  getDistanceBySize(const CameraInfo& cameraInfo, float sizeInReality, float sizeInPixels, float centerX, float centerY);

  /**
   * Calculates where a pixel in the image lies on the ground (relative to the robot).
   * @param x Specifies the x-coordinate of the pixel.
   * @param y Specifies the y-coordinate of the pixel.
   * @param cameraMatrix The camera matrix of the image.
   * @param cameraInfo The camera info of the image.
   * @param pointOnField The resulting point.
   */
  bool calculatePointOnField(
    const int x, const int y, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2<>& pointOnField);

  bool calculatePointOnField(
    const float x, const float y, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2<>& pointOnField);

  inline bool calculatePointOnField(const Vector2<int>& pointInImage,
                                    const CameraMatrix& cameraMatrix,
                                    const CameraInfo& cameraInfo,
                                    Vector2<>& pointOnField) {
    return calculatePointOnField(pointInImage.x, pointInImage.y, cameraMatrix, cameraInfo, pointOnField);
  }

  bool calculatePointOnField(const Vector2<>& image,
                             const float& fieldZ,
                             const CameraMatrix& cameraMatrix,
                             const CameraInfo& cameraInfo,
                             Vector3<>& field);

  bool calculatePointOnField(const Vector2<>& point,
                             const CameraMatrix& cameraMatrix,
                             const CameraInfo& cameraInfo,
                             Vector2<>& pointOnField);

  /**
   * Calculates where a pixel in the image lies on the ground (relative to the robot).
   * @param x Specifies the x-coordinate of the pixel.
   * @param y Specifies the y-coordinate of the pixel.
   * @param cameraMatrix The camera matrix of the image.
   * @param cameraInfo The camera info of the image.
   * @param pointOnField The resulting point.
   */
  bool calculatePointOnField(
    const int x, const int y, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, Vector2<int>& pointOnField);

  /**
   * Calculates where a relative point on the ground appears in an image.
   * @param point The coordinates of the point relative to the robot's origin.
   * @param cameraMatrix The camera matrix of the image.
   * @param cameraInfo The camera info of the image.
   * @param pointInImage The resulting point.
   * @return The result is valid, i.e. the point is in front of the camera. That
   *         still does not mean that the point is within the bounds of the image.
   */
  bool calculatePointInImage(const Vector2<>& point,
                             const CameraMatrix& cameraMatrix,
                             const CameraInfo& cameraInfo,
                             Vector2<int>& pointInImage);

  bool calculatePointsInImage(const std::vector<Vector2<>>& points,
                              const CameraMatrix& cameraMatrix,
                              const CameraInfo& cameraInfo,
                              std::vector<Vector2<int>>& pointsInImage);

  /**
   * Calculates where a relative point on the ground appears in an image.
   * @param point The coordinates of the point relative to the robot's origin.
   * @param cameraMatrix The camera matrix of the image.
   * @param cameraInfo The camera info of the image.
   * @param pointInImage The resulting point.
   * @return The result is valid, i.e. the point is in front of the camera. That
   *         still does not mean that the point is within the bounds of the image.
   */
  bool calculatePointInImage(const Vector3<>& point,
                             const CameraMatrix& cameraMatrix,
                             const CameraInfo& cameraInfo,
                             Vector2<>& pointInImage);

  /**
   * Calculates where a relative 3-D point appears in an image.
   * @param pointInWorld The coordinates of the point relative to the robot's origin.
   * @param cameraMatrix The camera matrix of the image.
   * @param cameraInfo The camera info of the image.
   * @param pointInImage The resulting point.
   * @return The result is valid, i.e. the point is in front of the camera. That
   *         still does not mean that the point is within the bounds of the image.
   */
  bool calculatePointInImage(const Vector3<>& pointInWorld,
                             const CameraMatrix& cameraMatrix,
                             const CameraInfo& cameraInfo,
                             Vector2<int>& pointInImage);

  /**
   * Calculates where a relative line segment on the ground (p1 - p2) appears in an image (img1 - img2).
   * @param p1 Endpoint 1 of the line segment.
   * @param p2 Endpoint 2 of the line segment.
   * @param cameraMatrixInv The inverse of the camera matrix for the image (e.g. theCameraMatrix.invert()).
   * @param cameraInfo The camera info for the image.
   * @param img1 Endpoint 1 of the line segment in the image.
   * @param img2 Endpoint 2 of the line segment in the image.
   * @return The result is valid, i.e. at least part of the line segment is in front of the camera.
   *          Could still be outside the bounds of the image though.
   */
  bool calculateLineInImage(const Vector2<>& p1,
                            const Vector2<>& p2,
                            const Pose3D& cameraMatrixInv,
                            const CameraInfo& cameraInfo,
                            Vector2<int>& img1,
                            Vector2<int>& img2);

  bool calculateLineInImage(
    const Line& line, const Pose3D& cameraMatrixInv, const CameraInfo& cameraInfo, Vector2<int>& img1, Vector2<int>& img2);

  /**
   * The function determines how big an object appears in the image depending on its distance and size.
   * @param cameraInfo Object containing camera paramters.
   * @param sizeInReality The real size of the object.
   * @param distance The distance to the object.
   * @return The size as it would appear in the image.
   */
  float getSizeByDistance(const CameraInfo& cameraInfo, float sizeInReality, float distance);

  /**
   * The function calculates the horizon.
   * @param cameraMatrix The camera matrix.
   * @param cameraInfo Object containing camera parameters.
   * @return The line of the horizon in the image.
   */
  Line calculateHorizon(const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo);

  /**
   * Calculates the angle size for a given pixel size.
   */
  float pixelSizeToAngleSize(float pixelSize, const CameraInfo& cameraInfo);

  /**
   * Calculates the pixel size for a given angle size.
   */
  float angleSizeToPixelSize(float angleSize, const CameraInfo& cameraInfo);

  void calculateAnglesForPoint(const Vector2<>& point,
                               const CameraMatrix& cameraMatrix,
                               const CameraInfo& cameraInfo,
                               Vector2<>& angles);

  std::vector<Vector2<>> computeFovQuadrangle(const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo);

  /**
   * Calculates the angle between a pose and a position
   * @param from The base pose.
   * @param to The other position.
   * @return the angle from the pose to the position.
   */
  float angleTo(const Pose2D& from, const Vector2<>& to);

  /**
   * Calculates the distance from a pose to a position
   * @param from The base pose.
   * @param to The other position.
   * @return the distance from the pose to the position.
   */
  float distanceTo(const Pose2D& from, const Vector2<>& to);

  /**
   * Calculates the relative vector from a pose to a position
   * @param from The base pose.
   * @param to The other position.
   * @return the vector from the pose to the position.
   */
  Vector2<> vectorTo(const Pose2D& from, const Vector2<>& to);

  /**
   * Estimates how stretched out a pixel in the image becomes when projected on to the field.
   * Higher values mean more horizontal stretching, lower values mean more vertical stretching
   * Assumes camera roll is zero!
   * @param x Specifies the x-coordinate of the pixel.
   * @param y Specifies the y-coordinate of the pixel.
   * @param cameraMatrix The camera matrix of the image.
   * @param cameraInfo The camera info of the image.
   * @param ratio The result.
   * @return Whether a ratio could be calculated (usually false if (x, y) is in the sky)
   */
  bool calculatePixelRatioOnField(
    const float x, const float y, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, float& ratio);

  bool
  estimateInvPixelRatioOnField(const float y, const CameraMatrix& cameraMatrix, const CameraInfo& cameraInfo, float& ratio);
} // namespace Geometry