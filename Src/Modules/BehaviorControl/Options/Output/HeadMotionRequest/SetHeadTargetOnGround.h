/**
 * @file SetHeadTargetOnGround
 *
 * Sets all members of the HeadMotionRequest representation for positioning the robot's head pointing towards a given point
 * on the field.
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

option(SetHeadTargetOnGround,
       const Vector3<>& target,
       HeadMotionRequest::CameraControlMode camera = HeadMotionRequest::autoCamera,
       bool watchField = false,
       float speed = pi) {

  /** Set the head motion request. */
  initial_state(setRequest) {
    action {
      theHeadMotionRequest.mode = HeadMotionRequest::targetOnGroundMode;
      theHeadMotionRequest.cameraControlMode = camera;
      theHeadMotionRequest.watchField = watchField;
      theHeadMotionRequest.target = target;
      theHeadMotionRequest.speed = speed;
    }
  }
}
