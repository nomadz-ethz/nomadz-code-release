/**
 * @file PassHelperProviderParameters.h
 *
 * Set of all user tuned parameters. Before a match on a new field, all these parameters should be retuned.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#pragma once

#include "Core/Streams/AutoStreamable.h"
#include "Representations/Configuration/FieldDimensions.h"

// Parameters for the state machine of the skill Pass
STREAMABLE(PassStateMachineParameters,
           {
             ,
             (int)(5000)timeLimit, // The time after which the robot can actually switch state from passDecision. Needed else
                                   // as soon as it kick the ball, the robot will
                                   // save the option
           });

// Parameter needed by the PassHelperProvider and pass skills
STREAMABLE(
  PassHelperProviderParameters,
  {
    ,
    // Used by the PassHelperProvider

    (float)(500)robotShadowRadius, // The radius of the circle that we use to calculate the shadow

    (float)(1000)yDistanceShot, // The y distance from the goal post to the limit that distinguish the lateral-backward zone
                                // from the shoot zone (please see documentation)

    (int)(100)distancePoints, // Distance between two consecutive points in x and y direction [mm]

    (int)(5)squareHalfDimension, // The half of the point that we will consider as possible candidate where to pass the ball
                                 // (in this case from the robot we are
    // going to consider a square of validity of (2*squareHalfDimension)*distancePoints = (2*5)*100 [mm] = 1m)

    (int)(1000)minDistFrontKick, // The minimum distance that there should be between the passing robot and the decided
                                 // point, if the kick is done by front

    (int)(1500)maxDistFrontKick, // The maximum distance that there should be between the passing robot and the decided point
                                 // = max of the distance that we can reach
                                 // with a front kick

    (int)(800)minDistYLateralKick, // The minimum distance that there should be between the passing robot and the decided
                                   // point, if the kick is done by lateral

    (int)(2000)maxDistYLateralKick, // The maximum distance that there should be between the passing robot and the decided
                                    // point = max of the distance that we can reach
                                    // with a lateral kick

    (int)(900)halfBandXDistanceLateralKick, // The maximum of distance along the x axes

    (int)(1400)halfBandDistanceForwardPass, // Half of the distance in y that divide the field in 3 part. This is used for
                                            // the decidePassPointForward() function.
                                            // Please for more information refer to the documentation
                                            // initialize always > 0

    // Used by the Pass skills

    (int)(700)startPassCalculation, // If distance Robot-ball smaller than startPassCalculation, the robot will start the
                                    // calculation for pass This in order to spare GPU power for other calculation

    (int)(300)stopPassCalculation, // If distance Robot-ball smaller than stopPassCalculation, according to the last output
                                   // of PassHelperProvider, the robot will
    // stop the calculation and use this output as condition to pass or shot to the opponent goal the ball.
    // This is needed since the robot could then continuously change target of the pass, and therefore it would continue to
    // change position
    // to kick the ball in another direction without actually kick the ball

    (bool)(true)continuousCalculation, // True if we want that the module work continuously. This is usefull only if we want
                                       // to move around the ball and see which
                                       // points will be chosen as true or false.
                                       // In a match, set it to false, so that only the knipser will calculate the
                                       // Pass Point and when it will start the kick action will stop to calculate
    // Note: during a match set it always to false. Else the robot will continue to calculate, and since the word around it
    // will continuously
    // change, the robot will continuously change its mind in what to do = the robot will never perform a pass, but just
    // continue to
    // change its position, or if before kick the pass point that was found is anymore available, it will kick to
    // (10000;10000)

    (PassStateMachineParameters)stateMachineParameters,
    //  (int)(200) minDistOpponents,		// Minimum distance between robot and opponent (At the moment not used)

  });

// This struct defines the areas where different type of pass are used
// NOTE: the function void PassHelperProvider::decidePassPoint()
// will work only if the shape of the different area will stay the same (also with other measure). If you want to change
// shape, then you have to change
// the logic in the void PassHelperProvider::decidePassPoint() function
struct PassZones {
  // Define the different areas that contain other pass type
  /*********** Forward ***********/
  Vector2<> Forward_Area[4];
  /*********** Forward / Lateral ***********/
  Vector2<> Forward_Lateral_Area[4];
  /*********** shot ***********/
  Vector2<> Shot_Area[4];
  /*********** Backward / Lateral Left ***********/
  Vector2<> Lateral_Backward_Left_Area[4];
  /*********** Backward / Lateral Right ***********/
  Vector2<> Lateral_Backward_Right_Area[4];

  // Constructor that initialize the different pass areas
  // NOTE: in order to see in which area is the ball, when we check for the x direction for the Forward and Forward+Lateral
  // areas, the second element of the array is
  // used as reference (Forward_Area[1].x, Forward_Lateral_Area[1])
  // This means that in
  // void PassHelperProvider::decidePassPoint()
  // We implicitly assume that these two zones are rectangular
  // If you want to change shape, then you have to modify the logic of the function void
  // PassHelperProvider::decidePassPoint()
  PassZones(FieldDimensions theFieldDimensions, PassHelperProviderParameters parameters) {
    // Initialize the different areas (pass Zones)
    // This will cause that for different area we use different type of pass
    // Please refer to the documentation for more info

    // Order of the element:

    //	1---------2
    //  |		  |
    //	|		  |
    //	|		  |
    //	4---------3

    /*********** Forward ***********/
    Forward_Area[0] = Vector2<>(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftSideline);
    Forward_Area[1] = Vector2<>(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosLeftSideline);
    Forward_Area[2] = Vector2<>(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosRightSideline);
    Forward_Area[3] = Vector2<>(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightSideline);

    /*********** Forward / Lateral ***********/
    Forward_Lateral_Area[0] = Vector2<>(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosLeftSideline);
    Forward_Lateral_Area[1] = Vector2<>(theFieldDimensions.xPosOpponentPenaltyMark, theFieldDimensions.yPosLeftSideline);
    Forward_Lateral_Area[2] = Vector2<>(theFieldDimensions.xPosOpponentPenaltyMark, theFieldDimensions.yPosRightSideline);
    Forward_Lateral_Area[3] = Vector2<>(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosRightSideline);

    /*********** shot ***********/
    Shot_Area[0] =
      Vector2<>(theFieldDimensions.xPosOpponentPenaltyMark, theFieldDimensions.yPosLeftGoal + parameters.yDistanceShot);
    Shot_Area[1] = Vector2<>(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal);
    Shot_Area[2] = Vector2<>(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal);
    Shot_Area[3] =
      Vector2<>(theFieldDimensions.xPosOpponentPenaltyMark, theFieldDimensions.yPosRightGoal - parameters.yDistanceShot);

    /*********** Backward / Lateral Left ***********/
    Lateral_Backward_Left_Area[0] =
      Vector2<>(theFieldDimensions.xPosOpponentPenaltyMark, theFieldDimensions.yPosLeftSideline);
    Lateral_Backward_Left_Area[1] = Vector2<>(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftSideline);
    Lateral_Backward_Left_Area[2] = Vector2<>(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal);
    Lateral_Backward_Left_Area[3] =
      Vector2<>(theFieldDimensions.xPosOpponentPenaltyMark, theFieldDimensions.yPosLeftGoal + parameters.yDistanceShot);

    /*********** Backward / Lateral Right ***********/
    Lateral_Backward_Right_Area[0] =
      Vector2<>(theFieldDimensions.xPosOpponentPenaltyMark, theFieldDimensions.yPosRightGoal - parameters.yDistanceShot);
    Lateral_Backward_Right_Area[1] = Vector2<>(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal);
    Lateral_Backward_Right_Area[2] =
      Vector2<>(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightSideline);
    Lateral_Backward_Right_Area[3] =
      Vector2<>(theFieldDimensions.xPosOpponentPenaltyMark, theFieldDimensions.yPosRightSideline);
  }
};
