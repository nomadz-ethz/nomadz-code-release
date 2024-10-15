#include "nomadz_motion_control/walk_engine/step_pattern_collection.hpp"

namespace side = nomadz_definitions::side;
using nomadz_core::Pose2D;
using nomadz_core::toAffine2;
using nomadz_core::toPose2D;

namespace nomadz_motion_control {
  // TODO(Emilio, Zichong): Implement the actually pattern logics for the following functions
  std::vector<Step> generateDefaultStepPatterns(const Pose2D& target_frame) {
    std::vector<Step> target_steps;
    Step step_i;
    step_i.side = side::LEFT;
    step_i.step_duration = 0.25F;
    step_i.leg_pose = toPose2D(toAffine2(target_frame) * makeAffine2(0.1F, 0.1F, 0.0F));
    target_steps.push_back(step_i);
    return target_steps;
  }

  std::vector<Step> generateOmniStepPatterns(const Pose2D& target_frame) {
    std::vector<Step> target_steps;
    side::Side kick_foot_side;
    if (target_frame.theta > 0.2F) {
      kick_foot_side = side::RIGHT;

    } else if (target_frame.theta < -0.2F) {
      kick_foot_side = side::LEFT;
    } else {
      kick_foot_side = (target_frame.y > 0.0F) ? side::LEFT : side::RIGHT;
    }
    float support_foot_offset_y = (kick_foot_side == side::LEFT) ? -0.105F : 0.105F;
    Step step_3{kick_foot_side, 0.25F, toPose2D(toAffine2(target_frame) * makeAffine2(-BALL_RADIUS / 2.F, 0.0F, 0.0F))};
    step_3.leg_pose.theta = 0.0F;

    Step step_2{side::mirror(kick_foot_side), 0.25F, step_3.leg_pose};
    step_2.leg_pose.x -= 0.06F;
    step_2.leg_pose.y += support_foot_offset_y;
    Step step_4{side::mirror(kick_foot_side), 0.25F, step_2.leg_pose};

    Step step_1{
      kick_foot_side, 0.25F, toPose2D(toAffine2(target_frame) * makeAffine2(-BALL_RADIUS / 2.F - 0.12F, 0.0F, 0.0F))};
    step_1.leg_pose.theta = 0.0F;
    Step step_0{side::mirror(kick_foot_side), 0.25F, step_1.leg_pose};
    step_0.leg_pose.x -= 0.02F;
    step_0.leg_pose.y += support_foot_offset_y;

    step_0.leg_pose.x -= FOOT_KICK_OFFSET_X;
    step_1.leg_pose.x -= FOOT_KICK_OFFSET_X;
    step_2.leg_pose.x -= FOOT_KICK_OFFSET_X;
    step_3.leg_pose.x -= FOOT_KICK_OFFSET_X;

    target_steps.push_back(step_1);
    target_steps.push_back(step_2);
    target_steps.push_back(step_3);
    target_steps.push_back(step_4);
    return target_steps;
  }

  std::vector<Step> generateAlignmentPatterns(const Pose2D& target_frame) {
    std::vector<Step> target_steps;
    side::Side kick_foot_side;
    if (target_frame.theta > 0.2F) {
      kick_foot_side = side::RIGHT;

    } else if (target_frame.theta < -0.2F) {
      kick_foot_side = side::LEFT;
    } else {
      kick_foot_side = (target_frame.y > 0.0F) ? side::LEFT : side::RIGHT;
    }
    float support_foot_offset_y = (kick_foot_side == side::LEFT) ? -0.105F : 0.105F;
    Step step_3{kick_foot_side, 0.25F, toPose2D(toAffine2(target_frame) * makeAffine2(-BALL_RADIUS / 2.F, 0.0F, 0.0F))};
    step_3.leg_pose.theta = 0.0F;

    Step step_2{side::mirror(kick_foot_side), 0.25F, step_3.leg_pose};
    step_2.leg_pose.x -= 0.06F;
    step_2.leg_pose.y += support_foot_offset_y;

    Step step_1{
      kick_foot_side, 0.25F, toPose2D(toAffine2(target_frame) * makeAffine2(-BALL_RADIUS / 2.F - 0.12F, 0.0F, 0.0F))};
    step_1.leg_pose.theta = 0.0F;
    Step step_0{side::mirror(kick_foot_side), 0.25F, step_1.leg_pose};
    step_0.leg_pose.x -= 0.02F;
    step_0.leg_pose.y += support_foot_offset_y;

    step_0.leg_pose.x -= FOOT_KICK_OFFSET_X;
    step_1.leg_pose.x -= FOOT_KICK_OFFSET_X;
    step_2.leg_pose.x -= FOOT_KICK_OFFSET_X;
    step_3.leg_pose.x -= FOOT_KICK_OFFSET_X;

    target_steps.push_back(step_1);
    target_steps.push_back(step_2);
    target_steps.push_back(step_3);
    return target_steps;
  }

  std::vector<Step> generateCircleAroundPatterns(const Pose2D& target_frame) {
    std::vector<Step> target_steps;
    Step step_i;
    step_i.side = side::LEFT;
    step_i.step_duration = 0.25F;
    step_i.leg_pose = toPose2D(toAffine2(target_frame) * makeAffine2(0.1F, 0.1F, 0.0F));
    target_steps.push_back(step_i);
    return target_steps;
  }
} // namespace nomadz_motion_control
