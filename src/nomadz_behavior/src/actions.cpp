#include "nomadz_behavior/actions.hpp"

#include "nomadz_behavior/action_nodes/motion_nodes/walk_actions.hpp"
#include "nomadz_behavior/action_nodes/motion_nodes/special_actions.hpp"
#include "nomadz_behavior/action_nodes/motion_nodes/head_motion_action.hpp"
#include "nomadz_behavior/action_nodes/helper_nodes/compute_positions.hpp"
#include "nomadz_behavior/action_nodes/helper_nodes/state_provider.hpp"
#include "nomadz_behavior/action_nodes/high_level_actions.hpp"

namespace nomadz_behavior {

  ActionNodes::ActionNodes(BT::BehaviorTreeFactory& factory) {
    // Initialize the condition tree nodes
    factory.registerNodeType<GameStatusProvider>("GameStatusProvider");
    factory.registerNodeType<WalkAtSpeed>("WalkAtSpeed");
    factory.registerNodeType<WalkToTarget>("WalkToTarget");
    factory.registerNodeType<Alignment>("Alignment");
    factory.registerNodeType<GetTarget>("GetTarget");
    factory.registerNodeType<Penalty>("Penalty");
    factory.registerNodeType<Recovery>("Recovery");
    factory.registerNodeType<Relax>("Relax");
    factory.registerNodeType<SpecialAction>("SpecialAction");
    factory.registerNodeType<ApplySkill>("ApplySkill");
    factory.registerNodeType<Engage>("Engage");
    factory.registerNodeType<RoleProvider>("RoleProvider");
    factory.registerNodeType<SaveShot>("SaveShot");
    factory.registerNodeType<LookAtBall>("LookAtBall");
    factory.registerNodeType<LookAtDirection>("LookAtDirection");
    factory.registerNodeType<Kick>("Kick");
    factory.registerNodeType<LogUndefinedState>("LogUndefinedState");
    factory.registerNodeType<FootAlignment>("FootAlignment");
  }
} // namespace nomadz_behavior
