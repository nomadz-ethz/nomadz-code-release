#include "nomadz_behavior/checks.hpp"

#include "nomadz_behavior/check_nodes/check_nodes.hpp"

namespace nomadz_behavior {

  CheckNodes::CheckNodes(BT::BehaviorTreeFactory& factory) {
    // Initialize the condition tree nodes
    factory.registerNodeType<Penalized>("Penalized");
    factory.registerNodeType<Standing>("Standing");
    factory.registerNodeType<Localized>("Localized");
    factory.registerNodeType<Lost>("Lost");
    factory.registerNodeType<BallFound>("BallFound");
    factory.registerNodeType<BallLost>("BallLost");
    factory.registerNodeType<HasBallLock>("HasBallLock");
    factory.registerNodeType<BallFree>("BallFree");
    factory.registerNodeType<HasBall>("HasBall");
    factory.registerNodeType<IsAligned>("IsAligned");
    factory.registerNodeType<IsHighestActivePlayer>("IsHighestActivePlayer");
    factory.registerNodeType<CanSaveShot>("CanSaveShot");
    factory.registerNodeType<IsInsideArea>("IsInsideArea");
  }
} // namespace nomadz_behavior
