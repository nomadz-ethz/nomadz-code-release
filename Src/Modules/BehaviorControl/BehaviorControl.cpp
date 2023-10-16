/**
 * @file BehaviorControl.cpp
 *
 * Implementation of a C-based state machine behavior control module.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2023 BHuman and NomadZ team
 *
 * @note The original authors are Thomas Röfer Tim Laue
 */

#include "BehaviorControl.h"

#include "Tools/Team.h"
#include "Core/Streams/InStreams.h"
#include "Options/Output/MotionRequest/DribblingParameters.h"
#include "Libraries.h"

namespace behavior {
  /**
   * The wrapper for the behavior options.
   */
  class Behavior : public Libraries {
    static PROCESS_WIDE_STORAGE(Behavior) _theInstance; /**< The instance of this behavior used. */

  public:
#include "Cabsl.h"

  private:
    OptionContext::StateType _stateType; /**< The state type of the last option called. */
    unsigned _lastFrameTime;             /**< The time stamp of the last time the behavior was executed. */
    unsigned char _depth;                /**< The depth level of the current option. Used for sending debug messages. */

    DribblingParameters theDribblingParameters;

#include "Options.h"

  public:
    Behavior(internal::ref_init_tag t) : Libraries(t) {}

    /**
     * Constructor.
     * @param base The blackboard configured for this module.
     */
    Behavior(const BehaviorControlBase& base, BehaviorControlOutput& behaviorControlOutput)
        : Libraries(base, behaviorControlOutput) {
      InMapFile stream2("DribblingParameters.cfg");
      ASSERT(stream2.exists());
      stream2 >> theDribblingParameters;

      _theInstance = this;
    }

    /* Destructor. */
    ~Behavior() { _theInstance = 0; }

    /**
     * Executes one behavior cycle.
     * @param roots A set of root options. They must be parameterless.
     */
    void update(const std::vector<OptionInfos::Option>& roots) {
      theOwnTeamInfo = BehaviorControlBase::theOwnTeamInfo;
      theRobotInfo = BehaviorControlBase::theRobotInfo;
      theGameInfo = BehaviorControlBase::theGameInfo;
      theActivationGraph.graph.clear();

      registerDebugOutputs();

      preProcessLibraries();

      for (std::vector<Behavior::OptionInfos::Option>::const_iterator i = roots.begin(); i != roots.end(); ++i) {
        OptionInfos::execute(this, *i);
      }

      postProcessLibraries();

      _lastFrameTime = theFrameInfo.time;
    }

    /* debug output initialization for behaviour. every id for behavior needs to
     * be registered here. */
    inline void registerDebugOutputs() {
      DEBUG_RESPONSE_REGISTER("module:behavior:defender");
      DEBUG_RESPONSE_REGISTER("module:behavior:striker");
      DEBUG_RESPONSE_REGISTER("module:behavior:general");
      // usage inside behavior:
      // DEBUG_RESPONSE("module:behavior2013:general", OUTPUT(idText, text, "defender output"););
    }

    /**
     * The operator allocates a memory block that is zeroed.
     * Therefore, all members of this class are initialized with 0.
     * @attention This operator is only called if this class is instantiated by
     * a separate call to new, i.e. it cannot be created as a part of another class.
     * @param size The size of the block in bytes.
     * @return A pointer to the block.
     */
    static void* operator new(std::size_t size) { return calloc(1, size); }

    /**
     * The operator frees a memory block.
     * @param p The address of the block to free.
     */
    static void operator delete(void* p) { return free(p); }
  };

  PROCESS_WIDE_STORAGE(Behavior) Behavior::_theInstance;
  std::unordered_map<std::string, Behavior::OptionDescriptor*> Behavior::OptionInfos::optionsByName;
  std::vector<Behavior::OptionDescriptor> Behavior::OptionInfos::optionsByIndex;
  Behavior::OptionInfos collectOptions; /**< This global instantiation collects data about all options. */
} // namespace behavior

using namespace behavior;

/**
 * @class BehaviorControl
 * A C-based state machine behavior control module.
 */
class BehaviorControl : public BehaviorControlBase {
  STREAMABLE(Parameters,
  {
    /** Helper for streaming a vector of enums that are defined in another class. */
    struct OptionInfos : Behavior::OptionInfos {typedef std::vector<Option> Options;
},

  (OptionInfos, Options)roots, /**< All options that function as behavior roots. */
  (Vector2<>)kickOffsetPass, (bool)optimizeStrategy,
});

void update(BehaviorControlOutput& behaviorControlOutput) {
  Parameters p(parameters); // make a copy, to make "unchanged" work
  MODIFY("parameters:BehaviorControl", p);

  DECLARE_DEBUG_DRAWING("behavior:Field", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:Image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING3D("module:validRegion", "field");
  if (theFrameInfo.time) {
    theBehavior->update(p.roots);
  }
}

/** Updates the motion request by copying from behavior control output */
void update(MotionRequest& motionRequest) {
  motionRequest = theBehaviorControlOutput.motionRequest;
}

/** Updates the arm motion request by copying from behavior control output */
void update(ArmMotionRequest& armMotionRequest) {
  armMotionRequest = theBehaviorControlOutput.armMotionRequest;
}

/** Updates the head motion request by copying from behavior control output */
void update(HeadMotionRequest& headMotionRequest) {
  headMotionRequest = theBehaviorControlOutput.headMotionRequest;
}

/** Update the behavior led request by copying from behavior control output */
void update(BehaviorLEDRequest& behaviorLEDRequest) {
  behaviorLEDRequest = theBehaviorControlOutput.behaviorLEDRequest;
}

/** Update the behavior execution graph by copying from behavior control output */
void update(ActivationGraph& executionGraph) {
  executionGraph = theBehaviorControlOutput.executionGraph;
}

Parameters parameters; /**< The root options. */
Behavior* theBehavior; /**< The behavior with all options and libraries. */

public:
BehaviorControl() : theBehavior(new Behavior(*this, const_cast<BehaviorControlOutput&>(theBehaviorControlOutput))) {
  InMapFile stream("behaviorControl.cfg");
  ASSERT(stream.exists());
  stream >> parameters;
}

~BehaviorControl() {
  delete theBehavior;
}
}
;

MAKE_MODULE(BehaviorControl, Behavior Control)
