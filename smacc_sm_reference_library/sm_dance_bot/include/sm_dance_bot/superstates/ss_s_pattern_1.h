#include <smacc/smacc.h>

namespace sm_dance_bot {
namespace SS5 {
namespace sm_dance_bot {
namespace s_pattern_states {

// FORWARD DECLARATIONS OF INNER STATES
class StiSPatternRotate1;
class StiSPatternForward1;
class StiSPatternRotate2;
class StiSPatternForward2;
class StiSPatternRotate3;
class StiSPatternForward3;
class StiSPatternRotate4;
class StiSPatternForward4;
class StiSPatternLoopStart;
} // namespace s_pattern_states
} // namespace sm_dance_bot

enum class TDirection
{
    LEFT,
    RIGHT
};

using namespace sm_dance_bot::s_pattern_states;

// STATE DECLARATION
struct SsSPattern1 : smacc::SmaccState<SsSPattern1, MsDanceBotRunMode, StiSPatternLoopStart>
{
public:
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<
    
    Transition<EvLoopEnd<StiSPatternLoopStart>, StRotateDegrees6, ENDLOOP>

    >reactions;

// STATE FUNCTIONS
    static void staticConfigure()
    {
        //configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
    }

    static constexpr float pitch1_lenght_meters() { return 0.75; }
    static constexpr float pitch2_lenght_meters() { return 1.45; }
    static constexpr int total_iterations() { return 11; }
    static constexpr TDirection direction() { return TDirection::RIGHT; }

    int iteration_count;

    void runtimeConfigure()
    {
        this->iteration_count = 0;
    }
};

// FORWARD DECLARATION FOR THE SUPERSTATE
using SS = SsSPattern1;
#include <sm_dance_bot/states/s_pattern_states/sti_spattern_rotate_1.h>
#include <sm_dance_bot/states/s_pattern_states/sti_spattern_forward_1.h>
#include <sm_dance_bot/states/s_pattern_states/sti_spattern_rotate_2.h>
#include <sm_dance_bot/states/s_pattern_states/sti_spattern_forward_2.h>
#include <sm_dance_bot/states/s_pattern_states/sti_spattern_rotate_3.h>
#include <sm_dance_bot/states/s_pattern_states/sti_spattern_forward_3.h>
#include <sm_dance_bot/states/s_pattern_states/sti_spattern_rotate_4.h>
#include <sm_dance_bot/states/s_pattern_states/sti_spattern_forward_4.h>
#include <sm_dance_bot/states/s_pattern_states/sti_spattern_loop_start.h>
} // namespace SS5
} // namespace sm_dance_bot