namespace sm_respira_1
{
using namespace sm_respira_1::cmv_cycle_inner_states;

// STATE DECLARATION
struct SsCMVCycle : smacc::SmaccState<SsCMVCycle, MsRun, StiCMVCycleLoop>
{
public:
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvLoopEnd<StiCMVCycleLoop>, StObserve>

    >
    reactions;

  // STATE VARIABLES
  static constexpr int total_iterations() { return 1000; }
  int iteration_count = 0;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure() {}
};  // namespace SS4

}  // namespace sm_respira_1
