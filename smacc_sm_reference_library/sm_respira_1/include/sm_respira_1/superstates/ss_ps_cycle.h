namespace sm_respira_1
{
using namespace sm_respira_1::ps_cycle_inner_states;

// STATE DECLARATION
struct SsPSCycle : smacc::SmaccState<SsPSCycle, MsRun, StiPSCycleLoop>
{
public:
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvLoopEnd<StiPSCycleLoop>, StObserve>

    >
    reactions;

  // STATE VARIABLES
  static constexpr int total_iterations() { return 1000; }
  int iteration_count = 0;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure() {}
};

}  // namespace sm_respira_1
