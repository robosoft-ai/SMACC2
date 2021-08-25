namespace sm_respira_1
{
using namespace sm_respira_1::pc_cycle_inner_states;

// STATE DECLARATION
struct SsPCCycle : smacc::SmaccState<SsPCCycle, MsRun, StiPCCycleLoop>
{
public:
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvLoopEnd<StiPCCycleLoop>, StObserve>

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
