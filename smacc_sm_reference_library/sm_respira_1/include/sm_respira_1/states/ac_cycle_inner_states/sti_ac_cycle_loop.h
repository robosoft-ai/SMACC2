namespace sm_respira_1
{
namespace ac_cycle_inner_states
{
// STATE DECLARATION
struct StiACCycleLoop : smacc::SmaccState<StiACCycleLoop, SsACCycle>
{
public:
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvLoopContinue<StiACCycleLoop>, StiACCycleInspire, CONTINUELOOP>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure() {}

  bool loopWhileCondition()
  {
    auto & superstate = this->context<SsACCycle>();

    RCLCPP_INFO(
      getLogger(), "Loop start, current iterations: %d, total iterations: %d",
      superstate.iteration_count, superstate.total_iterations());
    return superstate.iteration_count++ < superstate.total_iterations();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "LOOP START ON ENTRY");
    checkWhileLoopConditionAndThrowEvent(&StiACCycleLoop::loopWhileCondition);
  }
};
}  // namespace ac_cycle_inner_states
}  // namespace sm_respira_1
