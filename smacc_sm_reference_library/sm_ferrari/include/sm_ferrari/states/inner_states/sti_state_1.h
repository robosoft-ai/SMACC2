namespace sm_ferrari
{
namespace inner_states
{
// STATE DECLARATION
struct StiState1 : smacc::SmaccState<StiState1, SS>
{
public:
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvLoopContinue<StiState1>, StiState2, CONTINUELOOP>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure() {}

  bool loopWhileCondition()
  {
    auto & superstate = this->context<SS>();

    RCLCPP_INFO(
      getNode()->get_logger(), "Loop start, current iterations: %d, total iterations: %d",
      superstate.iteration_count, superstate.total_iterations());
    return superstate.iteration_count++ < superstate.total_iterations();
  }

  void onEntry()
  {
    RCLCPP_INFO(getNode()->get_logger(), "LOOP START ON ENTRY");
    checkWhileLoopConditionAndThrowEvent(&StiState1::loopWhileCondition);
  }

  void onExit() { RCLCPP_INFO(getNode()->get_logger(), "On Exit!"); }
};
}  // namespace inner_states
}  // namespace sm_ferrari