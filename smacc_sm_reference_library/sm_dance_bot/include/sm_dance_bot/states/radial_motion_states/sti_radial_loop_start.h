namespace sm_dance_bot
{
namespace radial_motion_states
{
// STATE DECLARATION
struct StiRadialLoopStart : smacc::SmaccState<StiRadialLoopStart, SS>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvLoopContinue<StiRadialLoopStart>, StiRadialRotate, CONTINUELOOP>

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
    checkWhileLoopConditionAndThrowEvent(&StiRadialLoopStart::loopWhileCondition);
  }
};
}  // namespace radial_motion_states
}  // namespace sm_dance_bot
