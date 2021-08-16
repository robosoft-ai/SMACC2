namespace sm_dance_bot_strikes_back
{
namespace s_pattern_states
{
// STATE DECLARATION
struct StiSPatternLoopStart : smacc::SmaccState<StiSPatternLoopStart, SS>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvLoopContinue<StiSPatternLoopStart>, StiSPatternRotate1, CONTINUELOOP>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure() {}

  bool loopCondition()
  {
    auto & superstate = this->context<SS>();
    if (superstate.iteration_count++ < superstate.total_iterations())
    {
      RCLCPP_INFO(
        getNode()->get_logger(), "Spattern iteration finished, going to next iteration (%d)",
        superstate.iteration_count);
      return true;
    }
    else
    {
      RCLCPP_INFO(
        getNode()->get_logger(), "Spattern iteration finished, All iterations finished (%d)",
        superstate.iteration_count - 1);
      return false;
    }
  }

  void onEntry() { checkWhileLoopConditionAndThrowEvent(&StiSPatternLoopStart::loopCondition); }
};

}  // namespace s_pattern_states
}  // namespace sm_dance_bot_strikes_back