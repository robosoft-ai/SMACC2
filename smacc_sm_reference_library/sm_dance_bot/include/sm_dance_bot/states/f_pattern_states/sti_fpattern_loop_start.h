namespace sm_dance_bot
{
namespace f_pattern_states
{
// STATE DECLARATION
template <typename SS>
struct StiFPatternStartLoop : smacc::SmaccState<StiFPatternStartLoop<SS>, SS>
{
  typedef SmaccState<StiFPatternStartLoop<SS>, SS> TSti;
  using TSti::context_type;
  using TSti::SmaccState;

// TRANSITION TABLE
  typedef mpl::list<
  
  Transition<EvLoopContinue<StiFPatternStartLoop<SS>>, StiFPatternRotate1<SS>, CONTINUELOOP> 
  
  >reactions;

// STATE FUNCTIONS
  static void staticConfigure()
  {
  }

  bool loopCondition()
  {
    auto &superstate = TSti::template context<SS>();
    return superstate.iteration_count++ < superstate.total_iterations();
  }

  void onEntry()
  {
    TSti::checkWhileLoopConditionAndThrowEvent(&StiFPatternStartLoop<SS>::loopCondition);
  }
};
} // namespace f_pattern_states
} // namespace sm_dance_bot