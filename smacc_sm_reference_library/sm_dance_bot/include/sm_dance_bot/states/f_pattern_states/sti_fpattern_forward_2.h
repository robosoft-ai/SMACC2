namespace sm_dance_bot
{
namespace f_pattern_states
{
// STATE DECLARATION
template <typename SS>
struct StiFPatternForward2 : smacc::SmaccState<StiFPatternForward2<SS>, SS>
{
  typedef SmaccState<StiFPatternForward2<SS>, SS> TSti;
  using TSti::SmaccState;
  using TSti::context_type;

// TRANSITION TABLE
  typedef mpl::list<
  
  Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiFPatternStartLoop<SS>> 
  
  >reactions;

// STATE FUNCTIONS
  static void staticConfigure()
  {
  }
  
  void runtimeConfigure()
  {
    auto &superstate = TSti::template context<SS>();
    RCLCPP_INFO(this->getNode()->get_logger(),"[SsrFpattern] Fpattern rotate: SS current iteration: %d/%d", superstate.iteration_count, superstate.total_iterations());

    TSti::template configure<OrNavigation, CbNavigateForward>(SS::pitch_lenght_meters());
    TSti::template configure<OrLED, CbLEDOff>();
  }
};
}
}