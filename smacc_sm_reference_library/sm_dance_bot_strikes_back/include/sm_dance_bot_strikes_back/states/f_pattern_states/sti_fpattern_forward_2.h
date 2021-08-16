namespace sm_dance_bot_strikes_back
{
namespace f_pattern_states
{
// STATE DECLARATION
template <typename SS>
struct StiFPatternForward2 : smacc::SmaccState<StiFPatternForward2<SS>, SS>
{
  typedef SmaccState<StiFPatternForward2<SS>, SS> TSti;
  using TSti::context_type;
  using TSti::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiFPatternRotate2<SS>>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure()
  {
    // auto &superstate = TSti::template context<SS>();

    TSti::template configure<OrNavigation, CbNavigateForward>(SS::pitch_lenght_meters());
    TSti::template configure<OrLED, CbLEDOff>();
  }
};
}  // namespace f_pattern_states
}  // namespace sm_dance_bot_strikes_back