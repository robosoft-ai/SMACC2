namespace sm_dance_bot
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

    Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiFPatternStartLoop<SS>>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    TSti::template configure_orthogonal<OrNavigation, CbNavigateForward>(SS::pitch_lenght_meters());
    TSti::template configure_orthogonal<OrLED, CbLEDOff>();
  }

  void runtimeConfigure() {}
};
}  // namespace f_pattern_states
}  // namespace sm_dance_bot
