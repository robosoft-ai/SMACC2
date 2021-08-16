namespace sm_dance_bot_strikes_back
{
namespace s_pattern_states
{
// STATE DECLARATION
struct StiSPatternForward3 : public smacc::SmaccState<StiSPatternForward3, SS>
{
  using SmaccState::SmaccState;

// TRANSITION TABLE
  typedef mpl::list<
  
  Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiSPatternRotate4>,
  Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StiSPatternRotate3>
  
  >reactions;

// STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateForward>(SS::pitch1_lenght_meters());
    configure_orthogonal<OrLED, CbLEDOn>();
  }

  void runtimeConfigure()
  {
  }
};
} // namespace s_pattern_states
} // namespace sm_dance_bot_strikes_back