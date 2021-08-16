namespace sm_dance_bot
{
namespace s_pattern_states
{
// STATE DECLARATION
struct StiSPatternForward2 : public smacc::SmaccState<StiSPatternForward2, SS>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

      Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiSPatternRotate3>,
      Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StiSPatternRotate2>

      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateForward>(SS::pitch2_lenght_meters());
    configure_orthogonal<OrLED, CbLEDOn>();
  }

  void runtimeConfigure()
  {
  }
};
}  // namespace s_pattern_states
}  // namespace sm_dance_bot