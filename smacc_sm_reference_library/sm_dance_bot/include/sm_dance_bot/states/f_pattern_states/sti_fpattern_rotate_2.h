namespace sm_dance_bot
{
namespace f_pattern_states
{
// STATE DECLARATION
template <typename SS>
struct StiFPatternRotate2 : smacc::SmaccState<StiFPatternRotate2<SS>, SS>
{
  typedef SmaccState<StiFPatternRotate2<SS>, SS> TSti;
  using TSti::context_type;
  using TSti::SmaccState;

// TRANSITION TABLE
  typedef mpl::list<
  
  Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>, StiFPatternForward2<SS>> 
  
  >reactions;

// STATE FUNCTIONS
  static void staticConfigure()
  {
    double offset = 0; // for a better behaving
    // float angle = 0;
    
    // if (SS::direction() == TDirection::LEFT)
    //   angle = -90 - offset;
    // else
    //   angle = 90 + offset;

    //TSti::template configure_orthogonal<OrNavigation, CbRotate>(angle);
    TSti::template configure_orthogonal<OrNavigation, CbAbsoluteRotate>(0 + offset); // absolute horizontal
    TSti::template configure_orthogonal<OrLED, CbLEDOff>();
  }

  void runtimeConfigure()
  {
  }
};
} // namespace f_pattern_states
} // namespace sm_dance_bot