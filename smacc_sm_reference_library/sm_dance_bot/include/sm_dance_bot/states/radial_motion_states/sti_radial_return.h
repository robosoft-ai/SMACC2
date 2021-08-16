namespace sm_dance_bot
{
namespace radial_motion_states
{
// STATE DECLARATION
struct StiRadialReturn : smacc::SmaccState<StiRadialReturn, SS>
{
  using SmaccState::SmaccState;
  
// TRANSITION TABLE
  typedef mpl::list<
  
  Transition<EvCbSuccess<CbUndoPathBackwards, OrNavigation>, StiRadialLoopStart, SUCCESS>,
  Transition<EvCbFailure<CbUndoPathBackwards, OrNavigation>, StiRadialEndPoint, ABORT>
  
  >reactions;

// STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbUndoPathBackwards>();
    configure_orthogonal<OrLED, CbLEDOff>();
  }

  void onExit()
  {
      ClMoveBaseZ* moveBase;
      this->requiresClient(moveBase);

      auto odomTracker = moveBase->getComponent<cl_move_base_z::odom_tracker::OdomTracker>();
      odomTracker->clearPath();
  }
};
}
}