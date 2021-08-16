#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <smacc/smacc.h>

namespace sm_dance_bot
{
// STATE DECLARATION
struct StNavigateForward1 : smacc::SmaccState<StNavigateForward1, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StRotateDegrees2>,
    Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StNavigateToWaypointsX, ABORT>
    //, Transition<EvActionPreempted<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX, PREEMPT>
    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateForward>(1);
    configure_orthogonal<OrLED, CbLEDOff>();
    configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }

  void runtimeConfigure()
  {
    ClMoveBaseZ * move_base_action_client;
    this->requiresClient(move_base_action_client);

    // we careful with the lifetime of the callbac, us a scoped connection if is not forever
    move_base_action_client->onSucceeded(&StNavigateForward1::onActionClientSucceeded, this);
  }

  void onActionClientSucceeded(cl_move_base_z::ClMoveBaseZ::WrappedResult & msg)
  {
    RCLCPP_INFO_STREAM(
      getNode()->get_logger(),
      " [Callback SmaccSignal] Success Detected from StAquireSensors (connected to client signal), "
      "result data: "
        << msg.result);
  }
};
}  // namespace sm_dance_bot