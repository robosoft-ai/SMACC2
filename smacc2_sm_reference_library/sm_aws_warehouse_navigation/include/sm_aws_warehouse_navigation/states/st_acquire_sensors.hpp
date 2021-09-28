#include <smacc2/smacc.hpp>

namespace sm_aws_warehouse_navigation
{
// STATE DECLARATION
struct StAcquireSensors : smacc2::SmaccState<StAcquireSensors, SmAwsWarehouseNavigation>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
        Transition<EvCbSuccess<CbWaitPose, OrNavigation>, StInitialNavigateForward, SUCCESS> 
        >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure() { configure_orthogonal<OrNavigation, CbWaitPose>(); }

  void runtimeConfigure() {}

  void onEntry() {}

  void onExit() {}
};
}  // namespace sm_aws_warehouse_navigation
