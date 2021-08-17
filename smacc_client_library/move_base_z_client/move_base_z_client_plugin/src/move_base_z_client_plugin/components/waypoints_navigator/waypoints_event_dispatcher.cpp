#include <move_base_z_client_plugin/components/waypoints_navigator/waypoints_event_dispatcher.h>

namespace cl_move_base_z
{
void WaypointEventDispatcher::postWaypointEvent(int index)
{
    auto& fn = postWaypointFn[index % WAYPOINTS_EVENTCOUNT];
    if(fn!=nullptr)
        fn();
}
} // namespace smacc