#include <tracetools/tracetools.h>

#ifdef __cplusplus
extern "C"
{

#endif
DECLARE_TRACEPOINT(spinOnce)

DECLARE_TRACEPOINT(smacc_event, const char* event_type)

DECLARE_TRACEPOINT(update_start, const char* updatable_element_name)

DECLARE_TRACEPOINT(update_end, const char* updatable_element_name)

DECLARE_TRACEPOINT(state_onRuntimeConfigure_start, const char* state_name)

DECLARE_TRACEPOINT(state_onRuntimeConfigure_end, const char* state_name)

DECLARE_TRACEPOINT(state_onEntry_start, const char* state_name)

DECLARE_TRACEPOINT(state_onEntry_end, const char* state_name)

DECLARE_TRACEPOINT(state_onExit_start, const char* state_name)

DECLARE_TRACEPOINT(state_onExit_end, const char* state_name)

DECLARE_TRACEPOINT(client_behavior_on_entry_start,
                 const char* state_name, const char* orthogonal_name, const char* client_behavior_name)

DECLARE_TRACEPOINT(client_behavior_on_entry_end,
                 const char* state_name, const char* orthogonal_name, const char* client_behavior_name)

DECLARE_TRACEPOINT(client_behavior_on_exit_start,
                 const char* state_name, const char* orthogonal_name, const char* client_behavior_name)

DECLARE_TRACEPOINT(client_behavior_on_exit_end,
                 const char* state_name, const char* orthogonal_name, const char* client_behavior_name)


#ifdef __cplusplus
}
#endif