//#pragma once

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER smacc_trace

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "smacc/trace_provider.h"

#if !defined(_SMACC_TRACE_PROVIDER_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _SMACC_TRACE_PROVIDER_H

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(
    smacc_trace,
    spinOnce,
    TP_ARGS(
        int, my_integer_arg,
        const char*, my_string_arg
    ),
    TP_FIELDS(
        ctf_string(my_string_field, my_string_arg)
        ctf_integer(int, my_integer_field, my_integer_arg)
    ))

TRACEPOINT_EVENT(
    smacc_trace,
    update_start,
    TP_ARGS(),
    TP_FIELDS()
)

TRACEPOINT_EVENT(
    smacc_trace,
    update_end,
    TP_ARGS(),
    TP_FIELDS()
)

TRACEPOINT_EVENT(
    smacc_trace,
    state_onRuntimeConfigure_start,
    TP_ARGS(const char*, state_name),
    TP_FIELDS(
        ctf_string(state_name, state_name)
    )
)

TRACEPOINT_EVENT(
    smacc_trace,
    state_onRuntimeConfigure_end,
    TP_ARGS(const char*, state_name),
    TP_FIELDS(
        ctf_string(state_name, state_name)
    )
)

TRACEPOINT_EVENT(
    smacc_trace,
    state_onEntry_start,
    TP_ARGS(const char*, state_name),
    TP_FIELDS(
        ctf_string(state_name, state_name)
    )
)

TRACEPOINT_EVENT(
    smacc_trace,
    state_onEntry_end,
    TP_ARGS(const char*, state_name),
    TP_FIELDS(
        ctf_string(state_name, state_name)
    )
)

TRACEPOINT_EVENT(
    smacc_trace,
    state_onExit_start,
    TP_ARGS(const char*, state_name),
    TP_FIELDS(
        ctf_string(state_name, state_name)
    )
)

TRACEPOINT_EVENT(
    smacc_trace,
    state_onExit_end,
    TP_ARGS(const char*, state_name),
    TP_FIELDS(
        ctf_string(state_name, state_name)
    )
)

TRACEPOINT_EVENT(
    smacc_trace,
    client_behavior_on_entry_start,
    TP_ARGS(
        const char*, state_name,
        const char*, orthogonal_name,
        const char*, client_behavior_name),
    TP_FIELDS(
        ctf_string(state_name, state_name)
        ctf_string(orthogonal_name, orthogonal_name)
        ctf_string(client_behavior_name, client_behavior_name)
    )
)

TRACEPOINT_EVENT(
    smacc_trace,
    client_behavior_on_entry_end,
    TP_ARGS(
        const char*, state_name,
        const char*, orthogonal_name,
        const char*, client_behavior_name),
    TP_FIELDS(
        ctf_string(state_name, state_name)
        ctf_string(orthogonal_name, orthogonal_name)
        ctf_string(client_behavior_name, client_behavior_name)
    )
)

TRACEPOINT_EVENT(
    smacc_trace,
    client_behavior_on_exit_start,
    TP_ARGS(
        const char*, state_name,
        const char*, orthogonal_name,
        const char*, client_behavior_name),
    TP_FIELDS(
        ctf_string(state_name, state_name)
        ctf_string(orthogonal_name, orthogonal_name)
        ctf_string(client_behavior_name, client_behavior_name)
    )
)

TRACEPOINT_EVENT(
    smacc_trace,
    client_behavior_on_exit_end,
    TP_ARGS(
        const char*, state_name,
        const char*, orthogonal_name,
        const char*, client_behavior_name),
    TP_FIELDS(
        ctf_string(state_name, state_name)
        ctf_string(orthogonal_name, orthogonal_name)
        ctf_string(client_behavior_name, client_behavior_name)
    )
)

#endif /* _SMACC_TRACE_PROVIDER_H */

#include <lttng/tracepoint-event.h>