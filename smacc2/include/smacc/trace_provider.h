#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER smacc_trace

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "smacc/trace_provider.h"

#if !defined(_HELLO_TP_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _SMACC_TRACE_PROVIDER_H

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(
    smacc_trace,
    spinOnce,
    TP_ARGS(
        int, my_integer_arg,
        char*, my_string_arg
    ),
    TP_FIELDS(
        ctf_string(my_string_field, my_string_arg)
        ctf_integer(int, my_integer_field, my_integer_arg)
    )
)

#endif /* _SMACC_TRACE_PROVIDER_H */

#include <lttng/tracepoint-event.h>