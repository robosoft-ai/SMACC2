// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// the pragma once does not work here
// #pragma once

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER ros2

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "smacc2/smacc_tracing/trace_provider.hpp"

#if !defined(_TRACEPOINT_PROVIDER_PROVIDER_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _TRACEPOINT_PROVIDER_PROVIDER_H

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(ros2, spinOnce, TP_ARGS(), TP_FIELDS())

TRACEPOINT_EVENT(
  ros2, smacc2_event, TP_ARGS(const char *, event_type),
  TP_FIELDS(ctf_string(event_type, event_type)))

TRACEPOINT_EVENT(
  ros2, smacc2_state_update_start, TP_ARGS(const char *, updatable_element_name),
  TP_FIELDS(ctf_string(updatable_element_name, updatable_element_name)))

TRACEPOINT_EVENT(
  ros2, smacc2_state_update_end, TP_ARGS(const char *, updatable_element_name),
  TP_FIELDS(ctf_string(updatable_element_name, updatable_element_name)))

TRACEPOINT_EVENT(
  ros2, smacc2_state_onRuntimeConfigure_start, TP_ARGS(const char *, state_name),
  TP_FIELDS(ctf_string(state_name, state_name)))

TRACEPOINT_EVENT(
  ros2, smacc2_state_onRuntimeConfigure_end, TP_ARGS(const char *, state_name),
  TP_FIELDS(ctf_string(state_name, state_name)))

TRACEPOINT_EVENT(
  ros2, smacc2_state_onEntry_start, TP_ARGS(const char *, state_name),
  TP_FIELDS(ctf_string(state_name, state_name)))

TRACEPOINT_EVENT(
  ros2, smacc2_state_onEntry_end, TP_ARGS(const char *, state_name),
  TP_FIELDS(ctf_string(state_name, state_name)))

TRACEPOINT_EVENT(
  ros2, smacc2_state_onExit_start, TP_ARGS(const char *, state_name),
  TP_FIELDS(ctf_string(state_name, state_name)))

TRACEPOINT_EVENT(
  ros2, smacc2_state_onExit_end, TP_ARGS(const char *, state_name),
  TP_FIELDS(ctf_string(state_name, state_name)))

TRACEPOINT_EVENT(
  ros2, smacc2_client_behavior_on_entry_start,
  TP_ARGS(
    const char *, state_name, const char *, orthogonal_name, const char *, client_behavior_name),
  TP_FIELDS(ctf_string(state_name, state_name) ctf_string(orthogonal_name, orthogonal_name)
              ctf_string(client_behavior_name, client_behavior_name)))

TRACEPOINT_EVENT(
  ros2, smacc2_client_behavior_on_entry_end,
  TP_ARGS(
    const char *, state_name, const char *, orthogonal_name, const char *, client_behavior_name),
  TP_FIELDS(ctf_string(state_name, state_name) ctf_string(orthogonal_name, orthogonal_name)
              ctf_string(client_behavior_name, client_behavior_name)))

TRACEPOINT_EVENT(
  ros2, smacc2_client_behavior_on_exit_start,
  TP_ARGS(
    const char *, state_name, const char *, orthogonal_name, const char *, client_behavior_name),
  TP_FIELDS(ctf_string(state_name, state_name) ctf_string(orthogonal_name, orthogonal_name)
              ctf_string(client_behavior_name, client_behavior_name)))

TRACEPOINT_EVENT(
  ros2, smacc2_client_behavior_on_exit_end,
  TP_ARGS(
    const char *, state_name, const char *, orthogonal_name, const char *, client_behavior_name),
  TP_FIELDS(ctf_string(state_name, state_name) ctf_string(orthogonal_name, orthogonal_name)
              ctf_string(client_behavior_name, client_behavior_name)))

#endif /* _TRACEPOINT_PROVIDER_PROVIDER_H */

#include <lttng/tracepoint-event.h>
