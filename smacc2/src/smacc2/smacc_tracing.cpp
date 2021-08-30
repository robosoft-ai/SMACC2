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

#include <smacc2/smacc_tracing/smacc_tracing.hpp>

#ifdef TRACETOOLS_LTTNG_ENABLED
#include <smacc2/smacc_tracing/trace_provider.hpp>
#define CONDITIONAL_TP(...) tracepoint(TRACEPOINT_PROVIDER, __VA_ARGS__)
#else
#define CONDITIONAL_TP(...)
#endif

void TRACEPOINT(spinOnce) { CONDITIONAL_TP(spinOnce); }

void TRACEPOINT(smacc_event, const char * event_type) { CONDITIONAL_TP(smacc_event, event_type); }

void TRACEPOINT(update_start, const char * updatable_element_name)
{
  CONDITIONAL_TP(update_start, updatable_element_name);
}

void TRACEPOINT(update_end, const char * updatable_element_name)
{
  CONDITIONAL_TP(update_end, updatable_element_name);
}

void TRACEPOINT(state_onRuntimeConfigure_start, const char * state_name)
{
  CONDITIONAL_TP(state_onRuntimeConfigure_start, state_name);
}

void TRACEPOINT(state_onRuntimeConfigure_end, const char * state_name)
{
  CONDITIONAL_TP(state_onRuntimeConfigure_end, state_name);
}

void TRACEPOINT(state_onEntry_start, const char * state_name)
{
  CONDITIONAL_TP(state_onEntry_start, state_name);
}

void TRACEPOINT(state_onEntry_end, const char * state_name)
{
  CONDITIONAL_TP(state_onEntry_end, state_name);
}

void TRACEPOINT(state_onExit_start, const char * state_name)
{
  CONDITIONAL_TP(state_onExit_start, state_name);
}

void TRACEPOINT(state_onExit_end, const char * state_name)
{
  CONDITIONAL_TP(state_onExit_end, state_name);
}

void TRACEPOINT(
  client_behavior_on_entry_start, const char * state_name, const char * orthogonal_name,
  const char * client_behavior_name)
{
  CONDITIONAL_TP(client_behavior_on_entry_start, state_name, orthogonal_name, client_behavior_name);
}

void TRACEPOINT(
  client_behavior_on_entry_end, const char * state_name, const char * orthogonal_name,
  const char * client_behavior_name)
{
  CONDITIONAL_TP(client_behavior_on_entry_end, state_name, orthogonal_name, client_behavior_name);
}

void TRACEPOINT(
  client_behavior_on_exit_start, const char * state_name, const char * orthogonal_name,
  const char * client_behavior_name)
{
  CONDITIONAL_TP(client_behavior_on_exit_start, state_name, orthogonal_name, client_behavior_name);
}

void TRACEPOINT(
  client_behavior_on_exit_end, const char * state_name, const char * orthogonal_name,
  const char * client_behavior_name)
{
  CONDITIONAL_TP(client_behavior_on_exit_end, state_name, orthogonal_name, client_behavior_name);
}
