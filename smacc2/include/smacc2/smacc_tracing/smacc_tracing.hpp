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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <tracetools/tracetools.h>

#ifdef __cplusplus
extern "C"
{
#endif
  DECLARE_TRACEPOINT(spinOnce)

  DECLARE_TRACEPOINT(smacc2_event, const char * event_type)

  DECLARE_TRACEPOINT(smacc2_state_update_start, const char * updatable_element_name)

  DECLARE_TRACEPOINT(smacc2_state_update_end, const char * updatable_element_name)

  DECLARE_TRACEPOINT(smacc2_state_onRuntimeConfigure_start, const char * state_name)

  DECLARE_TRACEPOINT(smacc2_state_onRuntimeConfigure_end, const char * state_name)

  DECLARE_TRACEPOINT(smacc2_state_onEntry_start, const char * state_name)

  DECLARE_TRACEPOINT(smacc2_state_onEntry_end, const char * state_name)

  DECLARE_TRACEPOINT(smacc2_state_onExit_start, const char * state_name)

  DECLARE_TRACEPOINT(smacc2_state_onExit_end, const char * state_name)

  DECLARE_TRACEPOINT(
    smacc2_client_behavior_on_entry_start, const char * state_name, const char * orthogonal_name,
    const char * client_behavior_name)

  DECLARE_TRACEPOINT(
    smacc2_client_behavior_on_entry_end, const char * state_name, const char * orthogonal_name,
    const char * client_behavior_name)

  DECLARE_TRACEPOINT(
    smacc2_client_behavior_on_exit_start, const char * state_name, const char * orthogonal_name,
    const char * client_behavior_name)

  DECLARE_TRACEPOINT(
    smacc2_client_behavior_on_exit_end, const char * state_name, const char * orthogonal_name,
    const char * client_behavior_name)

#ifdef __cplusplus
}
#endif
