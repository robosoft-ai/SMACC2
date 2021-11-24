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

#include <smacc2/smacc_asynchronous_client_behavior.hpp>

using namespace std::chrono_literals;

namespace smacc2
{
void SmaccAsyncClientBehavior::executeOnEntry()
{
  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] Creating asynchronous onEntry thread");
  this->onEntryThread_ = std::async(std::launch::async, [=] {
    this->onEntry();
    this->postFinishEventFn_();
    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] onEntry asynchronous thread was finished.");
    return 0;
  });
}

void SmaccAsyncClientBehavior::waitFutureIfNotFinished(std::future<int> & threadfut)
{
  try
  {
    rclcpp::Rate r(100);
    while (rclcpp::ok())
    {
      bool valid = threadfut.valid();
      if (valid)
      {
        auto status = threadfut.wait_for(std::chrono::milliseconds(20));
        if (status == std::future_status::ready)
        {
          threadfut.get();
          break;
        }
      }

      //r.sleep();
      rclcpp::sleep_for(100ms);
      // rclcpp::spin_some(getNode());
      RCLCPP_WARN_THROTTLE(
        getLogger(), *(getNode()->get_clock()), 1000,
        "[%s] waiting for finishing client behavior, before leaving the state. Is the client "
        "behavior stuck?",
        demangleType(typeid(*this)).c_str());
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_DEBUG(
      getLogger(),
      "[SmaccAsyncClientBehavior] trying to join function, but it was already finished.");
  }
}

void SmaccAsyncClientBehavior::executeOnExit()
{
  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] onExit - join async onEntry thread");

  waitFutureIfNotFinished(this->onEntryThread_);

  RCLCPP_INFO_STREAM(
    getLogger(), "[" << getName() << "] onExit - Creating asynchronous onExit thread");
  this->onExitThread_ = std::async(std::launch::async, [=] {
    this->onExit();
    return 0;
  });
}

void SmaccAsyncClientBehavior::dispose()
{
  RCLCPP_DEBUG_STREAM(
    getLogger(),
    "[" << getName()
        << "] Destroying client behavior- Waiting finishing of asynchronous onExit thread");
  try
  {
    this->onExitThread_.get();
  }
  catch (...)
  {
    RCLCPP_DEBUG(
      getLogger(),
      "[SmaccAsyncClientBehavior] trying to Join onExit function, but it was already finished.");
  }

  RCLCPP_DEBUG_STREAM(
    getLogger(),
    "[" << getName()
        << "] Destroying client behavior-  onExit thread finished. Proccedding destruction.");
}

SmaccAsyncClientBehavior::~SmaccAsyncClientBehavior() {}

void SmaccAsyncClientBehavior::postSuccessEvent() { postSuccessEventFn_(); }

void SmaccAsyncClientBehavior::postFailureEvent() { postFailureEventFn_(); }

}  // namespace smacc2
