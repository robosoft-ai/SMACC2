// Copyright 2025 Robosoft Inc.
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
  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] asynchronous onEntry thread started");
  this->onEntryThread_ = std::async(
    std::launch::async,
    [=]
    {
      this->onEntry();
      this->postFinishEventFn_();
      RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] asynchronous onEntry thread finished");
      return 0;
    });
}

void SmaccAsyncClientBehavior::waitFutureIfNotFinished(
  std::optional<std::future<int>> & threadfut, bool requestFinish)
{
  if (!threadfut || !threadfut->valid())
  {
    RCLCPP_DEBUG_STREAM(
      getLogger(),
      "[" << getName() << "] onEntry thread not created or already joined. Skipping wait.");
    return;
  }

  try
  {
    // The wait does not stop at rclcpp shutdown: the thread must be joined before this object
    // can be released, or it finishes on a destroyed behavior. At shutdown the finish is
    // requested at once instead of after the usual grace lap.
    while (true)
    {
      // A thread that has already returned needs no finish request. Without this check every
      // shutdown logged a FATAL "requestForceFinish" for behaviors whose onEntry had long finished
      // but whose state was still active (the future stays valid until it is consumed here).
      if (threadfut->wait_for(std::chrono::seconds(0)) == std::future_status::ready)
      {
        threadfut->get();
        break;
      }

      if (requestFinish && !rclcpp::ok())
      {
        requestForceFinish();
      }

      auto status = threadfut->wait_for(std::chrono::milliseconds(20));
      if (status == std::future_status::ready)
      {
        threadfut->get();
        break;
      }

      RCLCPP_INFO_STREAM(
        getLogger(),
        "[" << getName()
            << "] fut valid but waiting for asynchronous onEntry thread to finish: state: "
            << (int)status);

      rclcpp::sleep_for(100ms);
      RCLCPP_WARN_THROTTLE(
        getLogger(), *(getNode()->get_clock()), 1000,
        "[%s] waiting for finishing client behavior, before leaving the state. Is the client "
        "behavior stuck? requesting force finish",
        this->getName().c_str());

      if (requestFinish) requestForceFinish();
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_DEBUG(
      getLogger(),
      "[SmaccAsyncClientBehavior] trying to join function, but it was already finished.");
  }
}

void SmaccAsyncClientBehavior::waitOnEntryThread(bool requestFinish)
{
  waitFutureIfNotFinished(this->onEntryThread_, requestFinish);
}

void SmaccAsyncClientBehavior::executeOnExit()
{
  RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] onExit - join async onEntry thread");
  this->waitOnEntryThread(true);

  RCLCPP_INFO_STREAM(
    getLogger(), "[" << getName() << "] onExit - Creating asynchronous onExit thread");
  this->onExitThread_ = std::async(
    std::launch::async,
    [=]
    {
      this->onExit();
      RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] asynchronous onExit done.");
      return 0;
    });
}

void SmaccAsyncClientBehavior::dispose()
{
  RCLCPP_DEBUG_STREAM(
    getLogger(), "[" << getName()
                     << "] Destroying client behavior- Waiting finishing of asynchronous onExit "
                        "thread");
  try
  {
    if (this->onExitThread_)
    {
      this->onExitThread_->get();
    }
  }
  catch (...)
  {
    RCLCPP_DEBUG(
      getLogger(),
      "[SmaccAsyncClientBehavior] trying to Join onExit function, but it was already "
      "finished.");
  }

  // Backstop: executeOnExit has normally joined the onEntry thread by now. No behavior may
  // be released with that thread live.
  this->waitOnEntryThread(true);

  RCLCPP_DEBUG_STREAM(
    getLogger(), "[" << getName()
                     << "] Destroying client behavior-  onExit thread finished. Proccedding "
                        "destruction.");
}

SmaccAsyncClientBehavior::~SmaccAsyncClientBehavior()
{
  // Last line of defense. Every release path joins the threads first (executeOnExit and
  // dispose on state exit, ISmaccStateMachine::joinAsyncClientBehaviorThreads at shutdown).
  // If one is missed, join here, before the members the thread uses are destroyed, rather
  // than let ~future block after them.
  if (this->onEntryThread_ && this->onEntryThread_->valid())
  {
    RCLCPP_ERROR_STREAM(
      getLogger(), "[" << getName() << "] destroyed with a live onEntry thread; joining it");
    this->waitOnEntryThread(true);
  }
  if (this->onExitThread_ && this->onExitThread_->valid())
  {
    RCLCPP_ERROR_STREAM(
      getLogger(), "[" << getName() << "] destroyed with a live onExit thread; joining it");
    try
    {
      this->onExitThread_->get();
    }
    catch (...)
    {
    }
  }
}

void SmaccAsyncClientBehavior::postSuccessEvent() { postSuccessEventFn_(); }

void SmaccAsyncClientBehavior::postFailureEvent() { postFailureEventFn_(); }

void SmaccAsyncClientBehavior::requestForceFinish()
{
  RCLCPP_FATAL_STREAM_THROTTLE(
    getLogger(), *(getNode()->get_clock()), 1000,
    "[" << getName() << "] " << ((uint64_t)this) << " requestForceFinish");
  isShutdownRequested_ = true;
}

bool SmaccAsyncClientBehavior::isShutdownRequested() { return isShutdownRequested_; }

}  // namespace smacc2
