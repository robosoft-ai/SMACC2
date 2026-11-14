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

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <example_interfaces/action/fibonacci.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <smacc2/smacc.hpp>

namespace sm_coretest_action_client_1
{
// A Fibonacci action server on the state machine's own node, so the action client component runs
// against a real server with no second process. The goal's order selects the outcome:
//   kRejectOrder  the goal is rejected
//   kAbortOrder   the goal is accepted and aborted after two steps
//   any other     accepted; one feedback every 50 ms, success after `order` steps, or a cancelled
//                 result as soon as the client cancels
class CpFibonacciServer : public smacc2::ISmaccComponent
{
public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

  static constexpr int kRejectOrder = 13;
  static constexpr int kAbortOrder = 21;
  static constexpr const char * kActionName = "/coretest_fibonacci";

  ~CpFibonacciServer() override
  {
    if (context_) context_->remove_pre_shutdown_callback(preShutdownHandle_);
    stop();
  }

  void onInitialize() override
  {
    // Every goal must reach a terminal state while the context is still valid: a goal handle
    // released later tries to publish its result and throws from its destructor.
    context_ = getNode()->get_node_base_interface()->get_context();
    preShutdownHandle_ = context_->add_pre_shutdown_callback([this] { this->stop(); });

    server_ = rclcpp_action::create_server<Fibonacci>(
      getNode(), kActionName,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal> goal)
      {
        std::lock_guard<std::mutex> lock(workersMutex_);
        if (stopping_) return rclcpp_action::GoalResponse::REJECT;
        if (goal->order == kRejectOrder)
        {
          RCLCPP_INFO(getLogger(), "[CpFibonacciServer] goal rejected");
          return rclcpp_action::GoalResponse::REJECT;
        }
        ++acceptsPending_;  // stop() waits for the accepted callback that follows
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const std::shared_ptr<GoalHandle> goalHandle)
      {
        // Executed off the executor thread, as an action server must.
        std::lock_guard<std::mutex> lock(workersMutex_);
        --acceptsPending_;
        workers_.emplace_back([this, goalHandle] { this->execute(goalHandle); });
        acceptsDone_.notify_all();
      });
  }

private:
  // Ends every goal in flight (each worker aborts its own) and waits for the workers.
  void stop()
  {
    std::vector<std::thread> workers;
    {
      std::unique_lock<std::mutex> lock(workersMutex_);
      stopping_ = true;
      // A goal accepted just before this point gets its worker first, so it is aborted like the rest.
      acceptsDone_.wait_for(lock, std::chrono::seconds(2), [this] { return acceptsPending_ == 0; });
      workers.swap(workers_);
    }
    for (auto & worker : workers)
    {
      if (worker.joinable()) worker.join();
    }
  }

  void execute(const std::shared_ptr<GoalHandle> goalHandle)
  {
    // The context can shut down under this thread; a call into rclcpp then throws.
    try
    {
      const int order = goalHandle->get_goal()->order;
      auto result = std::make_shared<Fibonacci::Result>();
      auto feedback = std::make_shared<Fibonacci::Feedback>();
      for (int step = 0; step < order; ++step)
      {
        if (stopping_)
        {
          goalHandle->abort(result);
          return;
        }
        if (goalHandle->is_canceling())
        {
          goalHandle->canceled(result);
          RCLCPP_INFO(getLogger(), "[CpFibonacciServer] goal cancelled at step %d", step);
          return;
        }
        if (order == kAbortOrder && step == 2)
        {
          goalHandle->abort(result);
          RCLCPP_INFO(getLogger(), "[CpFibonacciServer] goal aborted");
          return;
        }
        feedback->sequence.push_back(step);
        goalHandle->publish_feedback(feedback);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      result->sequence = feedback->sequence;
      goalHandle->succeed(result);
      RCLCPP_INFO(getLogger(), "[CpFibonacciServer] goal succeeded");
    }
    catch (const std::exception &)
    {
    }
  }

  rclcpp_action::Server<Fibonacci>::SharedPtr server_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::PreShutdownCallbackHandle preShutdownHandle_;
  std::atomic<bool> stopping_{false};
  std::mutex workersMutex_;
  std::condition_variable acceptsDone_;
  int acceptsPending_ = 0;
  std::vector<std::thread> workers_;
};
}  // namespace sm_coretest_action_client_1
