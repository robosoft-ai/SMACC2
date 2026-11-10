// Copyright 2026 RobosoftAI Inc.
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
#include <thread>

#include <smacc2/smacc.hpp>
#include <smacc2/smacc_updatable.hpp>
#include <std_msgs/msg/u_int32.hpp>

namespace sm_coretest_callback_rate_1
{
// Publishes on its own thread at publish_rate_hz, subscribes to the same topic and runs a node
// timer at timer_rate_hz; update() (polling thread) reports the counts of the last second.
class ClCallbackCounter : public smacc2::ISmaccClient, public smacc2::ISmaccUpdatable
{
public:
  void onInitialize() override
  {
    auto node = getNode();
    node->declare_parameter<double>("publish_rate_hz", 200.0);
    node->declare_parameter<double>("timer_rate_hz", 100.0);
    node->get_parameter("publish_rate_hz", publishRateHz_);
    node->get_parameter("timer_rate_hz", timerRateHz_);
    RCLCPP_INFO(
      getLogger(), "[ClCallbackCounter] publishing at %.1f Hz, node timer at %.1f Hz",
      publishRateHz_, timerRateHz_);

    pub_ = node->create_publisher<std_msgs::msg::UInt32>("/callback_rate_test", 10);
    sub_ = node->create_subscription<std_msgs::msg::UInt32>(
      "/callback_rate_test", 10, [this](std_msgs::msg::UInt32::SharedPtr) { received_++; });
    timer_ = node->create_wall_timer(
      std::chrono::duration<double>(1.0 / timerRateHz_), [this]() { ticks_++; });

    start_ = std::chrono::steady_clock::now();
    lastReport_ = start_;
    publisherThread_ = std::thread([this]() { publishLoop(); });
  }

  ~ClCallbackCounter() override
  {
    stop_ = true;
    if (publisherThread_.joinable())
    {
      publisherThread_.join();
    }
  }

  // Polling thread, once per signal detector iteration.
  void update() override
  {
    iterations_++;
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> since = now - lastReport_;
    if (since.count() >= 1.0)
    {
      uint32_t published = published_, received = received_, ticks = ticks_,
               iterations = iterations_;
      RCLCPP_INFO(
        getLogger(),
        "[ClCallbackCounter] last %.2f s: published %u, received %u, timer ticks %u, loop "
        "iterations %u (totals: published %u, received %u, ticks %u, iterations %u)",
        since.count(), published - lastPublished_, received - lastReceived_, ticks - lastTicks_,
        iterations - lastIterations_, published, received, ticks, iterations);
      lastPublished_ = published;
      lastReceived_ = received;
      lastTicks_ = ticks;
      lastIterations_ = iterations;
      lastReport_ = now;
    }
  }

private:
  void publishLoop()
  {
    const std::chrono::duration<double> period(1.0 / publishRateHz_);
    auto next = std::chrono::steady_clock::now();
    std_msgs::msg::UInt32 msg;
    while (!stop_ && rclcpp::ok())
    {
      msg.data = published_;
      pub_->publish(msg);
      published_++;
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      std::this_thread::sleep_until(next);
    }
  }

  double publishRateHz_ = 200.0;
  double timerRateHz_ = 100.0;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<uint32_t> published_{0}, received_{0}, ticks_{0}, iterations_{0};
  uint32_t lastPublished_ = 0, lastReceived_ = 0, lastTicks_ = 0, lastIterations_ = 0;
  std::chrono::steady_clock::time_point start_, lastReport_;
  std::atomic<bool> stop_{false};
  std::thread publisherThread_;
};
}  // namespace sm_coretest_callback_rate_1
