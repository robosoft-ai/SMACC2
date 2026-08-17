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

#include <chrono>
#include <mutex>

#include <cl_keyboard/components/cp_keyboard_listener_1.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <smacc2/client_core_components/cp_topic_publisher.hpp>
#include <smacc2/smacc.hpp>
#include <smacc2/smacc_client_behavior.hpp>

namespace cl_keyboard
{

// Drives a Twist topic from the arrow keys (decoded by the keyboard server):
// Up/Down = +-linear step, Left/Right = +-angular step. Each arrow sets a pure
// motion (no combining); keyboard autorepeat keeps it alive while the key is
// held, and a deadman zeroes it on release. While no key is fresh, the
// configured idle twist is published instead - zero by default, or a constant
// "synthetic operator" push for unattended missions (e.g. driving the Nav2
// assisted_teleop collision guard without a human).
//
// The state machine's orthogonal must create the output topic component, e.g.
// createComponent<CpTopicPublisher<geometry_msgs::msg::Twist>>("/cmd_vel_teleop").
class CbKeyboardTwistTeleop : public smacc2::SmaccClientBehavior, public smacc2::ISmaccUpdatable
{
public:
  CbKeyboardTwistTeleop(
    float linearStep = 0.15f, float angularStep = 0.4f,
    geometry_msgs::msg::Twist idleTwist = geometry_msgs::msg::Twist())
  : linearStep_(linearStep), angularStep_(angularStep), idleTwist_(idleTwist)
  {
  }

  void onEntry() override
  {
    // sync behavior: onEntry runs on the state machine thread, so component
    // resolution and signal wiring are safe here
    this->requiresComponent(keyboardListener_);
    this->requiresComponent(twistPublisher_);

    if (keyboardListener_ != nullptr)
    {
      keyboardListener_->OnArrowPress(&CbKeyboardTwistTeleop::onArrowPress, this);
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "[%s] CpKeyboardListener1 not found", getName().c_str());
    }

    if (twistPublisher_ == nullptr)
    {
      RCLCPP_ERROR(
        getLogger(), "[%s] CpTopicPublisher<Twist> not found - create it in the orthogonal",
        getName().c_str());
    }
  }

  void onExit() override
  {
    // stop pushing: leave the topic at zero for whatever runs next
    if (twistPublisher_ != nullptr)
    {
      twistPublisher_->publish(geometry_msgs::msg::Twist());
    }
  }

  void update() override
  {
    if (twistPublisher_ == nullptr)
    {
      return;
    }

    geometry_msgs::msg::Twist out;
    bool held;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto now = std::chrono::steady_clock::now();
      held = lastArrowTime_ && (now - *lastArrowTime_) < deadman_;
      out = held ? heldTwist_ : idleTwist_;
    }
    twistPublisher_->publish(out);

    RCLCPP_INFO_THROTTLE(
      getLogger(), *getNode()->get_clock(), 2000,
      "[%s] publishing %s twist: linear=%.2f angular=%.2f", getName().c_str(),
      held ? "held" : "idle", out.linear.x, out.angular.z);
  }

private:
  void onArrowPress(ArrowKey arrow)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    heldTwist_ = geometry_msgs::msg::Twist();
    switch (arrow)
    {
      case ArrowKey::Up:
        heldTwist_.linear.x = linearStep_;
        break;
      case ArrowKey::Down:
        heldTwist_.linear.x = -linearStep_;
        break;
      case ArrowKey::Left:
        heldTwist_.angular.z = angularStep_;
        break;
      case ArrowKey::Right:
        heldTwist_.angular.z = -angularStep_;
        break;
    }
    lastArrowTime_ = std::chrono::steady_clock::now();
  }

  components::CpKeyboardListener1 * keyboardListener_ = nullptr;
  smacc2::client_core_components::CpTopicPublisher<geometry_msgs::msg::Twist> * twistPublisher_ =
    nullptr;

  float linearStep_;
  float angularStep_;
  geometry_msgs::msg::Twist idleTwist_;

  std::mutex mutex_;
  geometry_msgs::msg::Twist heldTwist_;
  std::optional<std::chrono::steady_clock::time_point> lastArrowTime_;

  // keyboard autorepeat (typically ~30 Hz after the initial delay) refreshes
  // lastArrowTime_ while a key is held; on release the twist dies within this
  // window. Long enough to bridge the repeat-start delay stutter is NOT
  // possible (that delay is ~500 ms) - accept one idle blip after the first
  // press or hold through it with the repeat stream
  std::chrono::milliseconds deadman_{350};
};

}  // namespace cl_keyboard
