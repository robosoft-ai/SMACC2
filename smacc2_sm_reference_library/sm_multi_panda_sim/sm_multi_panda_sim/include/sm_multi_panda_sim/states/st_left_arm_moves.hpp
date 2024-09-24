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

#include <smacc2/client_behaviors/cb_sequence.hpp>

namespace sm_multi_panda_sim
{
using namespace std::chrono_literals;
using smacc2::client_behaviors::CbSequence;
using namespace cl_keyboard;

// STATE DECLARATION
struct StLeftArmMoves : smacc2::SmaccState<StLeftArmMoves, SmMultiPandaSim>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct start_sequence_a : SUCCESS{};
  struct start_sequence_b : SUCCESS{};
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<
      smacc2::Transition<EvCbSuccess<CbMoveJoints, OrArmLeft>, StRightArmMoves, SUCCESS>,
      // smacc2::Transition<EvCbSuccess<CbSleepFor, OrArmLeft>, StRightArmMoves, SUCCESS>

      smacc2::Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StRightArmMoves, NEXT>

      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {


    std::map<std::string, double> leftJointValues{
                                                    { "left_panda_joint1", 0.1 },
                                                    { "left_panda_joint2", 0.1 },
                                                    { "left_panda_joint3", 0.1 },
                                                    { "left_panda_joint4", 0.1 },
                                                  };

    configure_orthogonal<OrArmLeft, CbMoveJoints>(leftJointValues);
    // configure_orthogonal<OrArmLeft, CbSleepFor>(15s);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure()
  {
    this->getClientBehavior<OrArmLeft, CbMoveJoints>()->scalingFactor_=100;

    // std::map<std::string, double> leftJointValues{
    //   { "left_panda_joint1", 0.1 },
    //   { "left_panda_joint2", 0.1 },
    //   { "left_panda_joint3", 0.1 },
    //   { "left_panda_joint4", 0.1 },
    //   // {"shoulder_pan_joint", 0.0},
    //   // {"wrist_1_joint", M_PI / 4},
    //   // {"wrist_2_joint", 0.0},
    //   // {"wrist_3_joint", 0.0}
    // };

    // std::map<std::string, double> leftJointValues2{
    //   { "left_panda_joint1", -0.3 },
    //   { "left_panda_joint2", -0.3 },
    //   { "left_panda_joint3", -0.3 },
    //   { "left_panda_joint4", -0.3 },
    //   // {"shoulder_pan_joint", 0.0},
    //   // {"wrist_1_joint", M_PI / 4},
    //   // {"wrist_2_joint", 0.0},
    //   // {"wrist_3_joint", 0.0}
    // };

    // auto cbSequence = this->template getClientBehavior<OrArmLeft, CbSequence>();

    // cbSequence->then<OrArmLeft, CbMoveJoints>(leftJointValues)
    //     ->then<OrArmLeft, CbMoveJoints>(leftJointValues2)
    //     ->then<OrArmLeft, CbMoveJoints>(leftJointValues)
    //     ->then<OrArmLeft, CbMoveJoints>(leftJointValues2);
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "On Entry!");
  }

  void onExit()
  {
    // cl_moveit2z::MoveGroupInterface* move_group_client_;
    // requiresClient(move_group_client_);
    // move_group_client_->stop();
  }
};
}  // namespace sm_multi_panda_sim
