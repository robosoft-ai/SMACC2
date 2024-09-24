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

using smacc2::client_behaviors::CbSequence;
using namespace cl_keyboard;

// STATE DECLARATION
struct StBothArmsMove : smacc2::SmaccState<StBothArmsMove, SmMultiPandaSim>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct start_sequence_a : SUCCESS{};
  struct start_sequence_b : SUCCESS{};
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<
      smacc2::Transition<EvCbSuccess<CbMoveJoints, OrArmLeft>, StLeftArmMoves, SUCCESS>,
      smacc2::Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StLeftArmMoves, NEXT>
      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {


    std::map<std::string, double> rightJointValues{ { "right_panda_joint1", -0.3 },
                                                     { "right_panda_joint2", -0.3 },
                                                     { "right_panda_joint3", -0.3 },
                                                     { "right_panda_joint4", -0.3 } };


    std::map<std::string, double> leftJointValues{
                                                    { "left_panda_joint1", -0.3 },
                                                    { "left_panda_joint2", -0.3 },
                                                    { "left_panda_joint3", -0.3 },
                                                    { "left_panda_joint4", -0.3 },
                                                  };

    configure_orthogonal<OrArmRight, CbMoveJoints>(rightJointValues);
    configure_orthogonal<OrArmLeft, CbMoveJoints>(leftJointValues);
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
    // configure_orthogonal<OrArmRight, CbSleepFor>(15s);
  }

  void runtimeConfigure()
  {
    // std::map<std::string, double> leftJointValues{
    //   { "left_panda_joint1", 0.3 },
    //   { "left_panda_joint2", 0.3 },
    //   { "left_panda_joint3", 0.3 },
    //   { "left_panda_joint4", 0.3 },
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
    RCLCPP_INFO(getLogger(), "On Exit!");
  }
};
}  // namespace sm_multi_panda_sim
