// Copyright 2021 MyName/MyCompany Inc.
// Copyright 2021 RobosoftAI Inc. (template)
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
 *****************************************************************************************************************/

#pragma once

namespace sm_multi_panda_sim
{
// SMACC2 classes
using smacc2::EvStateRequestFinish;
using smacc2::Transition;
using smacc2::default_transition_tags::SUCCESS;
using namespace smacc2;
using namespace cl_move_group_interface;


// STATE DECLARATION
struct StMoveJoints : smacc2::SmaccState<StMoveJoints, SmMultiPandaSim>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef boost::mpl::list<
    // Transition<EvCbSuccess<CbMoveJoints, OrArmLeft>, StMoveEndEffector, SUCCESS>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // std::map<std::string, double> leftJointValues{
    //     {"left_panda_joint1", 0.0},
    //     {"left_panda_joint2", 0.0},
    //     {"left_panda_joint3", 0.0},
    //     {"left_panda_joint4", 0.0},
    //     {"left_panda_joint5", M_PI/2.0},
    //     {"left_panda_joint7", 0.0},
    //     {"left_panda_joint6", 0.0},
    //     {"right_panda_joint1", 0.0},
    //     {"right_panda_joint2", M_PI/2.0},
    //     {"right_panda_joint3", 0.0},
    //     {"right_panda_joint4", 0.0},
    //     {"right_panda_joint5", 0.0},
    //     {"right_panda_joint6", 0.0},
    //     {"right_panda_joint7", 0.0},

    //   };

    // configure_orthogonal<OrArmLeft, CbMoveJoints>(leftJointValues);

    geometry_msgs::msg::PoseStamped endeffector1pose;
    endeffector1pose.pose.orientation.w = 1;
    endeffector1pose.header.frame_id = "left_panda_link8";

    geometry_msgs::msg::PoseStamped endeffector2pose;
    endeffector2pose.pose.orientation.w = 1;
    endeffector2pose.header.frame_id = "right_panda_link8";

    //configure_orthogonal<OrArmLeft, CbMoveSynchronizedLinkGoals>(std::vector<geometry_msgs::msg::PoseStamped>{endeffector1pose},std::vector<std::string>{endeffector1pose.header.frame_id});
    configure_orthogonal<OrArmLeft, CbMoveSynchronizedLinkGoals>(std::vector<geometry_msgs::msg::PoseStamped>{endeffector2pose},std::vector<std::string>{endeffector2pose.header.frame_id});

//    configure_orthogonal<OrArmLeft, CbMoveSynchronizedLinkGoals>(std::vector<geometry_msgs::msg::PoseStamped>{endeffector1pose, endeffector2pose},std::vector<std::string>{endeffector1pose.header.frame_id, endeffector2pose.header.frame_id});

    // std::map<std::string, double> rightJointValues{
    //     {"right_panda_joint1", 0.0},
    //     {"right_panda_joint2", 0.0},
    //     {"right_panda_joint3", 0.0},
    //     {"right_panda_joint4", 0.0},
    //     {"right_panda_joint5", 0.0},
    //     {"right_panda_joint7", 0.0},
    //     {"right_panda_joint6", 0.0},
    //     // {"right_panda_leftfinger", 0.0},
    //     // {"right_panda_rightfinger", 0.0}
    //   };

    // configure_orthogonal<OrArmRight, CbMoveJoints>(rightJointValues);
  };

  void runtimeConfigure()
  {
    // ClMoveGroup * moveGroupClientLeft;
    // this->requiresClient(moveGroupClientLeft);
    // this->getOrthogonal<OrArmLeft>()->getClientBehavior<CbMoveJoints>()->scalingFactor_ = 0.1;

    // ClMoveGroup * moveGroupClientRight;
    // this->requiresClient(moveGroupClientRight);
    // this->getOrthogonal<OrArmRight>()->getClientBehavior<CbMoveJoints>()->scalingFactor_ = 0.1;
  }
};
}  // namespace sm_multi_panda_sim
