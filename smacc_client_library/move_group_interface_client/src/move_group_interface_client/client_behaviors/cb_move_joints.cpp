/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_move_joints.h>
#include <future>

namespace cl_move_group_interface
{
  CbMoveJoints::CbMoveJoints(const std::map<std::string, double> &jointValueTarget) : jointValueTarget_(jointValueTarget)
  {
  }

  CbMoveJoints::CbMoveJoints()
  {
  }

  void CbMoveJoints::onEntry()
  {
    this->requiresClient(movegroupClient_);

    if (this->group_)
    {
      moveit::planning_interface::MoveGroupInterface move_group(*(this->group_));
      this->moveJoints(move_group);
    }
    else
    {
      this->moveJoints(movegroupClient_->moveGroupClientInterface);
    }
  }

  std::string currentJointStatesToString(moveit::planning_interface::MoveGroupInterface &moveGroupInterface, std::map<std::string, double>& targetJoints)
  {
    auto state = moveGroupInterface.getCurrentState();
    auto vnames = state->getVariableNames();
    
    std::stringstream ss;

    for(auto& tgj: targetJoints)
    {
      auto it = std::find(vnames.begin(),vnames.end(), tgj.first);
      auto index = std::distance(vnames.begin(), it);

      ss << tgj.first << ":" << state->getVariablePosition(index) << std::endl;
    }

    return ss.str();
  }

  void CbMoveJoints::moveJoints(moveit::planning_interface::MoveGroupInterface &moveGroupInterface)
  {
    if (scalingFactor_)
      moveGroupInterface.setMaxVelocityScalingFactor(*scalingFactor_);

    bool success;
    moveit::planning_interface::MoveGroupInterface::Plan computedMotionPlan;

    if (jointValueTarget_.size() == 0)
    {
      RCLCPP_WARN(nh_->get_logger(), "[CbMoveJoints] No joint was value specified. Skipping planning call.");
      success = false;
    }
    else
    {
      moveGroupInterface.setJointValueTarget(jointValueTarget_);
      //moveGroupInterface.setGoalJointTolerance(0.01);
      success = (moveGroupInterface.plan(computedMotionPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO_NAMED(nh_->get_logger(), "CbMoveJoints", "Success Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    }

    if (success)
    {
      auto executionResult = moveGroupInterface.execute(computedMotionPlan);

      auto statestr = currentJointStatesToString(moveGroupInterface, jointValueTarget_);
      
      if (executionResult == moveit_msgs::MoveItErrorCodes::SUCCESS)
      {
        RCLCPP_INFO_STREAM(nh_->get_logger(), "[" << this->getName() << "] motion execution succedded. Throwing success event. " << std::endl
                                                                                              << statestr);
        movegroupClient_->postEventMotionExecutionSucceded();
        this->postSuccessEvent();
      }
      else
      {
        RCLCPP_WARN_STREAM(nh_->get_logger(), "[" << this->getName() << "] motion execution failed. Throwing fail event." << std::endl
                                                                                       << statestr);
        movegroupClient_->postEventMotionExecutionFailed();
        this->postFailureEvent();
      }
    }
    else
    {
      auto statestr = currentJointStatesToString(moveGroupInterface, jointValueTarget_);
      RCLCPP_WARN_STREAM(nh_->get_logger(), "[" << this->getName() << "] motion execution failed. Throwing fail event." << std::endl << statestr);
      movegroupClient_->postEventMotionExecutionFailed();
      this->postFailureEvent();
    }
  }

  void CbMoveJoints::onExit()
  {
  }
} // namespace cl_move_group_interface