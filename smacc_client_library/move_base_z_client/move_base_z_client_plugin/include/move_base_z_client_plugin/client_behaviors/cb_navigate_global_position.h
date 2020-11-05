/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include "cb_move_base_client_behavior_base.h"
#include <boost/optional.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace cl_move_base_z
{
  class CbNavigateGlobalPosition : public CbMoveBaseClientBehaviorBase
  {
  public:
    boost::optional<geometry_msgs::msg::Point> goalPosition;
    boost::optional<float> goalYaw;
    boost::optional<float> yawTolerance;
    boost::optional<float> yawToleranceX;
    boost::optional<float> yawToleranceY;

    CbNavigateGlobalPosition();

    CbNavigateGlobalPosition(float x, float y, float yaw /*radians*/);

    void setGoal(const geometry_msgs::msg::Pose &pose);

    virtual void onEntry();

    // auxiliar function that defines the motion that is requested to the move_base action server
    void execute();

    void readStartPoseFromParameterServer(ClMoveBaseZ::Goal &goal);

    // This is the substate destructor. This code will be executed when the
    // workflow exits from this substate (that is according to statechart the moment when this object is destroyed)
    virtual void onExit() override;
  };
} // namespace cl_move_base_z
