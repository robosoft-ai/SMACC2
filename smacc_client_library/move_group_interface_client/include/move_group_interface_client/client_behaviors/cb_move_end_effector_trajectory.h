/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc/smacc_asynchronous_client_behavior.h>
#include <move_group_interface_client/cl_movegroup.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf/transform_datatypes.h>

namespace cl_move_group_interface
{
  template <typename AsyncCB, typename Orthogonal>
  struct EvJointDiscontinuity : sc::event<EvJointDiscontinuity<AsyncCB, Orthogonal>>
  {
    moveit_msgs::msg::RobotTrajectory trajectory;
  };

  template <typename AsyncCB, typename Orthogonal>
  struct EvIncorrectInitialPosition : sc::event<EvIncorrectInitialPosition<AsyncCB, Orthogonal>>
  {
    moveit_msgs::msg::RobotTrajectory trajectory;
  };

  enum class ComputeJointTrajectoryErrorCode
  {
    SUCCESS,
    INCORRECT_INITIAL_STATE,
    JOINT_TRAJECTORY_DISCONTINUITY
  };

  class CbMoveEndEffectorTrajectory : public smacc::SmaccAsyncClientBehavior,
                                      public smacc::ISmaccUpdatable
  {
  public:
    // std::string tip_link_;
    boost::optional<std::string> group_;

    boost::optional<std::string> tipLink_;

    boost::optional<bool> allowInitialTrajectoryStateJointDiscontinuity_;

    CbMoveEndEffectorTrajectory(std::string tipLink = "");

    CbMoveEndEffectorTrajectory(const std::vector<geometry_msgs::msgs::PoseStamped> &endEffectorTrajectory, std::string tipLink = "");

    template <typename TOrthogonal, typename TSourceObject>
    void onOrthogonalAllocation()
    {
      smacc::SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();

      postJointDiscontinuityEvent = [this](auto traj) {
        auto ev = new EvJointDiscontinuity<TSourceObject, TOrthogonal>();
        ev->trajectory = traj;
        this->postEvent(ev);
      };

      postIncorrectInitialStateEvent = [this](auto traj) 
      {
        auto ev = new EvIncorrectInitialPosition<TSourceObject, TOrthogonal>();
        ev->trajectory = traj;
        this->postEvent(ev);
      };

      postMotionExecutionFailureEvents = [this]
      {
          RCLCPP_INFO_STREAM(nh_->get_logger(), "[" << this->getName() << "] motion execution failed");
          movegroupClient_->postEventMotionExecutionFailed();
          this->postEvent<EvMoveGroupMotionExecutionFailed<TSourceObject,TOrthogonal>>();
      };
    }    

    virtual void onEntry() override;

    virtual void onExit() override;

    virtual void update() override;

  protected:
    ComputeJointTrajectoryErrorCode computeJointSpaceTrajectory(moveit_msgs::msgs::RobotTrajectory &computedJointTrajectory);
    
    void executeJointSpaceTrajectory(const moveit_msgs::msgs::RobotTrajectory &computedJointTrajectory);

    virtual void generateTrajectory();

    virtual void createMarkers();

    std::vector<geometry_msgs::msgs::PoseStamped> endEffectorTrajectory_;

    ClMoveGroup *movegroupClient_ = nullptr;

    visualization_msgs::msgs::MarkerArray beahiorMarkers_;

    void getCurrentEndEffectorPose(std::string globalFrame, tf::StampedTransform& currentEndEffectorTransform);

  private:
    void initializeROS();

    ros::Publisher markersPub_;

    std::atomic<bool> markersInitialized_;

    ros::ServiceClient iksrv_;

    std::mutex m_mutex_;

    std::function<void(moveit_msgs::msgs::RobotTrajectory &)> postJointDiscontinuityEvent;
    std::function<void(moveit_msgs::msgs::RobotTrajectory &)> postIncorrectInitialStateEvent;

    std::function<void()> postMotionExecutionFailureEvents;

    bool autocleanmarkers = false;
  };
} // namespace cl_move_group_interface