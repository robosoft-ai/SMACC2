/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <move_base_z_client_plugin/components/planner_switcher/planner_switcher.h>
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <smacc/smacc_asynchronous_client_behavior.h>

namespace cl_move_base_z
{
class CbMoveBaseClientBehaviorBase : public smacc::SmaccAsyncClientBehavior
{
public:
  virtual ~CbMoveBaseClientBehaviorBase();

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->requiresClient(moveBaseClient_);
    smacc::SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
    moveBaseClient_->onSucceeded(&CbMoveBaseClientBehaviorBase::propagateSuccessEvent, this);
    moveBaseClient_->onAborted(&CbMoveBaseClientBehaviorBase::propagateFailureEvent, this);
  }

protected:
  cl_move_base_z::ClMoveBaseZ * moveBaseClient_;

private:
  void propagateSuccessEvent(ClMoveBaseZ::WrappedResult &);
  void propagateFailureEvent(ClMoveBaseZ::WrappedResult &);
};
}  // namespace cl_move_base_z
