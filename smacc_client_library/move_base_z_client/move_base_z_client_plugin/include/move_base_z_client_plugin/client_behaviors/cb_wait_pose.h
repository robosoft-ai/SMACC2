/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <smacc/smacc_asynchronous_client_behavior.h>

namespace cl_move_base_z
{
// waits a robot pose message. Usually used for the startup synchronization.
class CbWaitPose : public smacc::SmaccAsyncClientBehavior
{
public:
  CbWaitPose();
  virtual ~CbWaitPose();

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->requiresClient(moveBaseClient_);
    smacc::SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }

  void onEntry() override;

protected:
  cl_move_base_z::ClMoveBaseZ * moveBaseClient_;
};
}  // namespace cl_move_base_z