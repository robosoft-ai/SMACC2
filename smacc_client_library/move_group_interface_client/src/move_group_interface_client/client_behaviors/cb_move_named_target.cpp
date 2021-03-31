/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_move_named_target.h>

namespace cl_move_group_interface
{
CbMoveNamedTarget::CbMoveNamedTarget(std::string namedtarget)
: namedTarget_(namedtarget)
{
}

void CbMoveNamedTarget::onEntry()
{
    this->requiresClient(movegroupClient_);
    movegroupClient_->moveGroupClientInterface.setNamedTarget(this->namedTarget_);
}

void CbMoveNamedTarget::onExit()
{
}

std::map<std::string, double> CbMoveNamedTarget::getNamedTargetValues()
{
    return movegroupClient_->moveGroupClientInterface.getNamedTargetValues(this->namedTarget_);
}
}  // namespace cl_move_group_interface
